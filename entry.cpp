#include "bvh.h"
#include "vec.h"
#include "ray.h"
#include "node.h"
#include "default_builder.h"
#include "thread_pool.h"
#include "executor.h"
#include "stack.h"
#include "tri.h"

#include <iostream>
#include <span>

#ifdef WIN32
#define CPURT_API __declspec(dllexport) 
#else
#define CPURT_API
#endif

using Scalar = float;
using Vec3 = v2::Vec<Scalar, 3>;
using BBox = v2::BBox<Scalar, 3>;
using Tri = v2::Tri<Scalar, 3>;
using Node = v2::Node<Scalar, 3>;
using Bvh = v2::Bvh<Node>;
using Ray = v2::Ray<Scalar, 3>;

using PrecomputedTri = v2::PrecomputedTri<Scalar>;

struct RayDesc
{
	float ox;
	float oy;
	float oz;
	float tmin;
	float dx;
	float dy;
	float dz;
	float tmax;
};

struct HitInfo
{
	int hit;
	float t;
	int primIdx;
	float baryX;

	float baryY;
	int hitBackFace;
	float pad1;
	float pad2;
};

struct Context
{
	std::vector<BBox> bboxes;
	std::vector<Vec3> centers;
	std::vector<PrecomputedTri> precomputed_tris;
	v2::Bvh<Node> bvh;
	v2::ThreadPool thread_pool;
	v2::ParallelExecutor executor;

	Context(int triNum)
		:
		bboxes(triNum),
		centers(triNum),
		precomputed_tris(triNum),
		executor(thread_pool)
	{
	}
};

// Permuting the primitive data allows to remove indirections during traversal, which makes it faster.
static constexpr bool sShouldPermute = true;

extern "C"
{
	CPURT_API Context* cpurt_init(float* trisPtr, int triNum)
	{
		Context* context = new Context(triNum);

		auto getTri = [&](int triIdx)
		{
			return Tri(
				Vec3(trisPtr[(triIdx * 3 + 0) * 3 + 0], trisPtr[(triIdx * 3 + 0) * 3 + 1], trisPtr[(triIdx * 3 + 0) * 3 + 2]),
				Vec3(trisPtr[(triIdx * 3 + 1) * 3 + 0], trisPtr[(triIdx * 3 + 1) * 3 + 1], trisPtr[(triIdx * 3 + 1) * 3 + 2]),
				Vec3(trisPtr[(triIdx * 3 + 2) * 3 + 0], trisPtr[(triIdx * 3 + 2) * 3 + 1], trisPtr[(triIdx * 3 + 2) * 3 + 2]));
		};

		// Get triangle centers and bounding boxes (required for BVH builder)

		context->executor.for_each(0, triNum, [&](size_t begin, size_t end) {
			for (size_t i = begin; i < end; ++i) {
				context->bboxes[i] = getTri(i).get_bbox();
				context->centers[i] = getTri(i).get_center();
			}
			});

		typename v2::DefaultBuilder<Node>::Config config;
		config.quality = v2::DefaultBuilder<Node>::Quality::High;
		context->bvh = v2::DefaultBuilder<Node>::build(context->thread_pool, context->bboxes, context->centers, config);

		context->executor.for_each(0, triNum, [&](size_t begin, size_t end)
			{
				for (size_t i = begin; i < end; ++i)
				{
					auto j = sShouldPermute ? context->bvh.prim_ids[i] : i;
					context->precomputed_tris[i] = getTri(j);
				}
			});

		return context;
	}

	CPURT_API void cpurt_dispatch_rays(Context* context, RayDesc* rayDescs, int rayCount, HitInfo* results)
	{
		context->executor.for_each(0llu, (size_t)rayCount, [&](size_t begin, size_t end)
			{
				for (size_t rayIdx = begin; rayIdx < end; ++rayIdx)
				{
					HitInfo hitInfo;

					RayDesc rayDesc = rayDescs[rayIdx];
					Ray ray = Ray
					{
						Vec3(rayDesc.ox, rayDesc.oy, rayDesc.oz), // Ray origin
						Vec3(rayDesc.dx, rayDesc.dy, rayDesc.dz), // Ray direction
						rayDesc.tmin,               // Minimum intersection distance
						rayDesc.tmax                // Maximum intersection distance
					};

					static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
					static constexpr size_t stack_size = 64;
					static constexpr bool use_robust_traversal = false;

					auto prim_id = invalid_id;
					Scalar u, v;
					int hitBackFace;

					// Traverse the BVH and get the u, v coordinates of the closest intersection.
					v2::SmallStack<Bvh::Index, stack_size> stack;
					context->bvh.intersect<false, use_robust_traversal>(ray, context->bvh.get_root().index, stack,
						[&](size_t begin, size_t end)
						{
							for (size_t i = begin; i < end; ++i)
							{
								size_t j = sShouldPermute ? i : context->bvh.prim_ids[i];
								if (auto hit = context->precomputed_tris[j].intersect(ray))
								{
									prim_id = i;
									hitBackFace = v2::dot(context->precomputed_tris[j].n, -ray.dir) > 0.0f ? 1 : 0;
									std::tie(u, v) = *hit;
								}
							}
							return prim_id != invalid_id;
						});

					if (prim_id != invalid_id)
					{
						hitInfo.hit = 1;
						hitInfo.t = ray.tmax;
						hitInfo.primIdx = (int)prim_id;
						hitInfo.baryX = u;
						hitInfo.baryY = v;
						hitInfo.hitBackFace = hitBackFace;
					}
					else
					{
						hitInfo.hit = 0;
					}

					results[rayIdx] = hitInfo;
				}
			});

		context->thread_pool.wait();
	}

	CPURT_API void cpurt_release(Context* context)
	{
		if (context != nullptr)
		{
			delete context;
		}
	}
}