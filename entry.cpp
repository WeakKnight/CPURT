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

#define CPURT_API __declspec(dllexport) 

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
	float dx;
	float dy;
	float dz;
	float tmin;
	float tmax;
};

struct HitInfo
{
	int hit;
	float t;
	int primIdx;
	float baryX;

	float baryY;
	float pad0;
	float pad1;
	float pad2;
};

extern "C"
{
	CPURT_API void dispatch_rays(float* trisPtr, int triNum, RayDesc* rayDescs, int rayCount, HitInfo* results)
	{
		v2::ThreadPool thread_pool;
		v2::ParallelExecutor executor(thread_pool);

		std::vector<Tri> tris(triNum);
		executor.for_each(0, tris.size(), [&](size_t begin, size_t end) 
		{
			for (size_t triIdx = begin; triIdx < end; ++triIdx)
			{
				tris[triIdx] = Tri(
					Vec3(trisPtr[(triIdx * 3 + 0) * 3 + 0], trisPtr[(triIdx * 3 + 0) * 3 + 1], trisPtr[(triIdx * 3 + 0) * 3 + 2]),
					Vec3(trisPtr[(triIdx * 3 + 1) * 3 + 0], trisPtr[(triIdx * 3 + 1) * 3 + 1], trisPtr[(triIdx * 3 + 1) * 3 + 2]),
					Vec3(trisPtr[(triIdx * 3 + 2) * 3 + 0], trisPtr[(triIdx * 3 + 2) * 3 + 1], trisPtr[(triIdx * 3 + 2) * 3 + 2]));
			}
		});

		// Get triangle centers and bounding boxes (required for BVH builder)
		std::vector<BBox> bboxes(tris.size());
		std::vector<Vec3> centers(tris.size());
		executor.for_each(0, tris.size(), [&](size_t begin, size_t end) {
			for (size_t i = begin; i < end; ++i) {
				bboxes[i] = tris[i].get_bbox();
				centers[i] = tris[i].get_center();
			}
			});

		typename v2::DefaultBuilder<Node>::Config config;
		config.quality = v2::DefaultBuilder<Node>::Quality::High;
		auto bvh = v2::DefaultBuilder<Node>::build(thread_pool, bboxes, centers, config);

		// Permuting the primitive data allows to remove indirections during traversal, which makes it faster.
		static constexpr bool should_permute = true;

		// This precomputes some data to speed up traversal further.
		std::vector<PrecomputedTri> precomputed_tris(tris.size());
		executor.for_each(0, tris.size(), [&](size_t begin, size_t end) {
			for (size_t i = begin; i < end; ++i) {
				auto j = should_permute ? bvh.prim_ids[i] : i;
				precomputed_tris[i] = tris[j];
			}
			});

		std::vector<Ray> rays(rayCount);
		executor.for_each(0llu, (size_t)rayCount, [&](size_t begin, size_t end) {
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

				// Traverse the BVH and get the u, v coordinates of the closest intersection.
				v2::SmallStack<Bvh::Index, stack_size> stack;
				bvh.intersect<false, use_robust_traversal>(ray, bvh.get_root().index, stack,
					[&](size_t begin, size_t end) {
						for (size_t i = begin; i < end; ++i) {
							size_t j = should_permute ? i : bvh.prim_ids[i];
							if (auto hit = precomputed_tris[j].intersect(ray)) {
								prim_id = i;
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
				}
				else
				{
					hitInfo.hit = 0;
				}

				results[rayIdx] = hitInfo;
			}});
	}
}