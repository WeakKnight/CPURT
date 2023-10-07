# CPURT

a handy cpu ray tracing kernel based on [BVH](https://github.com/madmann91/bvh).

### C++
```cpp
Context* cpurt_init(float* trisPtr, int triNum);

void cpurt_dispatch_rays(Context* context, RayDesc* rayDescs, int rayCount, HitInfo* results);

void cpurt_release(Context* context);
```

### Unity C#
``` csharp
IntPtr Init(Mesh mesh);

void DispatchRays(IntPtr context, NativeArray<RayDesc> rays, NativeArray<HitInfo> hitInfos);

void Release(IntPtr context);
```

### Performance
On an AMD Ryzen 9 5950X, it takes 0.2ms to shoot 512 rays at a mesh consisting of 15,000 triangles.
