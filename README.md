# CPURT

a handy cpu-based ray tracing kernel based on [BVH](https://github.com/madmann91/bvh).

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
