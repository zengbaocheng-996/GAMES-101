# 包围盒

###### Ray Intersection with Axis-Aligned Box

For each pair, calculate the t_min and t_max
$$
\begin{align}
t_{enter}&=max\{t_{min}\}\\
t_{exit}&=min\{t_{max}\}
\end{align}
$$
Ray and AABB intersect iff
$$
\begin{align}
t_{enter}&<t_{exit}\\
t_{exit}&>=0
\end{align}
$$

###### Bounding Volume Hierarchy

1. Find bounding box
2. Recursively split set of objects in two subsets
3. Recompute the bounding box of the subsets
4. Stop when necessary
5. Store objects in each leaf node

```c++
Intersect(Ray ray, BVH node){
    if(ray misses node.bbox) return;
    
    if(node is a leaf node)
    	test intersection with all objs;
    	return closest intersection;
    
    hit1 = Intersect(ray, node.child1);
    hit2 = Intersect(ray, node.child2);
    
    return the closer of hit1, hit2;
}
```

# 作业 6

###### Renderer.cpp

```c++
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(-1, 5, 10);
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            Vector3f dir = normalize(Vector3f(x, y, -1));
            Ray ray(eye_pos, dir);
            framebuffer[m++] = scene.castRay(ray, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
```

###### Triangle.hpp

```c++
inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;

    if (dotProduct(ray.direction, normal) > 0)
        return inter;
    double u, v, t_tmp = 0;
    Vector3f pvec = crossProduct(ray.direction, e2);
    double det = dotProduct(e1, pvec);
    if (fabs(det) < EPSILON)
        return inter;

    double det_inv = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = dotProduct(tvec, pvec) * det_inv;
    if (u < 0 || u > 1)
        return inter;
    Vector3f qvec = crossProduct(tvec, e1);
    v = dotProduct(ray.direction, qvec) * det_inv;
    if (v < 0 || u + v > 1)
        return inter;
    t_tmp = dotProduct(e2, qvec) * det_inv;

    inter.happened = true;
    auto view2intersection = t_tmp * ray.direction;
    inter.coords = ray.origin + view2intersection;
    inter.normal = this->normal;
    inter.distance = dotProduct(view2intersection, view2intersection);
    inter.obj = this;
    inter.m = this->m;

    return inter;
}
```

###### Bounds3.hpp

```c++
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    float t_enter;
    float t_exit;
    Vector3f t_enter_v3f = (pMin - ray.origin) * invDir;
    Vector3f t_exit_v3f = (pMax - ray.origin) * invDir;
    if(!dirIsNeg[0])
        std::swap(t_enter_v3f.x, t_exit_v3f.x);
    if(!dirIsNeg[1])
        std::swap(t_enter_v3f.y, t_exit_v3f.y);
    if(!dirIsNeg[2])
        std::swap(t_enter_v3f.z, t_exit_v3f.z);
    t_enter = std::max(t_enter_v3f.x, std::max(t_enter_v3f.y, t_enter_v3f.z));
    t_exit = std::min(t_exit_v3f.x, std::min(t_exit_v3f.y, t_exit_v3f.z));
    if(t_enter < t_exit && t_exit >=0)
        return true;
    else
        return false;
}
```

###### BVH.cpp

```c++
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection isect;
    auto direction = ray.direction;
    if(!node->bounds.IntersectP(ray, ray.direction_inv, std::array<int, 3>{direction.x>0, direction.y>0, direction.z>0}))
        return isect;
    if(node->object != nullptr)
        return node->object->getIntersection(ray);
    Intersection isect_left, isect_right;
    isect_left = getIntersection(node->left, ray);
    isect_right = getIntersection(node->right, ray);
    return isect_left.distance <= isect_right.distance ? isect_left : isect_right;
}
```

