# 作业 7

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

###### Bound3.hpp

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
    if(t_enter <= t_exit && t_exit >=0)
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

###### Scene.cpp

