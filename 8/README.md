# 辐射度量学

###### Intensity

power per solid angle

###### Radiant Energy

$$
Q\text{[J=Joule]}
$$

###### Radiant Flux (Power)

#photons flowing through a sensor in unit time
$$
\Phi\equiv\frac{\text{d}Q}{\text{d}t}\text{[{W=Watt}][lm=lumen]}^*
$$

$$
I(w)\equiv\frac{\text{d}\Phi}{\text{d}w}
$$

$$
[\frac{\text{W}}{\text{sr}}][\frac{\text{lm}}{\text{sr}}=\text{cd=candela}]
$$

###### Angle

$$
\theta=\frac{l}{r}
$$

###### Solid angle

$$
\Omega=\frac{A}{r^2}
$$

###### Differential Solid Angle

$$
\begin{align}
\text{d}A&=(r\text{d}\theta)(rsin\theta{\text{d}\phi})\\
&=r^2sin\theta{\text{d}\theta}\text{d}\phi
\end{align}
$$

$$
\text{d}w=\frac{\text{d}A}{r^2}=sin\theta\text{d}\theta\text{d}\phi
$$

###### Isotropic Point Source

$$
\begin{align}
\Phi&=\int_{S^2}I\text{d}w\\
&=4\pi{I}
\end{align}
$$

$$
I=\frac{\Phi}{4\pi}
$$

###### Irradiance 

power per projected unit area
$$
E(x)\equiv\frac{\text{d}\Phi(x)}{\text{d}A}
$$

$$
[\frac{\text{W}}{\text{m}^2}][\frac{\text{lm}}{\text{m}^2}=\text{lux}]
$$

###### Radiant Intensity

light emitted from a source

###### Radiance

1. power per unit solid angle, per projected unit area
2. Irradiance per solid angle
3. Intensity per projected unit area

$$
L(p,w)\equiv{\frac{\text{d}^2\Phi(p,w)}{\text{d}w\text{d}Acos\theta}}
$$

$$
[\frac{\text{W}}{\text{srm}^2}][\frac{\text{cd}}{\text{m}^2}=\frac{\text{lm}}{\text{srm}^2}=\text{nit}]
$$



# 双向反射分布函数

BRDF represents how much ligh is reflected into each outgoing direction w_r from each incoming direction
$$
f_r(w_i\rightarrow{w_r})=\frac{\text{d}L_r(w_r)}{\text{d}E_i(w_i)}=\frac{\text{d}L_r(w_r)}{L_i(w_i)cos\theta_i\text{d}w_i}\left[\frac{1}{\text{sr}}\right]
$$
The Reflection Equation
$$
L_r(p,w_r)=\int_{H^2}f_r(p,w_i\rightarrow{w_r})L_i(p,w_i)cos\theta_i\text{d}w_i
$$
The Rendering Equation
$$
L_o(p,w_o)=L_e(p,w_o)+\int_{\Omega^+}L_i(p,w_i)f_r(p,w_i,w_o)(n\cdot{w_i})\text{d}w_i
$$
Linear Operator Equation
$$
\begin{align}
L&=E+KL\\
IL-KL&=E\\
(I-K)L&=E\\
L&=(I-K)^{-1}E\\
L&=(I+K+K^2+K^3+\cdots)E\\
L&=E+KE+K^2E+K^3E+\cdots
\end{align}
$$

# Monte Carlo 积分



# 路径追踪



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

```c++

```

