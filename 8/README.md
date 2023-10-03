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

$$
\int{f(x)}dx=\frac{1}{N}\sum^N_{i=1}\frac{f(X_i)}{p(X_i)}\ \ \ \ X_i\sim{p(x)}
$$

###### Definite integral

$$
\int_a^bf(x)dx
$$

###### Random variable

$$
X_i\sim{p(x)}
$$

###### Monte Carlo estimator

$$
F_N=\frac{1}{N}\sum^N_{i=1}\frac{f(X_i)}{p(X_i)}
$$

# 光线追踪 (路径追踪)

###### Whitted Style Ray Tracing Problem

1. Glossy reflection != Mirror reflection
2. No reflections between diffuse materials

###### A Simple Monte Carlo Solution

$$
L_o(p,w_o)=L_e(p,w_o)+\int_{\Omega^+}L_i(p,w_i)f_r(p,w_i,w_o)(n\cdot{w_i})\text{d}w_i
$$

assume all directions are pointing outwards
$$
L_o(p,w_o)=\int_{\Omega^+}L_i(p,w_i)f_r(p,w_i,w_o)(n\cdot{w_i})\text{d}w_i
$$

$$
\begin{align}
f(x)&=L_i(p,w_i)f_r(p,w_i,w_o)(n\cdot{w_i})\\
p(w_i)&=1/2\pi
\end{align}
$$

$$
\begin{align}
\int{f(x)}dx&=\frac{1}{N}\sum^N_{i=1}\frac{f(X_i)}{p(X_i)}\ \ \ \ X_i\sim{p(x)}\\
L_o(p,w_o)&=\frac{1}{N}\sum_{i=1}^{N}\frac{L_i(p,w_i)f_r(p,w_i,w_o)(n\cdot{w_i})}{p(w_i)}
\end{align}
$$

###### Russian Roulette

$$
E=P*(L_o/P)+(1-P)*0=L_o
$$

###### Ray Generation 

N (ssp samples per pixel)

```c++
ray_generation(camPos, pixel)
    Uniformly choose N sample positions within the pixel
    pixel_radiance = 0.0
    For each sample in the pixel
    	Shoot a ray r(camPos, cam_to_sample)
    	If ray r hit the scene at p
    		pixel_radiuance += 1 / N * shade(p, sample_to_cam)
    Return pixel_radiance
```

###### Sampling the Light

$$
\begin{align}
\int{\text{pdf}}\ \text{d}A&=1\\
\text{pdf}&=1/A
\end{align}
$$

$$
\begin{align}
L_o&=\int{L_i}f_rcos\text{d}w\\
\text{d}w&=\frac{\text{d}Acos\theta'}{||x'-x||^2}
\end{align}
$$

$$
\begin{align}
L_o(p,w_o)&=\int_{\Omega^+}L_i(x,w_i)f_r(x,w_i,w_o)cos\theta\text{d}w_i\\
&=\int_{A}L_i(x,w_i)f_r(x,w_i,w_o)\frac{cos\theta{cos\theta'}}{||x'-x||^2}\text{d}A
\end{align}
$$

###### Path Tracing

```c++
shade(p, wo)
    # Contribution from the light source.
    Uniformly sample the light at x' (pdf_light = 1 / A)
    L_dir = L_i * f_r * cosθ * cosθ' / |x' - p|^2 / pdf_light
    
    # Contribution from other reflectors.
    L_indir = 0.0
    Test Russian Roulette with probability P_RR
    Uniformly sample the hemisphere toward wi (pdf_hemi = 1 / 2pi)
    Trace a ray r(p, wi)
    If ray r hit a non-emitting object at q
    	L_indir = shade(q, -wi) * f_r * cosθ / pdf_hemi / P_RR
   	
    Return L_dir + L_indir
```

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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection intersection = Scene::intersect(ray);
    if(intersection.happened) {

        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal;
        Material *m = intersection.m;
  
        Vector3f L_dir(0.0), L_indir(0.0);

        Intersection intersection_light;
        float pdf_light;
        sampleLight(intersection_light, pdf_light);
        Vector3f dir_p_x = (intersection_light.coords - hitPoint).normalized();
        Ray ray_p_x(hitPoint + EPSILON * N, dir_p_x);
        Intersection intersection_p_x = Scene::intersect(ray_p_x);
        if(intersection_p_x.happened && intersection_p_x.m->hasEmission()) {
            Vector3f NN = intersection_p_x.normal;
            L_dir = intersection_p_x.m->m_emission * m->eval(ray.direction, dir_p_x, N) * dotProduct(dir_p_x, N) * dotProduct(-dir_p_x, NN) / intersection_p_x.distance / pdf_light;
        }

        if(get_random_float() <= RussianRoulette) {
            Vector3f dir_i = m->sample(ray.direction, N).normalized();
            Ray ray_p_diri(hitPoint, dir_i);
            Intersection intersection_p_diri = Scene::intersect(ray_p_diri);
            if(intersection_p_diri.happened && !intersection_p_diri.m->hasEmission()) {
                L_indir = castRay(ray_p_diri, depth+1) * m->eval(ray.direction, dir_i, N) * dotProduct(dir_i, N) / m->pdf(ray.direction, dir_i, N) / RussianRoulette;
            }
        }

        return m->getEmission() + L_dir + L_indir;
    } 
    else 
        return Vector3f(0,0,0);
}
```
