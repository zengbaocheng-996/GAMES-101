# 光线追踪

###### Light Rays

1. Light travels in straight lines
2. Light rays do not "collide" with each other if they cross
3. Light rays travel from the light sources to the eye (reciprocity)

###### Ray Casting

1. Generate an image by casting one ray per pixel
2. Check for shadows by sending a ray to the light

###### Pinhole Camera Model

1. eye ray (starts at eye and goes through pixel)
2. image plane (or the near plane in perspective projection)
3. closest scene intersection point
4. perform shading calculation here to compute color of pixel (e.g. Blinn Phong model)

###### Recursive (Whitted-Style) Ray Tracing

1. Reflected ray (specular reflection)
2. Refracted rays (specular transmission)
3. Primary rays
4. Secondary rays
5. Shadow rays

###### Ray Equation

$$
r(t)=o+td,\ t>0
$$

###### Möller Trumbore Algorithm

A faster approach, giving barycentric coordinate directly

$$
\overrightarrow{O}+t\overrightarrow{D}=(1-b_1-b_2)\overrightarrow{P_0}+b_1\overrightarrow{P_1}+b_2\overrightarrow{P_2}
$$

$$
\left(
\begin{matrix}
t\\
b_1\\
b_2
\end{matrix}
\right)=
\frac{1}{\overrightarrow{S_1}\cdot{\overrightarrow{E_1}}}
\left(
\begin{matrix}
\overrightarrow{S_2}\cdot{\overrightarrow{E_2}}\\
\overrightarrow{S_1}\cdot{\overrightarrow{S}}\\
\overrightarrow{S_2}\cdot{\overrightarrow{D}}
\end{matrix}
\right)
$$

$$
\begin{align}
\overrightarrow{E_1}&=\overrightarrow{P_1}-\overrightarrow{P_0}\\
\overrightarrow{E_2}&=\overrightarrow{P_2}-\overrightarrow{P_0}\\
\overrightarrow{S}&=\overrightarrow{O}-\overrightarrow{P_0}\\
\overrightarrow{S_1}&=\overrightarrow{D}-\overrightarrow{E_2}\\
\overrightarrow{S_2}&=\overrightarrow{S}-\overrightarrow{E_1}
\end{align}
$$

# 作业 5

###### Renderer.cpp

```c++
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            float x, world_scene_width;
            float y, world_scene_height;
            world_scene_height = 2 * scale;
            world_scene_width = world_scene_height * imageAspectRatio; 
            x = (i + 0.5) / (scene.width - 1);
            x = x * 2 - 1;
            x = x * (world_scene_width / 2);
            y = (j + 0.5) / (scene.height - 1);
            y = y * 2 - 1; 
            y = y * (world_scene_height / 2);         
            Vector3f dir = Vector3f(x, y, -1);
            dir = normalize(dir);
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
```

###### Triangle.hpp

```c++
bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    Vector3f E1, E2, S, S1, S2, result;
    E1 = v1 - v0;
    E2 = v2 - v0;
    S = orig - v0;
    S1 = crossProduct(dir, E2);
    S2 = crossProduct(S, E1);
    result = 1 / dotProduct(S1, E1) * Vector3f(dotProduct(S2, E2), dotProduct(S1, S), dotProduct(S2, dir));
    tnear = result.x;
    u = result.y;
    v = result.z;
    if(tnear>0 && u>=0 && u<=1 && v>=0 && v<=1)
        return true;
    else
        return false;
}
```

