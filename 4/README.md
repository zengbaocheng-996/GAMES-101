# Blinn-Phong 反射模型

$$
\begin{align}
L&=L_a+L_d+L_s\\
&=k_aI_a+k_d(I/r^2)max(0,n\cdot{l})+k_s(I/r^2)max(0,n\cdot{h})^p
\end{align}
$$

###### Shading Point

1. View direction, v
2. Surface normal, n
3. Light direction, l (for each of many lights)
4. Surface parameters (color, shininess, ...)

###### Diffuse

$$
L_d=k_d(I/r^2)max(0,n\cdot{l})
$$

Lambert's cosine law

$$
cos\theta=l\cdot{n}
$$
Light Falloff
$$
I/r^2
$$

###### Specular

$$
\begin{align}
L_s&=k_s(I/r^2)max(0,cos\alpha)^p\\
&=k_s(I/r^2)max(0,n\cdot{h})^p
\end{align}
$$

half vector

$$
h=bisector(v,l)=\frac{v+l}{||v+l||}
$$

###### Ambient

$$
L_a=k_aI_a
$$

# 着色频率

###### Flat Shading

Triangle face is flat one normal vector

###### Gouraud Shading

Interpolate colors from vertices across triangle

Each vertex has a normal vector 

###### Phong Shading

Interpolate normal vectors across each triangle

Compute full shading model at each pixel

###### Per-Vertex Normal Vectors

$$
N_v=\frac{\sum_iN_i}{||\sum_iN_i||}
$$

###### Per-Pixel Normal Vectors

Barycentric interpolation

# 图形管线

1. Vertex Processing (Model, View, Projection transforms, Shading)
2. Triangle Processing
3. Rasterization (Sampling triangle coverage)
4. Fragment (one per covered sample like pixel) Processing (Z-Buffer Visibility Tests, Shading)
5. Framebuffer Operations

# 重心坐标

$$
\begin{align}
(x,y)&=\alpha{A}+\beta{B}+\gamma{C}\\
\alpha+\beta+\gamma&=1
\end{align}
$$

$$
\begin{align}
\alpha&=\frac{-(x-x_B)(y_C-y_B)+(y-y_B)(x_C-x_B)}{-(x_A-x_B)(y_C-y_B)+(y_A-y_B)(x_C-x_B)}\\
\beta&=\frac{-(x-x_C)(y_A-y_C)+(y-y_C)(x_A-x_C)}{-(x_B-x_C)(y_A-y_C)+(y_B-y_C)(x_A-x_C)}\\
\gamma&=1-\alpha-\gamma
\end{align}
$$

# 纹理映射

###### Texture Mapping (kd)

Each triangle vertex is assigned a texture coordinate (u, v)

Interpolate across triangles by barycentric coordinates

```c++
for each rasterized screen sample (x,y): // Usually a pixel's center
	(u,v) = evaluate texture coordinate at (x,y); // Using barycentric coordinates
    texcolor = texture.sample(u,v);
	set sample's color to texcolor; // Usually the diffuse albedo Kd (Blinn-Phong reflectance model)
```

###### Bump Mapping 凹凸贴图

1D

$$
\begin{align}
dp&=c[h(p+1)-h(p)]\\
n(p)&=(-dp,1).normalized()
\end{align}
$$

3D

$$
\begin{align}
dp/du&=c1[h(u+1)-h(u)]\\
dp/dv&=c2[h(v+1)-h(v)]\\
n&=(-dp/du,-dp/dv,1).normalized()
\end{align}
$$

###### Displacement Mapping 位移贴图

Actually moves the vertices

# 作业 3

###### rasterizer.cpp

```c++
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    auto v = t.toVector4();
    auto v0 = v[0];
    auto v1 = v[1];
    auto v2 = v[2];
    float x1 = v0[0];
    float y1 = v0[1];
    float x2 = v1[0];
    float y2 = v1[1];
    float x3 = v2[0];
    float y3 = v2[1];
    float minX = std::min(x1,std::min(x2,x3));
    float maxX = std::max(x1,std::max(x2,x3));
    float minY = std::min(y1,std::min(y2,y3));
    float maxY = std::max(y1,std::max(y2,y3));
    for(int x = minX; x < maxX; x++)
        for(int y = minY; y < maxY; y++)
        {
            if(insideTriangle(x, y, t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                zp *= Z;

                int idx = get_index(x,y);
                if(zp < depth_buf[idx])
                {
                    depth_buf[idx] = zp;
                    auto color = t.color;
                    auto normal = t.normal;
                    auto tex_coords = t.tex_coords;
                    auto interpolated_color = interpolate(alpha, beta, gamma, color[0], color[1], color[2], 1);
                    auto interpolated_normal = interpolate(alpha, beta, gamma, normal[0], normal[1], normal[2], 1);
                    auto interpolated_texcoords = interpolate(alpha, beta, gamma, tex_coords[0], tex_coords[1], tex_coords[2], 1);
                    auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);

                    fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    payload.view_pos = interpolated_shadingcoords;
                    auto pixel_color = fragment_shader(payload);
                    set_pixel(Eigen::Vector2i(x,y), pixel_color);
                }
            }
        }
}
```

###### main.cpp

```c++
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // std::cout<<eye_fov/2<<std::endl;
    float top = zNear * std::tan(eye_fov/2/180.0 * std::acos(-1));
    float right = aspect_ratio * top;
    Eigen::Matrix4f T_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f S_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Persp2Ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    T_ortho << 1/right,     0,              0, 0,
                     0, 1/top,              0, 0,
                     0,     0, 2/(zNear-zFar), 0,
                     0,     0,              0, 1;

    S_ortho << 1, 0, 0,               0,
               0, 1, 0,               0,
               0, 0, 1, -(zNear+zFar)/2,
               0, 0, 0,               1;

    Persp2Ortho << zNear,     0,          0,           0,
                       0, zNear,          0,           0,
                       0,     0, zNear+zFar, -zNear*zFar,
                       0,     0,          1,           0;
	
    projection = S_ortho * T_ortho * Persp2Ortho;
    return projection;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    Eigen::Vector3f ambient;
    Eigen::Vector3f diffuse;
    Eigen::Vector3f specular;

    Eigen::Vector3f view_dir = (eye_pos - point).normalized();
    Eigen::Vector3f light_dir;
    Eigen::Vector3f point2light;
    Eigen::Vector3f bisector;
    for (auto& light : lights)
    {
        point2light = light.position - point;
        light_dir = point2light.normalized();
        float r2 = point2light.dot(point2light);
        ambient = ka.cwiseProduct(amb_light_intensity); 
        diffuse = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.dot(light_dir));
        bisector = (view_dir + light_dir).normalized();
        specular = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.0f, normal.dot(bisector)), p);
        result_color += (ambient + diffuse + specular);
        // std::cout<<result_color<<std::endl;
    }
    return result_color * 255.f;
}

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    
    Eigen::Vector3f ambient;
    Eigen::Vector3f diffuse;
    Eigen::Vector3f specular;

    Eigen::Vector3f view_dir = (eye_pos - point).normalized();
    Eigen::Vector3f light_dir;
    Eigen::Vector3f point2light;
    Eigen::Vector3f bisector;
    for (auto& light : lights)
    {
        point2light = light.position - point;
        light_dir = point2light.normalized();
        float r2 = point2light.dot(point2light);
        ambient = ka.cwiseProduct(amb_light_intensity); 
        diffuse = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.dot(light_dir));
        bisector = (view_dir + light_dir).normalized();
        specular = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.0f, normal.dot(bisector)), p);
        result_color += (ambient + diffuse + specular);
        // std::cout<<result_color<<std::endl;
    }
    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t(x*y/std::sqrt(x*x+z*z),std::sqrt(x*x+z*z),z*y/std::sqrt(x*x+z*z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t.x(), t.y(), t.z(),
           b.x(), b.y(), b.z(),
               x,     y,     z;
    
    auto tex_coords =  payload.tex_coords;
    auto texture = payload.texture;
    float u = tex_coords.x();
    float v = tex_coords.y();
    float w = texture->width;
    float h = texture->height;

    auto colorUV = texture->getColor(u, v).norm();
    float dU = kh * kn * (texture->getColor(std::min(1.0f, u+1/w), v).norm() - colorUV);
    float dV = kh * kn * (texture->getColor(u, std::min(1.0f, v+1/h)).norm() - colorUV);
    Eigen::Vector3f ln(-dU, -dV, 1);
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t(x*y/std::sqrt(x*x+z*z),std::sqrt(x*x+z*z),z*y/std::sqrt(x*x+z*z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t.x(), t.y(), t.z(),
           b.x(), b.y(), b.z(),
               x,     y,     z;
    
    auto tex_coords =  payload.tex_coords;
    auto texture = payload.texture;
    float u = tex_coords.x();
    float v = tex_coords.y();
    float w = texture->width;
    float h = texture->height;

    auto colorUV = texture->getColor(u, v).norm();
    float dU = kh * kn * (texture->getColor(std::min(1.0f, u+1/w), v).norm() - colorUV);
    float dV = kh * kn * (texture->getColor(u, std::min(1.0f, v+1/h)).norm() - colorUV);
    Eigen::Vector3f ln(-dU, -dV, 1);
    point += kn * normal * colorUV;
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    Eigen::Vector3f ambient;
    Eigen::Vector3f diffuse;
    Eigen::Vector3f specular;

    Eigen::Vector3f view_dir = (eye_pos - point).normalized();
    Eigen::Vector3f light_dir;
    Eigen::Vector3f point2light;
    Eigen::Vector3f bisector;
    for (auto& light : lights)
    {
        point2light = light.position - point;
        light_dir = point2light.normalized();
        float r2 = point2light.dot(point2light);
        ambient = ka.cwiseProduct(amb_light_intensity); 
        diffuse = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.dot(light_dir));
        bisector = (view_dir + light_dir).normalized();
        specular = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.0f, normal.dot(bisector)), p);
        result_color += (ambient + diffuse + specular);
        // std::cout<<result_color<<std::endl;
    }
    return result_color * 255.f;
}
```

