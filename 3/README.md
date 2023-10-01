# 视口变换

###### Canonical Cube to Screen

$$
[-1,1]^2\rightarrow{[0,width]\times[0,height]}
$$

###### Viewport transform Matrix

$$
M_{viewport}=
\left(
\begin{matrix}
\frac{width}{2}&0&0&\frac{width}{2}\\
0&\frac{height}{2}&0&\frac{height}{2}\\
0&0&1&0\\
0&0&0&1
\end{matrix}
\right)
$$

# 光栅化

###### 2D Sampling

```c++
for(int x = 0; x < xmax; ++x)
    for(int y = 0; y < ymax; ++y)
        image[x][y] = inside(tri, 
                             x+0.5,
                             y+0.5);
```

###### Point-in-triangle test

Three Cross Products

# 深度缓存

###### Z-Buffer Algorithm

```c++
for(each triangle T)
    for(each sample(x,y,z) in T)
        if(z < zbuffer[x,y])        // closest sample so far
            framebuffer[x,y] = rgb; // update color
            zbuffer[x,y] = z;       // update depth
        else
            ; // do nothing, this sample is occluded
```

# 作业 2

###### main.cpp

```c++
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
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
```

###### rasterizer.cpp

```c++
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
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
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                int idx = get_index(x,y);
                if(z_interpolated < depth_buf[idx])
                {
                    depth_buf[idx] = z_interpolated;
                    set_pixel(Eigen::Vector3f(x,y,0), t.getColor());
                }
            }
        }
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    auto v0 = _v[0];
    auto v1 = _v[1];
    auto v2 = _v[2];
    float x1 = v0[0];
    float y1 = v0[1];
    float x2 = v1[0];
    float y2 = v1[1];
    float x3 = v2[0];
    float y3 = v2[1];
    Eigen::Vector2f vec12(x2 - x1, y2 - y1);
    Eigen::Vector2f vec23(x3 - x2, y3 - y2);
    Eigen::Vector2f vec31(x1 - x3, y1 - y3);
    Eigen::Vector2f one2Point(x - x1, y - y1);
    Eigen::Vector2f two2Point(x - x2, y - y2);
    Eigen::Vector2f three2Point(x - x3, y - y3);
    float a = one2Point.x() * vec31.y() - one2Point.y() * vec31.x();
    float b = two2Point.x() * vec12.y() - two2Point.y() * vec12.x();
    float c = three2Point.x() * vec23.y() - three2Point.y() * vec23.x();
    if(a>0 && b>0 && c>0)
        return true;
    else if(a<0 && b<0 && c<0)
        return true;
    else
        return false;
}
```

