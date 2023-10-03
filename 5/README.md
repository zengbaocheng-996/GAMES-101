# 贝塞尔曲线

###### de Casteljau Algorithm

1. Consider three points (quadratic Bezier)
2. Insert a point using linear interpolation
3. Repeat recursively

# 作业 4

###### main.cpp

```c++
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    int size = control_points.size();
    if(size == 2)
        return control_points[0] + t * (control_points[1] - control_points[0]);

    std::vector<cv::Point2f> control_points_temp;
    for(int i=0; i<size-1; i++)
        control_points_temp.push_back(control_points[i] + t * (control_points[i+1] - control_points[i]));

    return recursive_bezier(control_points_temp, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    for(double t=0; t<=1; t+=0.001f)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}

```

