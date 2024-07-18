#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // 对于高阶的贝塞尔可不断通过递归到一阶求出
    if (control_points.size()==2)
    {
        auto point = control_points[0] * (1 - t) + control_points[1] * t;
        return point;
    }
    std::vector<cv::Point2f> points;
    for (int i = 0; i < control_points.size()-1; i++)
        points.push_back(control_points[i] * (1 - t) + control_points[i + 1] * t);
    
    return recursive_bezier(points, t);
    // return cv::Point2f();
}

// 这里的反走样和三角形光栅化的反走样不一样，这里主要是看曲线上各点到周围四个邻近像素中心点的距离来做
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
        // 2x2 反走样
    // 
    // without Anti-aliasing
    //for (double i = 0; i <= 1; i += 0.001)
    //{
    //    auto point = recursive_bezier(control_points, i);
    //    window.at<cv::Vec3b>(point.y, point.x)[1] = 255; // 改颜色
    //}

    #pragma region distance
    // 首先上一个做法有错误的地方，就是如果只是按简单的比例(点和像素中心距离/总距离)，
    // 越远的点，着色其实是越重的，所以会产生线比较粗，而且还是有走样现象。
    // with Anti-aliasing
    //for (double t = 0; t < 1.; t += 0.001)
    //{
    //    auto point = recursive_bezier(control_points, t);
    //    
    //    // 先算出此时距离point最近的像素点
    //    // 再找出相邻四个点
    //    cv::Point2i p(point.x - std::floor(point.x) < 0.5 ? std::floor(point.x) : std::ceil(point.x),
    //                point.y - std::floor(point.y) < 0.5 ? std::floor(point.y) : std::ceil(point.y));

    //    std::vector<cv::Point2i> pv = { p, cv::Point2i(p.x - 1,p.y), cv::Point2i(p.x,p.y - 1), cv::Point2i(p.x - 1,p.y - 1) };

    //    // point到相邻四个点中心点的距离
    //    float sum_distance = 0.f;
    //    float max_distance = std::sqrt(2.);
    //    std::vector<float> ds = {};
    //    for (int i = 0; i < 4; i++)
    //    {
    //        cv::Point2f center_point(pv[i].x + 0.5, pv[i].y + 0.5);
    //        float d = max_distance - std::sqrt(std::pow(point.x - center_point.x, 2) + std::pow(point.y - center_point.y, 2));
    //        ds.push_back(d);
    //        sum_distance += d;
    //    }
    //    // assign colors
    //    for (int i = 0; i < 4; i++)
    //    {
    //        float k = ds[i] / sum_distance;
    //        window.at<cv::Vec3b>(point.y, point.x)[1] = std::min(255.f, window.at<cv::Vec3b>(pv[i].y, pv[i].x)[1] + 255.f); // 改颜色
    //    }
    //}
    #pragma endregion
    // 其实简单用比例并不能解决该问题，反而线变得更粗

    #pragma region bilinear interpolation
    //cv::Mat window_copy = window.clone();
    //float min_x = window.cols;
    //float max_x = 0;
    //float min_y = window.rows;
    //float max_y = 0;

    //// find out the bounding box of current line
    //for (int i = 0; i < control_points.size(); i++)
    //{
    //    min_x = std::min(control_points[i].x, min_x);
    //    max_x = std::max(control_points[i].x, max_x);
    //    min_y = std::min(control_points[i].y, min_y);
    //    max_y = std::max(control_points[i].y, max_y);
    //}

    //// 然后遍历bounding box
    //for (int y = min_y; y < max_y; y++)
    //{
    //    for (int x = min_x; x < max_x; x++)
    //    {
    //        for (float j = 0.25; j < 1; j += 0.5)
    //        {
    //            for (float i = 0.25; i < 1; i += 0.5)
    //            {
    //                // find out the center point/coordinates
    //                int cx = i > 0.5 ? x + 1 : x;
    //                int cy = j > 0.5 ? y + 1 : y;
    //                if (cx > max_x || cy > max_y) continue;

    //                cv::Vec3b u00 = window_copy.at<cv::Vec3b>(cy - 0.5, cx - 0.5);
    //                cv::Vec3b u10 = window_copy.at<cv::Vec3b>(cy - 0.5, cx + 0.5);
    //                cv::Vec3b u01 = window_copy.at<cv::Vec3b>(cy + 0.5, cx - 0.5);
    //                cv::Vec3b u11 = window_copy.at<cv::Vec3b>(cy + 0.5, cx + 0.5);

    //                float s = (x + i) - (cx - 0.5);
    //                float t = (y + j) - (cy - 0.5);

    //                cv::Vec3b u0 = (1 - s) * u00 + s * u10;
    //                cv::Vec3b u1 = (1 - s) * u01 + s * u11;
    //                cv::Vec3b res = (1 - t) * u0 + t * u1;

    //                window.at<cv::Vec3b>(y, x)[0] = res[0];
    //                window.at<cv::Vec3b>(y, x)[1] = res[1];
    //                window.at<cv::Vec3b>(y, x)[2] = res[2];
    //            }
    //        }
    //    }
    //}
    #pragma endregion

    #pragma region 调整着色比例
    // method 3
    // 调整方法一中的着色比例，改成：(最大像素点距离-点和当前处理的像素距离)/(A1+A2+A3+A4)，
    // ps：最大像素点距离 - 点和当前处理的像素距离 = Ai(i = 1, 2, 3, 4)
    // 当然像素的颜色要在处理像素颜色通道基础上加上比例颜色，才可以和背景颜色分开
    for (float t = 0.; t < 1.; t+= 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        // 先找point周围的四个像素
        cv::Point2f p0(point.x - std::floor(point.x) < 0.5 ? std::floor(point.x) : std::ceil(point.x),
                        point.y - std::floor(point.y) < 0.5 ? std::floor(point.y) : std::ceil(point.y));
        cv::Point2f p1(p0.x - 1, p0.y);
        cv::Point2f p2(p0.x - 1, p0.y - 1);
        cv::Point2f p3(p0.x, p0.y - 1);
        std::vector<cv::Point2f> points{ p0,p1,p2,p3 };

        float max_distance = std::sqrt(2.);
        float point_distance = 0.0f; // 0.x默认是double类型
        float sum_distance = 0.0f; // 用来记录点距离四个像素中心的总长度
        std::vector<float> distance_list{}; // 储存四个距离\

        for (int i = 0; i < 4; i++)
        {
            cv::Point2f center_coordinate(points[i].x + 0.5, points[i].y + 0.5);
            point_distance = max_distance - std::sqrt(std::pow(point.x - center_coordinate.x, 2) + std::pow(point.y - center_coordinate.y, 2));
            // 求出:像素中心最远距离-点与当前像素中心坐标的距离
            // 为什么这样做：因为越远的点，插值颜色越小，所以考虑线性插值按比例就这样做
            distance_list.push_back(point_distance);
            sum_distance += point_distance;
        }
        
        for (int i = 0; i < 4; i++)
        {
            float k = distance_list[i] / sum_distance; // 求出距离占比系数k，离的远的相对应占比小一点，作为每个像素的颜色的比例
            window.at <cv::Vec3b>(points[i].y, points[i].x)[1] = std::min(255.f, window.at <cv::Vec3b>(points[i].y, points[i].x)[1] + 255.f * k);
            // 这里是对每个像素本身进行操作，自身一开始的颜色通道就是window.at <cv::Vec3b>(point_coordinate[i].y, point_coordinate[i].x)[1]，
            // 在此基础上要加上比例颜色，为了不超过255.f，要进行最小值比较
        }
    }
    #pragma endregion
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
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("curve_with_bezier.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
