#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
constexpr double MY_PI = 3.1415926;

// 把相机移到标准系下
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity(); // 用单位矩阵对view进行初始化

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                0, 1, 0, -eye_pos[1],
                0, 0, 1, -eye_pos[2], 
                0, 0, 0, 1;

    view = translate * view;

    return view;
}

//在此函数中，你只需要实现三维中绕 z 轴旋转的变换矩阵，
//而不用处理平移与缩放。
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float ra = rotation_angle / 180 * MY_PI; //转为弧度制，因为Eigen默认用弧度制
    float cosa = cos(ra);
    float sina = sin(ra);
    model <<
        cosa, -sina, 0, 0,
        sina, cosa, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float n, float f)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float t = tan((eye_fov / 360) * MY_PI) * (abs(n)); //top
    float r = t * aspect_ratio;

    Eigen::Matrix4f Mp;//透视矩阵
    Mp <<
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    Eigen::Matrix4f Mo_tran;//平移矩阵
    Mo_tran <<
        1, 0, 0, 0,
        0, 1, 0, 0,  //b=-t;
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Eigen::Matrix4f Mo_scale;//缩放矩阵
    Mo_scale <<
        1 / r, 0, 0, 0,
        0, 1 / t, 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    projection = (Mo_scale * Mo_tran) * Mp;//投影矩阵
    //正交等于中心移到{0，0，0}原心，再压缩到[-1，1]的立方体
    //这里一定要注意顺序，先透视再正交;正交里面先平移再缩放；否则做出来会是一条直线！
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700); // 定义窗口大小

    Eigen::Vector3f eye_pos = { 0, 0, 5 }; //定义相机初始的位置

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} }; //三角形的三个顶点

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} }; // 上面三个点的遍历顺序

    // 加载顶点与遍历顺序信息
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;
    
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth); // 作初始化

        // MVP三大变换
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}

