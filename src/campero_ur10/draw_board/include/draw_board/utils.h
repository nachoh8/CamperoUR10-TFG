#pragma once

#include <eigen3/Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

typedef Eigen::Matrix<float, 3, 1> VectorEigen;

void show_img(const cv::Mat& img, const std::string& title = "img") {
    cv::imshow(title, img);
    cv::waitKey(0);
    cv::destroyAllWindows();
}