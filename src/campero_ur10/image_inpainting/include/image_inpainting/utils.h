#pragma once

#include <vector>

#include <eigen3/Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

typedef Eigen::Matrix<float, 3, 1> VectorEigen;

void show_img(const cv::Mat& img, const std::string& title = "img", const bool wait = true) {
    cv::imshow(title, img);
    if (wait) {
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}

void show_img_from_vec(const std::vector<cv::Point>& vec, const cv::Size img_size, const std::string& title = "img", const bool wait = true) {
    cv::Mat img(img_size, CV_8UC1); 
    cv::polylines(img, vec, false, cv::Scalar(255)); // or perhaps 0
    cv::imshow(title, img);
    if (wait) {
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}