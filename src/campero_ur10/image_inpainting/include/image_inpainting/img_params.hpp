#pragma once

#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h> 

#define DEAULT_MAX_DIST_ERROR 7

class ImgProcessParams {
    std::string getErodeDilateStrType(int type) {
        std::string str_type;
        switch(type) {
            case 1:
                str_type = "MORPH_RECT";
                break;
            case 2:
                str_type = "MORPH_CROSS";
                break;
            case 3:
                str_type = "MORPH_ELLIPSE";
                break;
            default:
                str_type = "NOT_APPLY";
                break;
        }

        return str_type;
    }

    int getErodeDilateType(int type) {
        switch(type) {
            case 1:
                return cv::MORPH_RECT;
            case 2:
                return cv::MORPH_CROSS;
            case 3:
                return cv::MORPH_ELLIPSE;
            default:
                return -1;
        }
    }

    std::string getContourMethodName(int type) {
        switch(type) {
            case 1:
                return "WATERSHED";
            case 2:
                return "WATERSHED_DISTANCE_TRANSFORM";
            default:
                return "NOT_WATERSHED";
        }
    }

public:
    int max_dist_error;

    bool valid_matrix_pnp = false;
    cv::Mat cameraMatrix, // 3x3
            distortionEffect, // 4x1
            rvec, tvec; // 1x3
    cv::Size camSize;

    int canny_threshold1, canny_threshold2;
    int blur_ksize;

    int contour_method; // 0-1
    int min_contour_size;
    int binary_thresh; // 0-255
    int conectivity_way; // 4 or 8

    // concaveman:
    double concaveman_alpha;
    bool apply_concaveman;
    
    // smoothPath
    int smooth_path_kernel;
    bool apply_smooth_path;
    
    // method: not_watershed
    bool apply_sharp;

    // method: watershed
    int erode_type, erode_size, dilate_type, dilate_size, number_iterations;

    ImgProcessParams() {}

    ImgProcessParams(int _max_dist_error, int _canny_threshold1, int _canny_threshold2, int _blur_ksize,
                    int _erode_type, int _erode_size, int _dilate_type, int _dilate_size)
    {
        max_dist_error = _max_dist_error;
        canny_threshold1 = _canny_threshold1;
        canny_threshold2 = _canny_threshold2;
        blur_ksize = _blur_ksize;
        erode_type = _erode_type;
        erode_size = _erode_size;
        dilate_type = _dilate_type;
        dilate_size = _dilate_size;
    }

    void print_config() {
        std::string msg = "--Image Processing Parameters--\n";
        msg += "Max Distance Error: " + std::to_string(max_dist_error) + " px\n";
        msg += "Contour method: " + getContourMethodName(contour_method) + "\n";
        msg += "Apply Concaveman: " + std::to_string(apply_concaveman) + "\n";
        if (apply_concaveman) msg += "\tConcaveman alpha: " + std::to_string(concaveman_alpha) + "\n";
        msg += "Apply smoothPath: " + std::to_string(apply_smooth_path) + "\n";
        if (apply_smooth_path) {
			msg += "\tKernel size: " + std::to_string(smooth_path_kernel) + "\n";
		}
        msg += "Min contour size: " + std::to_string(min_contour_size) + " px\n";
        msg += "Binary Threshold: " + std::to_string(binary_thresh) + "\n";
        msg += "Conectivity way: " + std::to_string(conectivity_way) + "\n";
        msg += "Method WATERSHED params:\n";
        msg += "\tOpening operation num iterations: " + std::to_string(number_iterations) + "\n";
        msg += "\tDilate type: " + getErodeDilateStrType(dilate_type) + "\n";
        msg += "\tDilate size: " + std::to_string(dilate_size) + "\n";
        msg += "\tErode type: " + getErodeDilateStrType(erode_type) + "\n";
        msg += "\tErode size: " + std::to_string(erode_size) + "\n";
        msg += "Method NOT_WATERSHED params:\n";
        msg += "\tApply Sharp: " + std::to_string(apply_sharp) + "\n";
        msg += "\tBlur Kernel Size: " + std::to_string(blur_ksize) + "\n";

        ROS_INFO(" %s ", msg.c_str());
        /*
        ROS_INFO("--Image Processing Parameters--");
        ROS_INFO("Max Distance Error: %d px", max_dist_error);
        ROS_INFO("Canny Threshold 1: %d ", canny_threshold1);
        ROS_INFO("Canny Threshold 2: %d ", canny_threshold2);
        ROS_INFO("Blur Kernel Size: %dx%d ", blur_ksize, blur_ksize);
        ROS_INFO("Erode type: %s ", getErodeDilateStrType(erode_type).c_str());
        ROS_INFO("Erode Size: %d ", erode_size);
        ROS_INFO("Dilate type: %s ", getErodeDilateStrType(dilate_type).c_str());
        ROS_INFO("Dilate Size: %d ", dilate_size);*/
    }

    inline int getErodeType() {
        return getErodeDilateType(erode_type);
    }

    inline int getDilateType() {
        return getErodeDilateType(dilate_type);
    }

    /*inline void setConcavemanAlpha(const int _alpha) {
        alpha = double(_alpha) / double(100);
    }

    inline void getConcavemanAlpha() {
        return alpha;
    }*/
};
