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
           default:
                return "SIMPLE (NOT_WATERSHED)";
        }
    }

public:
    // Calculate transform params
    cv::Mat cameraMatrix, // 3x3
            distortionEffect, // 4x1
            rvec, tvec; // 1x3
    cv::Size camSize;

    bool valid_matrix_pnp = false;
    int max_dist_error;

    // concaveman params:
    double concaveman_alpha;
    bool apply_concaveman;
    
    // smoothPath params:
    int smooth_path_kernel;
    bool apply_smooth_path;
    
    // common proc params:
    int contour_method; // 0-1
    int blur_ksize;
    int conectivity_way; // 4 or 8
    int min_contour_size;

    // not_watershed proc method params:
    bool apply_sharp;

    // watershed proc method params:
    int erode_type, erode_size, dilate_type, dilate_size, number_iterations;

    ImgProcessParams() {}

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
    }

    inline int getErodeType() {
        return getErodeDilateType(erode_type);
    }

    inline int getDilateType() {
        return getErodeDilateType(dilate_type);
    }
};
