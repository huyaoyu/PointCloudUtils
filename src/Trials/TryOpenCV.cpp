//
// Created by yaoyu on 5/8/20.
//

#include <iostream>
#include <opencv2/opencv.hpp>

int main( int argc, char** argv ) {
    std::cout << "Hello, TryOpenCV! " << std::endl;

    cv::Mat img( 2, 2, CV_8UC3 );
    std::cout << "img: " << img << "\n";
    std::cout << "img.step = " << img.step << "\n";

    cv::Mat img1;
    cv::add( img, cv::Scalar::all(255), img1 );
    std::cout << "img1: " << img1 << "\n";

    return 0;
}