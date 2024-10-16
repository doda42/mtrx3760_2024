#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Note that there is no integration with ROS here
// Compile this with g++ display_image.cpp -o display_image `pkg-config --cflags --libs opencv4`

int main()
{
    std::string path = "../images/superman.jpeg";
    cv::Mat image = cv::imread(path);
    cv::imshow("Image", image);
    cv::waitKey(0);
}