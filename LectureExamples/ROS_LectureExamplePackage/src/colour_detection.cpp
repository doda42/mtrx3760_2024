#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ColourDetector : public rclcpp::Node
{
    public:
        ColourDetector()
            : Node("colour_detector")
        {
            subscriber = create_subscription<sensor_msgs::msg::Image>(
                            "/camera/camera/color/image_raw", 10,
                            std::bind(&ColourDetector::callback, this, std::placeholders::_1));
        }

    private:
        void callback(const sensor_msgs::msg::Image::SharedPtr message)
        {
            // Transform ROS Image to OpenCV image through CvBridge
            cv_bridge::CvImagePtr cvPointer;
            try
            {
                cvPointer = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
            }
            catch (const cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(get_logger(), "CvBridge exception: %s", e.what());
                return;
            }

            cv::Mat hsvImage;
            cv::Mat mask;
            cv::cvtColor(cvPointer->image, hsvImage, cv::COLOR_BGR2HSV);

            
            cv::Scalar lower(10, 100, 100);
            cv::Scalar upper(25, 255, 255);

            cv::inRange(hsvImage, lower, upper, mask);
            cv::imshow("Original Image", cvPointer->image); 
            cv::imshow("HSV Image", hsvImage); 
            cv::imshow("Mask", mask); 
            cv::waitKey(1);
        }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColourDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
