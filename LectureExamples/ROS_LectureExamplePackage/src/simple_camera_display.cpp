#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageDisplayer : public rclcpp::Node
{
    public:
        ImageDisplayer()
            : Node("image_displayer")
        {
            subscriber = create_subscription<sensor_msgs::msg::Image>(
                            "/camera/camera/color/image_raw", 10,
                            std::bind(&ImageDisplayer::callback, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Node to display camera started");
        }

    private:
        void callback(const sensor_msgs::msg::Image::SharedPtr message)
        {
            // Transform ROS Image to OpenCV image through CvBridge
            cv_bridge::CvImagePtr cvPointer;
            try
            {
                // OpenCV's color channels follow BGR instead of RGB
                cvPointer = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
            }
            catch (const cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(get_logger(), "CvBridge exception: %s", e.what());
                return;
            }

            cv::Mat frame = cvPointer->image;
            cv::imshow("Frame", frame);
            cv::waitKey(1);
        }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageDisplayer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
