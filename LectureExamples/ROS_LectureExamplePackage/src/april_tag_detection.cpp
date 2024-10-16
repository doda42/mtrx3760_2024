#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class AprilTagDetector : public rclcpp::Node
{
    public:
        AprilTagDetector()
            : Node("april_tag_detector")
        {
            subscriber = create_subscription<sensor_msgs::msg::Image>(
                            "/camera/camera/color/image_raw", 10,
                            std::bind(&AprilTagDetector::callback, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Node to subscribe to camera started");
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

            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            std::vector<int> markerIds;
            cv::aruco::detectMarkers(cvPointer->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

            cv::Mat outputImage = cvPointer->image.clone();
            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

            cv::imshow("Detected Markers", outputImage);
            cv::waitKey(1);
        }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
