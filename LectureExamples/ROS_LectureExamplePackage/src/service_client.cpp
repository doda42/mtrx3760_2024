
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsClient : public rclcpp::Node
{
    public:
        AddTwoIntsClient()
            : Node("add_two_ints_client")
        {
            RCLCPP_INFO(get_logger(), "Server to add two ints has started");
            client = create_client<AddTwoInts>("add_two_ints");
            while (!client->wait_for_service(std::chrono::seconds(1)));
            sendRequest(4, 8);
        }
        
    private:
        void handleResponse(rclcpp::Client<AddTwoInts>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Result of adding two ints: %ld", response->sum);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
            }
        }

        void sendRequest(int64_t a, int64_t b)
        {
            auto request = std::make_shared<AddTwoInts::Request>();
            request->a = a;
            request->b = b;
            client->async_send_request(request, std::bind(&AddTwoIntsClient::handleResponse, this, 
                    std::placeholders::_1));
        }
        rclcpp::Client<AddTwoInts>::SharedPtr client;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


