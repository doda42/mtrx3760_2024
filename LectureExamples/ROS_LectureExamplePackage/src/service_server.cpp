#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsServer : public rclcpp::Node
{
    public:
        AddTwoIntsServer()
            : Node("add_two_ints_server")
        {
            server = create_service<AddTwoInts>("add_two_ints", 
                        std::bind(&AddTwoIntsServer::callback, this, std::placeholders::_1,
                        std::placeholders::_2));
            RCLCPP_INFO(get_logger(), "Server to add two ints has started");
        }

    private:
        void callback(const AddTwoInts::Request::SharedPtr request,
                      const AddTwoInts::Response::SharedPtr response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
        }

        rclcpp::Service<AddTwoInts>::SharedPtr server;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}