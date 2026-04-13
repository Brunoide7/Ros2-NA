#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
//permite usar literales de tiempo, ej: 1s
using namespace std::chrono_literals;
using namespace std::placeholders;
class AddTwoIntsClientNode: public rclcpp::Node
{
    public:
    AddTwoIntsClientNode(): Node("add_two_ints_client")
    {
        //creación del cliente para el servicio "add_two_ints"
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }
    //método para llamar al servicio
    void callAddTwoInts(int a, int b)
    {
        while(!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "esperando el servidor..");
        }
        //creación de la request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        //envío asincrono de la request. El cb maneja la respuesta.
        client_->async_send_request(request, std::bind(&AddTwoIntsClientNode::callbackAddTwoInts, this, _1));
    }
    private:
    //cb se ejecuta cuando llega la rta del servicio
    void callbackAddTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        //obtiene la rta del servidor
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "suma: %d", (int)response->sum);
    }
    //ptr al cliente del servicio
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->callAddTwoInts(10,5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}