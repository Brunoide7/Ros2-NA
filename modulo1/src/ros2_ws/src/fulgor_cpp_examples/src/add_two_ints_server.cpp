#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
// Permite usar _1, _2 en std::bind sin escribir std::placeholders::_1, etc.
using namespace std::placeholders;
class AddTwoIntsServerNode: public rclcpp::Node
{
    public:
    //Constructor de la clase.
    AddTwoIntsServerNode(): Node("add_two_ints_server")
    {
        // Creación del servidor de servicio
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
        std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this,_1, _2));
        RCLCPP_INFO(this->get_logger(),"Add two ints service creado!");
    }
    private:
        // Función callback que se ejecuta cuando un cliente llama al servicio
        void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d",
                                            (int)request->a, (int) request->b, (int)response->sum);
        }
        // Puntero al servidor del servicio
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};
int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}