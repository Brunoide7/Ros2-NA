#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        //declaración de los parámetros
        this->declare_parameter("number", 2);
        this->declare_parameter("timer_period", 1.0);
        //lectura de parámetros
        number_ = this->get_parameter("number").as_int();
        double timer_period = this->get_parameter("timer_period").as_double();
        //cb se ejecuta cuando un parámetro cambia en tiempo de ejecución
        param_callback_handle_ = this->add_post_set_parameters_callback(
            std::bind(&NumberPublisherNode::parametersCallback, this, _1));

        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        number_timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period),
                                                std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number publisher iniciado!");
    }
private:
    void publishNumber()    //función que publica el número periódicamente
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        number_publisher_->publish(msg);
    }
    //cb cuando cambian parámetros
    void parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        for (const auto &param: parameters) {
            if (param.get_name() == "number") {
                number_ = param.as_int();      }
        }
    }
    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
    //handle del cb de parámetros. Es necesario para mantenerlo activo.
    PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}