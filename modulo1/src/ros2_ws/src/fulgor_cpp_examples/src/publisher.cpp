//Descripción: Nodo publicador que envía un mensaje del tipo String al tópico /información cada 2 seg.

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

//define una clase Publisher (crea un nodo) que hereda de rclcpp::Node
class Publisher: public rclcpp::Node
{
public:
    //constructor
    Publisher(): Node("publisher")
    {
        //creación del publisher
        publisher_ = this -> create_publisher<example_interfaces::msg::String>("informacion", 10);
        
        //creación del timer
        timer_ = this ->create_wall_timer(2.0s, std::bind(&Publisher::publishInfo, this));
        RCLCPP_INFO(this->get_logger(), "Nodo creado correctamente!"); 
    }
private:

    //función que ejecuta el timer
    void publishInfo()
    {
        //creación del msj y asignación del dato en el campo
        auto msg = example_interfaces::msg::String();
        msg.data = "Hello world!";

        //publicación en el tópico
        publisher_->publish(msg);
    }

    //Publisher guardado como SharedPtr para que viva mientras el nodo esté activo..
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;

    //Timer guardado como SharedPtr para evitar que se destruya 
    rclcpp::TimerBase::SharedPtr timer_;
};


//función principal
int main (int argc, char **argv)
{
    //inicializa ros 2, crea el nodo y lo mantiene activo
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
