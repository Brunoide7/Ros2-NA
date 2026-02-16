//Descripción: Nodo suscriptor del tópico /información que recibe mensajes del tipo String.

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders; 

//define una clase Subscriber (crea un nodo) que hereda de rclcpp::Node
class Subscriber: public rclcpp::Node 
{
public:
    //constructor
    Subscriber(): Node ("subscriber")
    {
        //creación del subscriber
        subscriber_ = this ->create_subscription<example_interfaces::msg::String>("informacion", 10, 
        std::bind(&Subscriber::cbSubscriberInformacion, this,_1));

        RCLCPP_INFO(this->get_logger(), "Nodo creado correctamente!");

    }
private: 

    //callback que se ejecuta cuando llega un nuevo mensaje al tópico
    void cbSubscriberInformacion(const example_interfaces::msg::String::SharedPtr msg)
    {
        //imprime el mensaje del tópico en pantalla
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    //suscripción guardado como SharedPtr para que no se destruya al salir del constructor
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

//función principal
int main (int argc, char **argv)
{
    //inicializa ros 2, crea el nodo y lo mantiene activo
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
