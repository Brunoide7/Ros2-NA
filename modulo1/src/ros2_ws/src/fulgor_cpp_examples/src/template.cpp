#include "rclcpp/rclcpp.hpp"
/*Creamos una clase heredada de la clase Node.*/
class MyNode: public rclcpp::Node
{
    public:
    //Constructor de la clase.
    MyNode(): Node("node_name")
    {
        //Acá va: publicadores, suscriptores, timers, servicios..
    }
    private:
        //Acá va: los métodos y variables privadas del nodo
};

int main (int argc, char **argv)
{
    //Inicializa ros 2:
    rclcpp::init(argc, argv);
    //Crea una instancia del nodo usando un shared ptr:
    auto node = std::make_shared<MyNode>();
    //Mantenemos vivo el nodo:
    rclcpp::spin(node);
    //Para finalizar ros 2:
    rclcpp::shutdown();
    return 0;
}