#Descripción: Nodo suscriptor del tópico /información que recibe mensajes del tipo String.

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

#definición del nodo
class Subscriber(Node):

    def __init__(self):

        #constructor del nodo
        super().__init__("subscriber")

        #creación del suscriptor
        self.subscriber_ = self.create_subscription(String, "informacion", self.subscriber_cb, 10)

    #callback que se ejecuta cada vez que llega un msj al tópico
    def subscriber_cb (self, msg: String):
        self.get_logger().info(msg.data)

#función principal
def main(args=None):

    #inicializa ros 2, crea el nodo y lo mantiene activo
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == "__main__":

    main()

