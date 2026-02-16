#Descripción: Nodo publicador que envía un mensaje del tipo String al tópico "informacion" cada 2 seg.

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

#definición del nodo
class Publisher(Node):
   
    def __init__(self):
        
        #constructor del nodo
        super().__init__("publisher")

        #creación del publisher
        self.publisher_ = self.create_publisher(String, "informacion", 10)
        self.get_logger().info("Nodo publisher creado exitosamente!")

        #creación del timer
        self.timer_ = self.create_timer(2,self.publish_info_)

    def publish_info_(self):
        
        #creación del msj y asignación del dato en el campo
        msg = String ()
        msg.data = "hola mundo!"

        #publicación en el tópico
        self.publisher_.publish(msg)

#función principal
def main(args=None):
   
    #inicializa ros 2, crea el nodo y lo mantiene activo
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
