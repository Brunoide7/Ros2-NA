#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoInitServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        #el servidor de servicios se crea en el constructor, donde:
        #el 1er arg es el tipo de msj, el 2do el nombre del servidor que querramos darle,
        #el 3er arg es el cb. El nombre del 2do arg es recomendable que empiece con un verb/acción.
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Servidor: Add two ints creado!")

    #callback_nombre_servicio():
    def callback_add_two_ints(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        #hay que hacer el return si o si, por más que ya completamos el response
        return response
    
def main (args=None):
    rclpy.init(args=args)
    node = AddTwoInitServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()