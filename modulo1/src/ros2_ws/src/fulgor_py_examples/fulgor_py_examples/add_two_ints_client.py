#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        #creamos el cliente:
        self.client_ = self.create_client(AddTwoInts,"add_two_ints")

    #creamos un método para llamar al servicio para enviar la solicitud:
    def call_add_two_ints(self, a, b):
        #verificamos que el servicio está en ejecución:
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Esperando Add Two Ints server...")

        #creamos una solicitud:
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        #enviamos la solicitud:
        future = self.client_.call_async(request)
        #como ya tenemos un spin en el main, no hay que hacer otro.
        #generamos un cb para cuando llegue la respuesta:
        #partial, nos permite agregar más args, sino no podríamos agregar el arg request. Esto es-
        #-para los obj Future
        future.add_done_callback(partial(self.callback_call_add_two_ints,request=request))

    def callback_call_add_two_ints (self, future, request):
        response = future.result()
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")


def main (args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    #las sig líneas llaman directamente al método para enviar solicitudes:
    node.call_add_two_ints(20,5)
    node.call_add_two_ints(2,23)
    node.call_add_two_ints(3,1)
    #mantenemos en ejecución:
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()