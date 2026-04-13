#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Int64

class NumberPublisher(Node):

    def __init__(self):
        super().__init__("number_publisher")
        
        #1ro declaramos el parámetro, y asignamos valor por defecto
        self.declare_parameter("number",2)
        self.declare_parameter("timer_period",1.0)

        #2do obtenemos el valor "actual" del parámetro
        self.number_= self.get_parameter("number").value
        self.timer_period = self.get_parameter("timer_period").value

        #para modificar parámetros una vez iniciado el nodo.
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.publisher_ = self.create_publisher(Int64,"number",10)
        self.get_logger().info("Publicador creado!")
        self.timer_ = self.create_timer(self.timer_period,self.publish_number)

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
    
        self.publisher_.publish(msg)

    #cb para modificar los valores de los parámetros en tiempo de ejecución
    def parameters_callback(self, params: list[Parameter]):
        for param in params: 
            if param.name == "number":
                self.number_ = param.value

def main (args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
