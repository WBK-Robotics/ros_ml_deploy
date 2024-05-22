import random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

class FloatPublisher(Node):

    def __init__(self):
        super().__init__('float publisher')
        self.publisher = self.create_publisher(Float32, 'float_topic')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = Float32()
        msg.data = random.uniform(0.0, 10.0)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    float_publisher = FloatPublisher()
    rclpy.spin(float_publisher)