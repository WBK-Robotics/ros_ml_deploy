import random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class IntPublisher(Node):

    def __init__(self):
        super().__init__('int publisher')
        self.publisher = self.create_publisher(Int32, 'int_topic')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = Int32()
        msg.data = random.randint(0, 10)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    int_publisher = IntPublisher()
    rclpy.spin(int_publisher)