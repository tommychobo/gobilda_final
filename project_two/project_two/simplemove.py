import numpy as np
import rclpy
from rclpy.node import node

from geometry_msgs.msg import TwistStamped

class SimpleMove(Node):
    def __init(self):
        super().__init__('simple_move')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = 1.0
        self.turn_right_msg = TwistStamped()
        self.turn_right_msg.twist.angular.z = -np.pi/2
        self.turn_left_msg = TwistStamped()
        self.turn_left_msg.twist.angular.z = np.pi/2

        self.instructions = [('f', 5), ('r', 1), ('f', 5), ('l', 1), ('f', 4)]
        self.i_ptr = 0
        self.i_cnt = 0

    def timer_callback(self):
        if(self.i_ptr < len(self.instructions)):
            self.publish_mvt(self.instructions[self.i_ptr])
            if(self.i_cnt >= self.instructions[self.i_ptr][1]):
                
