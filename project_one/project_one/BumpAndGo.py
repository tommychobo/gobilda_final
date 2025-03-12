import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class BumpAndGo(Node):

    def __init__(self):
        super().__init__('bump_and_go')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
            )
        self.subscription
        
        self.max_sensing_dist = 1.0
        self.scanning_width = 0.45 


        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.mode = 0
        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = 1.0
        self.backwards_msg = TwistStamped()
        self.backwards_msg.twist.linear.x = -1.0
        self.turning_msg = TwistStamped()
        self.turning_msg.twist.angular.z = np.pi / 2
        self.stop_msg = TwistStamped()
        self.stop_msg.twist.linear.x = 0.0
        self.stop_msg.twist.angular.z = 0.0


        self.publisher_.publish(self.forward_msg)
        self.get_logger().info('Going forward!')

    def listener_callback(self, msg):
            
        self.get_logger().info('listener callback triggered')
        index_0 = int((msg.angle_max - msg.angle_min) / (2 * msg.angle_increment))
        #self.forward_distance = msg.ranges[index_0]
        
        alpha = np.arcsin(self.scanning_width/2)
        # We are trying to calculate the minimum forward distance within our scanning box
        closest_obj = 10000000
        filtered_range = []
        angle = msg.angle_min
        for r in msg.ranges:
            if(-np.pi/2 < angle and angle < np.pi/2):
                filtered_range.append((angle, r))
            angle += msg.angle_increment
        
        for i in filtered_range:
            sine_theta = np.abs(np.sin(i[0]))

            # this keeps us safe from a div by zero (!!!)
            if(np.abs(sine_theta) < 0.005):
                sine_theta = 0.005
            
            max_dist = min(self.max_sensing_dist, self.scanning_width/(2*sine_theta))
            if(i[1] < max_dist and i[1] < closest_obj):
                closest_obj = i[1]

        self.forward_distance = closest_obj
        if self.mode == 0:

            if self.forward_distance <= self.max_sensing_dist: # 1 ft = 0.3048 m
                self.publisher_.publish(self.stop_msg)
                self.get_logger().info('Stopped!')
                self.mode += 1
            else:
                self.get_logger().info('Going forward!')
                self.publisher_.publish(self.forward_msg)

        elif self.mode == 3:

            if self.forward_distance >= self.max_sensing_dist:

                self.publisher_.publish(self.forward_msg)
                self.get_logger().info('Going forward!')
                self.mode = 0

            else:
                
                self.get_logger().info('Turning!')
                self.publisher_.publish(self.turning_msg)

    def timer_callback(self):
        self.get_logger().info('timer_callback triggered')

        if self.mode == 1:# this is just a 1 second delay making the robot pause
            self.mode = 5

        elif self.mode == 5:
            self.publisher_.publish(self.backwards_msg)
            self.get_logger().info('Going backwards!')
            self.mode = 2

        elif self.mode == 2:

            self.publisher_.publish(self.turning_msg)
            self.get_logger().info('Turning!')
            self.mode += 1

def main(args=None):
    rclpy.init(args=args)

    bump_and_go = BumpAndGo()

    rclpy.spin(bump_and_go)

    bump_and_go.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
