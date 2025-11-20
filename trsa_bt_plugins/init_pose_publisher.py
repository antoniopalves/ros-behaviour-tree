#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__(
            'initial_pose_publisher',
            allow_undeclared_parameters=True
        )

        # usar sim time
        self.set_parameters([ 
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.already_published = False

    def timer_callback(self):
        if self.already_published:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'

        # evitar timestamp real → AMCL aceita sem erro
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.pose.pose.position.x = -1
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().info('Initial pose published')
        self.already_published = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
