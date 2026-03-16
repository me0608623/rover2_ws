#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from campusrover_msgs.msg import TrackedObstacleArray
from campusrover_msgs.msg import TrackedObstacle
from costmap_converter_msgs.msg import ObstacleArrayMsg


class SimMotNode(Node):
    def __init__(self):
        super().__init__('sim_mot_node')

        self.declare_parameter('tracker_pub_period', 0.05)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('dimensions', 0.5)

        self.tracker_pub_period = self.get_parameter('tracker_pub_period').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.dimensions = self.get_parameter('dimensions').get_parameter_value().double_value

        self.oam = ObstacleArrayMsg()
        self.toa = TrackedObstacleArray()

        self.pub = self.create_publisher(TrackedObstacleArray, 'tracked_label_obstacle', 10)
        self.create_subscription(ObstacleArrayMsg, 'obstacles', self.obs_callback, 10)
        self.create_timer(self.tracker_pub_period, self.timer_callback)

    def obs_callback(self, msg):
        self.oam = msg

    def timer_callback(self):
        self.toa.header.frame_id = self.frame_id
        self.toa.header.stamp = self.get_clock().now().to_msg()

        for obs in self.oam.obstacles:
            is_find = False
            for t_obs in self.toa.obstacles:
                if t_obs.id == obs.id:
                    is_find = True

                    t_obs.velocity.linear.x = (obs.polygon.points[0].x - t_obs.pose.position.x) / self.tracker_pub_period
                    t_obs.velocity.linear.y = (obs.polygon.points[0].y - t_obs.pose.position.y) / self.tracker_pub_period

                    t_obs.pose.position.x = obs.polygon.points[0].x
                    t_obs.pose.position.y = obs.polygon.points[0].y
                    t_obs.pose.position.z = obs.polygon.points[0].z

                    t_obs.is_dynamic = abs(t_obs.velocity.linear.x) > 0.1 and abs(t_obs.velocity.linear.y) > 0.1

            if not is_find:
                to = TrackedObstacle()

                to.id = obs.id
                to.pose.position.x = obs.polygon.points[0].x
                to.pose.position.y = obs.polygon.points[0].y
                to.pose.position.z = obs.polygon.points[0].z

                to.velocity.linear.x = 0.0
                to.velocity.linear.y = 0.0

                to.dimensions.x = self.dimensions

                self.toa.obstacles.append(to)

        self.pub.publish(self.toa)


def main(args=None):
    rclpy.init(args=args)
    node = SimMotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
