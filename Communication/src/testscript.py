#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class WaypointPublisher:
    def __init__(self):
        rospy.init_node('waypoint_publisher_node', anonymous=True)

        # Publish waypoints on the "/local_planner/waypoints" topic
        self.waypoint_pub = rospy.Publisher("/local_planner/waypoints", Path, queue_size=1)
        self.waypoints_info_pub = rospy.Publisher("/local_planner/info", Twist, queue_size=1, latch=True)

        # Set the publishing rate (Hz)
        self.rate = rospy.Rate(1)  # 1 Hz, change this value based on your desired publishing frequency

        # Initialize a TF listener
        self.tf_listener = tf.TransformListener()

        # Initialize log_flags if it's supposed to be a class attribute
        self.log_flags = {'tf_error': False}

    def publish_waypoints(self):
        # Create a straight line of waypoints along the x-axis
        num_waypoints = 10

        # Create a Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for i in range(num_waypoints):
            # Create a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"

            # Set the X and Y coordinates of the waypoints
            pose_msg.pose.position.x = i  # X-coordinate
            pose_msg.pose.position.y = 0  # Y-coordinate
            pose_msg.pose.position.z = 0  # Z-coordinate

            # Convert the numpy array to a Quaternion message
            quat_msg = Quaternion(*quaternion_from_euler(0, 0, 0))
            pose_msg.pose.orientation = quat_msg

            # Add the pose to the Path message
            path_msg.poses.append(pose_msg)

        # Publish the Path message
        self.waypoint_pub.publish(path_msg)

    def publish_local_plan_info(self, safe_speed, curvature, nearest_obstacle_dist):
        waypoints_info = Twist()
        waypoints_info.linear.x = safe_speed
        waypoints_info.angular.z = curvature
        waypoints_info.angular.x = nearest_obstacle_dist
        self.waypoints_info_pub.publish(waypoints_info)

        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            curr_pose = np.array([trans[0], trans[1], euler_from_quaternion(rot)[2]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if not self.log_flags['tf_error']:
                rospy.logwarn('Failed to find current pose')
                self.log_flags['tf_error'] = False

        self.log_flags['tf_error'] = False

    def run(self):
        while not rospy.is_shutdown():
            self.publish_waypoints()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_publisher = WaypointPublisher()
        waypoint_publisher.run()
    except rospy.ROSInterruptException:
        pass
