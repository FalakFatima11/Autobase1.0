import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
import numpy as np

class WaypointPublisher:
    def __init__(self):
        rospy.init_node('waypoint_publisher_node', anonymous=True)

        # Publish waypoints on the "/local_planner/waypoints" topic
        self.waypoint_pub = rospy.Publisher("/local_planner/waypoints", Pose2D, queue_size=1)

        # Set the publishing rate (Hz)
        self.rate = rospy.Rate(1)  # 1 Hz, change this value based on your desired publishing frequency

    def publish_waypoints(self):
        # Create a straight line of waypoints along the x-axis
        num_waypoints = 10

        for i in range(num_waypoints):
            # Create a Pose2D message
            waypoint_msg = Pose2D()

            waypoint_msg.x = i  # X-coordinate
            waypoint_msg.y = 0  # Y-coordinate
            waypoint_msg.theta = 0  # Orientation (theta)

            # Publish the Pose2D message
            self.waypoint_pub.publish(waypoint_msg)

            # Sleep to maintain the desired publishing rate
            self.rate.sleep()

    def run(self):
        while not rospy.is_shutdown():
            self.publish_waypoints()

if __name__ == '__main__':
    try:
        waypoint_publisher = WaypointPublisher()
        waypoint_publisher.run()
    except rospy.ROSInterruptException:
        pass