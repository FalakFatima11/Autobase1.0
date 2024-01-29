#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs  # Import this for transforming geometry_msgs types

class TransformListenerNode:
    def __init__(self):
        rospy.init_node('transform_listener_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster =tf2_ros.TransformBroadcaster()
    def transform_listener(self):
        rate = rospy.Rate(10.0)  # Adjust the rate as needed

        while not rospy.is_shutdown():
            try:
                # print("HI")
                # Try to get the latest transform from 'map' to 'base_link'
                transform = self.tf_buffer.lookup_transform('camera', 'base_link', rospy.Time(0))

                # Access the translation and rotation from the transform
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                # print("Hello")
                print(type(translation))
                print("Let's try indexing")
                print(f"I am value at first index of translation {translation.x}")
                
                rospy.loginfo("Translation: {}, Rotation: {}".format(translation, rotation))

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Handle exceptions when the transform is not available
                rospy.logwarn("Could not get transform between 'map' and 'base_link'")

            rate.sleep()

if __name__ == '__main__':
    try:
        listener_node = TransformListenerNode()
        listener_node.transform_listener()
    except rospy.ROSInterruptException:
        pass
