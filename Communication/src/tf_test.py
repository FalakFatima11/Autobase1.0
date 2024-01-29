#!/usr/bin/env python

import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('list_frames_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1)  # Give time for tf buffer to populate

    try:
        # Get the list of available frames as a string
        all_frames_string = tf_buffer.all_frames_as_string()

        rospy.loginfo("List of available frames:\n%s", all_frames_string)

    except rospy.ROSInterruptException:
        pass
