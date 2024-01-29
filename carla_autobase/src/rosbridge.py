import rospy
import std_msgs
import sensor_msgs
from sensor_msgs.msg import PointCloud2

import numpy as np

rospy.init_node('X')

import sys
sys.path.append("communication/msg")
from communication.msg import CarControls,CarState

class RosCom:
    def __init__(self) -> None:

        self.points_publisher = rospy.Publisher(
            '/velodyne_points', PointCloud2, queue_size=1)
        
        '''
        /cmd_vel
        control_command.angular.z = steer
        control_command.linear.x = self.target_speed
        '''
        self.ll_cmd_subscriber = rospy.Subscriber("ll_cmd", CarControls, self.ll_cmd_callback)
        self.ll_state_publisher = rospy.Publisher("ll_state", CarState, queue_size=1)
        self.ego_th = 0
        self.ego_br = 0
        self.ego_st = 0

        self.ll_state = CarState()
        self.ll_state.gear = "temp"
        self.ll_state.driving_mode = "auto"
        self.ll_state.throttle = self.ego_th
        self.ll_state.angle = self.ego_st
        self.ll_state.velocity = 0
        self.ll_state.brake  = self.ego_br

    def publish_state(self, velocity):
        self.ll_state.throttle = self.ego_th
        self.ll_state.angle = self.ego_st
        self.ll_state.velocity = velocity
        self.ll_state.brake = self.ego_br
        self.ll_state_publisher.publish(self.ll_state)
    

    def ll_cmd_callback(self, ll_cmd):
        self.ego_th = ll_cmd.throttle
        self.ego_br = ll_cmd.brake
        self.ego_st = ll_cmd.angle
        
    def pcd_2_point_cloud(self, points, parent_frame, frametime):
        assert points.shape[1] == 4, 'PCD should be in XYZI format!'
        ros_dtype = sensor_msgs.msg.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = points.astype(dtype).tobytes()
        fields = [
            sensor_msgs.msg.PointField(
                name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate(['x', 'y', 'z', 'intensity'])
        ]
        header = std_msgs.msg.Header(frame_id=parent_frame, stamp=frametime)

        return sensor_msgs.msg.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 4),
            row_step=(itemsize * 4 * points.shape[0]),
            data=data
        )

    def publish_points(self, pcd):
        assert pcd.shape[1] == 3, 'PCD should be in XYZ format'
        pcd = np.hstack([pcd, np.ones((pcd.shape[0], 1))])
        ros_pcd = self.pcd_2_point_cloud(pcd, 'velodyne', rospy.Time.now())
        self.points_publisher.publish(ros_pcd)