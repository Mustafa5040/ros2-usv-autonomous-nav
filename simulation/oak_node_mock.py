from pathlib import Path
import sys

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
from vision_msgs.msg import ObjectHypothesis
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from gazebo_msgs.msg import ModelStates
from math import atan2, hypot, pi, radians, sin, cos, degrees

from utils.parameters import YOLO_CLASSES

class OakDNodeMock(Node):
    def __init__(self):
        super().__init__('oak_d_mock_node')
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.model_state_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_cb, self.sensor_qos
        )
        self.det_pub = self.create_publisher(SpatialDetectionArray, '/oak/nn/spatial_detections', self.sensor_qos)
        self.sc_pub = self.create_publisher(LaserScan, '/oak/rgbd/scan', self.sensor_qos)
        
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.robot_name = "ertugrul_ida"
        self.robot_found = False
        self.robot_name_idx = None
        self.max_dist = 25.0
        self.fov_deg = 100.0

        self.buoy_width = 0.6
        self.buoy_height = 0.3
    def model_states_cb(self, msg: ModelStates):
        if not msg: return
        try:
            self.robot_name_idx = msg.name.index(self.robot_name)
            if not self.robot_found:
                self.get_logger().info(f"Robot '{self.robot_name} found in GAZEBO...'")
                self.robot_found = True
            
            # robot coordinates
            rx = msg.pose[self.robot_name_idx].position.x
            ry = msg.pose[self.robot_name_idx].position.y
            rz = msg.pose[self.robot_name_idx].position.z

            #robot yaw
            q = msg.pose[self.robot_name_idx].orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            ryaw = atan2(siny_cosp, cosy_cosp)

        except ValueError:
            if not self.robot_found: 
                 pass
        # Publish SpatialDetections
        det_arr_msg = SpatialDetectionArray()
        det_arr_msg.header.stamp = self.get_clock().now().to_msg()
        det_arr_msg.header.frame_id = 'base_link'

        for i, name in enumerate(msg.name):
            
            if name == self.robot_name: continue
            name = name.rsplit("_",1)[0]
                
            if name not in YOLO_CLASSES:
                    continue

            bx = msg.pose[i].position.x
            by =  msg.pose[i].position.y
            bz = msg.pose[i].position.z

            dx = bx - rx
            dy = by - ry
            dist = hypot(dx,dy)

            if dist < self.max_dist:
                angle_to_obj = atan2(dy,dx)
                rel_angle = self.normalize_angle(angle_to_obj - ryaw)

                if abs(rel_angle) < radians(self.fov_deg / 2.0):
                    sd = SpatialDetection()
                    hyp = ObjectHypothesis()
                    hyp.class_id = str(name)
                    hyp.score = 0.5
                    sd.results.append(hyp)
                    sd.bbox.center.position.x = 320.0
                    sd.bbox.center.position.y = 320.0
                    sd.bbox.size_x = float(self.buoy_width)
                    sd.bbox.size_y = float(self.buoy_height)
                    sd.position.x = float(dist * cos(rel_angle))
                    sd.position.y = float(dist * sin(rel_angle))
                    sd.position.z = 0.0
                    det_arr_msg.detections.append(sd)
                    self.get_logger().info(f"Name: {name}, PositionY: {sd.position.y}, PositionX: {sd.position.x}")
                else:
                        self.get_logger().info(f"[DELETED - FOV] {name} Degree:: {degrees(rel_angle):.1f}")
        self.det_pub.publish(det_arr_msg)

    def normalize_angle(self, angle):
        while angle > pi: angle -= 2.0 * pi
        while angle < -pi: angle += 2.0 * pi
        return angle
def main():
    rclpy.init()
    node = OakDNodeMock()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
