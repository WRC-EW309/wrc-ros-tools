from functools import partial
import math
# import numpy as np
# import quaternion
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import tf2
import tf2_tools


def check_for_nan(pose_iter):
    # Iterate over all the fields in the object's __dict__ (i.e., attributes)
    for val in pose_iter:
        if(math.isnan(val)):
            return True
    return False


class RigidBodiesPublisher(Node):

    def __init__(self):
        super().__init__('rigid_bodies_republisher')

        self.rigid_bodies_list = []
        self.pub_list = []

        self.adv_indx = 0
        self.send_indx = 0
        self.get_clock().now()

        self.create_subscription(RigidBodies,'/rigid_bodies',self.rigid_bodies_callback,10)


    def rigid_bodies_callback(self,msg):
        for i in range(len(msg.rigidbodies)):
            curr_name = msg.rigidbodies[i].rigid_body_name
            if(curr_name in self.rigid_bodies_list):
            
                ind = self.rigid_bodies_list.index(curr_name)
                curr_pose = msg.rigidbodies[ind].pose
                pose_iterable = (
                    curr_pose.position.x, curr_pose.position.y, curr_pose.position.z,  # Position fields
                    curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w  # Orientation fields
                )
                if(not check_for_nan(pose_iterable)):
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.pose = curr_pose
                    self.pub_list[ind].publish(pose_msg)
                    
            else:
                self.rigid_bodies_list.append(curr_name)
                self.get_logger().info(f"Adding pose publisher for rigid body {curr_name} on topic '/{curr_name}/pose")
                pub = self.create_publisher(PoseStamped, '/'+curr_name+'/pose', 10)
                self.pub_list.append(pub)


def main(args=None):
    rclpy.init(args=args)

    rbp = RigidBodiesPublisher()

    rclpy.spin(rbp)

    rbp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
