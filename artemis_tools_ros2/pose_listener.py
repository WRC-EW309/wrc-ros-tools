from functools import partial
import math
import numpy as np
import quaternion
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import tf2
import tf2_tools



class OdomPublisher(Node):

    def __init__(self):
        super().__init__('vrpn_odom_publisher')


        self.declare_parameter('rigid_bodies', ['rover','drone'])
        self.names = my_param = self.get_parameter('rigid_bodies').get_parameter_value().string_value
        self.names = ['koala1','koala2','koala3','koala4']
        self.names_n = len(self.names)
        self.sub_list = []
        self.pub_list = []
        self.pose_list = []
        self.pos_list = []
        self.old_pos_list = []
        self.vel_est_list = []
        self.old_pose_list = []
        self.last_time_list = []
        self.adv_indx = 0
        self.send_indx = 0
        self.get_clock().now()

        for current_name in self.names:
            sub = self.create_subscription(PoseStamped,'/'+current_name+'/pose',partial(self.pose_callback,current_name),10)
            self.sub_list.append(sub)
            pub = self.create_publisher(Odometry, '/'+current_name+'/mocap/odom', 10)
            self.pub_list.append(pub)
            tmp_pose = PoseStamped()
            self.pose_list.append(tmp_pose)
            self.last_time_list.append(self.get_clock().now())
            pos = np.zeros([1,3])
            self.pos_list.append(pos)
            old_pos = np.zeros([1,3])
            self.old_pos_list.append(old_pos)
            vel = np.zeros([1,3])
            self.vel_est_list.append(vel)

    def pose_callback(self,name,msg):

        ind = self.names.index(name)

        time_dur = self.get_clock().now() - self.last_time_list[ind]
        dt = (time_dur.nanoseconds)*(1e-9)
        q_vrpn = np.quaternion(msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            )

        # Rotations from VRPN frame back to Artemis Frame
        q_rot = quaternion.from_euler_angles(0,math.pi/2,math.pi/2)
        q_pos = np.quaternion(0,msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
        q_pos_arty =  q_rot*(q_pos)*np.conjugate(q_rot)

        # Rotate VRPN Frame to Artemis Frame
        q_arty = q_rot*q_vrpn

        self.pos_list[ind][0,:] = np.array([q_pos_arty.x,q_pos_arty.y,q_pos_arty.z])

        a = 1/0.1
        self.vel_est_list[ind][0,:] = (1-a*dt)*self.vel_est_list[ind][0,:] + a*(self.pos_list[ind][0,:] - self.old_pos_list[ind][0,:])
        vel = np.quaternion(0,self.vel_est_list[ind][0,0],self.vel_est_list[ind][0,1],self.vel_est_list[ind][0,2])
        vel_body = np.conjugate(q_arty)*vel*q_arty


        self.old_pos_list[ind][0,:] = self.pos_list[ind][0,:]
        self.last_time_list[ind] = self.get_clock().now()

        odom_cmd = Odometry()
        odom_cmd.header.frame_id = 'mocap_odom'
        odom_cmd.child_frame_id = 'base_link'
        odom_cmd.header.stamp.sec = self.get_clock().now().to_msg().sec
        odom_cmd.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        odom_cmd.pose.pose.position.x = q_pos_arty.x
        odom_cmd.pose.pose.position.y = q_pos_arty.y
        odom_cmd.pose.pose.position.z = q_pos_arty.z

        odom_cmd.pose.pose.orientation.w = q_arty.w
        odom_cmd.pose.pose.orientation.x = q_arty.x
        odom_cmd.pose.pose.orientation.y = q_arty.y
        odom_cmd.pose.pose.orientation.z = q_arty.z


        odom_cmd.twist.twist.linear.x = vel_body.x
        odom_cmd.twist.twist.linear.y = vel_body.y
        odom_cmd.twist.twist.linear.z = vel_body.z

        self.pub_list[ind].publish(odom_cmd)



def main(args=None):
    rclpy.init(args=args)

    my_odom_pub = OdomPublisher()

    rclpy.spin(my_odom_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_odom_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
