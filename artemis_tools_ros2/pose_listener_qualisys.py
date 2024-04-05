from functools import partial
import math
import numpy as np
import quaternion
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from tf2_ros import tf2
import tf2_tools



class OdomPublisher(Node):

    def __init__(self):
        super().__init__('vrpn_odom_publisher')


        self.declare_parameter('rigid_bodies', ['rover','drone'])
        self.names = self.get_parameter('rigid_bodies').get_parameter_value().string_value
        self.names = ['daisy']
        self.names_n = len(self.names)
        self.sub_list = []
        self.pub_list = []
        self.vel_pub_list = []
        self.eul_pub_list = []
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
            sub = self.create_subscription(PoseStamped,'/qualisys/'+current_name+'/pose',partial(self.pose_callback,current_name),10)
            self.sub_list.append(sub)
            pub = self.create_publisher(Odometry, '/'+current_name+'/mocap/odom', 10)
            self.pub_list.append(pub)
            vel_pub = self.create_publisher(Vector3Stamped,'/'+current_name+'/vel',10)
            self.vel_pub_list.append(vel_pub)
            eul_pub = self.create_publisher(Vector3Stamped,'/'+current_name+'/eul',10)
            self.eul_pub_list.append(eul_pub)
            
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
        q_arty = np.quaternion(msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            )

        R = quaternion.as_rotation_matrix(q_arty)
        yaw = -math.atan2(R[0][1],R[0][0])
        pitch = -math.asin(R[2][0])
        roll = math.atan2(R[2][1],R[2][2])

        # Rotations from VRPN frame back to Artemis Frame
       # q_rot = quaternion.from_euler_angles(0,math.pi/2,math.pi/2)
       # q_pos = np.quaternion(0,msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
       # q_pos_arty =  q_rot*(q_pos)*np.conjugate(q_rot)

        # Rotate VRPN Frame to Artemis Frame
       # q_arty = q_rot*q_vrpn

        self.pos_list[ind][0,:] = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])

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
        odom_cmd.pose.pose.position.x = msg.pose.position.x
        odom_cmd.pose.pose.position.y = msg.pose.position.y
        odom_cmd.pose.pose.position.z = msg.pose.position.z

        odom_cmd.pose.pose.orientation.w = q_arty.w
        odom_cmd.pose.pose.orientation.x = q_arty.x
        odom_cmd.pose.pose.orientation.y = q_arty.y
        odom_cmd.pose.pose.orientation.z = q_arty.z

        odom_cmd.twist.twist.linear.x = vel_body.x
        odom_cmd.twist.twist.linear.y = vel_body.y
        odom_cmd.twist.twist.linear.z = vel_body.z

        #print(odom_cmd)
        self.pub_list[ind].publish(odom_cmd)

        vel_msg = Vector3Stamped()
        vel_msg.header.frame_id = 'world'
        vel_msg.header.stamp.sec = odom_cmd.header.stamp.sec
        vel_msg.header.stamp.nanosec = odom_cmd.header.stamp.nanosec
        vel_msg.vector.x = self.vel_est_list[ind][0,0]
        vel_msg.vector.y = self.vel_est_list[ind][0,1]
        vel_msg.vector.z = self.vel_est_list[ind][0,2]
	
        self.vel_pub_list[ind].publish(vel_msg)

        eul_msg = Vector3Stamped()
        eul_msg.header.frame_id = 'world'
        eul_msg.header.stamp.sec = odom_cmd.header.stamp.sec
        eul_msg.header.stamp.nanosec = odom_cmd.header.stamp.nanosec
        eul_msg.vector.x = roll
        eul_msg.vector.y = pitch
        eul_msg.vector.z = yaw
        self.eul_pub_list[ind].publish(eul_msg)
	

def main(args=None):
    rclpy.init(args=args)

    my_odom_pub = OdomPublisher()

    rclpy.spin(my_odom_pub)

    my_odom_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
