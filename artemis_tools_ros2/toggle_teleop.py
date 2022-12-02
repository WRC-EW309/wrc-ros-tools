from functools import partial
import math
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy





class TeleopToggle(Node):

    def __init__(self):
        super().__init__('toggle_teleop')


        self.declare_parameter('rigid_bodies', ['rover','drone'])
        self.names = my_param = self.get_parameter('rigid_bodies').get_parameter_value().string_value
        self.names = ['koala1','koala2','koala3','koala4']
        self.names_n = len(self.names)

        self.pub_list = {}

        self.spd_scale = 1.5
        self.yawrate_scale = 3.14
        self.adv_indx = 0
        self.old_joy_msg = Joy()
        self.old_btn4 = 0
        self.send_indx = 0
        self.get_clock().now()
        self.send_flag = False
        self.joy_sub = self.create_subscription(Joy,'/joy',self.joy_callback,10)

        for current_name in self.names:

            pub = self.create_publisher(Twist, '/'+current_name+'/cmd_vel', 10)
            self.pub_list[current_name] = pub

          
    def joy_callback(self,msg):

        cmd_msg = Twist()

        cmd_msg.linear.x = self.spd_scale*msg.axes[1] 
        cmd_msg.angular.z = self.yawrate_scale*msg.axes[3]
        
        #print(cmd_msg)
        if(msg.buttons[4] and msg.buttons[4] != self.old_joy_msg.buttons[4]):
            self.send_indx += 1
            self.send_indx = self.send_indx % self.names_n
            print(self.names[self.send_indx])
            

        if(msg.buttons[5]):
            name = self.names[self.send_indx]
            #print(name)
            self.pub_list[name].publish(cmd_msg)



        self.old_joy_msg = msg





def main(args=None):
    rclpy.init(args=args)

    teleop = TeleopToggle()

    rclpy.spin(teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
