import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
import subprocess
import datetime
import os

class RosbagServiceNode(Node):
    def __init__(self):
        super().__init__('rosbag_service_node')

        self.topic = []
        self.ns = self.get_namespace()
        self.declare_parameter("file_header",Parameter.Type.STRING)
        self.declare_parameter("save_dir",Parameter.Type.STRING)
        self.declare_parameter("save_topics",Parameter.Type.STRING_ARRAY)

        self.bag_name = self.get_parameter("file_header").get_parameter_value().string_value
        self.save_dir = self.get_parameter("save_dir").get_parameter_value().string_value
        self.topics = self.get_parameter("save_topics").get_parameter_value().string_array_value

        print(f"Bag name {self.bag_name} \r Save dir {self.save_dir} \r Topic list {self.topics}")
        
        self.log_topics = []
        for topic in self.topics:
            self.log_topics.append(f'{self.ns+topic}')

        print(f"log topics are: {self.log_topics}")

        # Declare service with name 'control_rosbag' and type ControlRosbag
        self.start_srv = self.create_service(Trigger, 'start_log', self.start_rosbag_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_log', self.stop_rosbag_callback)

        self.get_logger().info('Rosbag Service Node has been started.')

        # State of rosbag process
        self.rosbag_process = None


    def start_rosbag_callback(self,request,response):

        # Start the rosbag recording
        if self.rosbag_process is None:
            try:

                date_time_str = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
                file_name = f"{self.bag_name}_{date_time_str}"
                full_path = os.path.join(self.save_dir,file_name)

                # Start rosbag in subprocess (ensure the correct arguments for your system)
                self.rosbag_process = subprocess.Popen(['ros2', 'bag', 'record',
                                                        '-s','mcap',
                                                        '-o',full_path]+
                                                        self.log_topics)
                response.success = True
                response.message = "Rosbag started."
            except Exception as e:
                response.success = False
                response.message = f"Error starting rosbag: {str(e)}"
        else:
            response.success = False
            response.message = "Rosbag is already running."

        return response


    def stop_rosbag_callback(self,request,response):

        # Stop the Rosbag recording
        if self.rosbag_process is not None:
            try:
                self.rosbag_process.terminate()  # Stop the rosbag process
                self.rosbag_process = None
                response.success = True
                response.message = "Rosbag stopped."
            except Exception as e:
                response.success = False
                response.message = f"Error stopping rosbag: {str(e)}"
        else:
            response.success = False
            response.message = "No rosbag is currently running."

        return response


def main(args=None):
    rclpy.init(args=args)

    rosbag_service_node = RosbagServiceNode()

    rclpy.spin(rosbag_service_node)

    rosbag_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()