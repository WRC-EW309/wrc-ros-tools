import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from std_msgs.msg import UInt32, Float64
import subprocess
import datetime
import os

class RosbagServiceNode(Node):
    def __init__(self):
        super().__init__('rosbag_service_node')

        self.topic = []
        self.ns = self.get_namespace()
        self.declare_parameter("file_header", "default_bag")
        self.declare_parameter("save_dir", "/tmp")
        self.declare_parameter("save_topics", ['rosout'])

        self.bag_name = self.get_parameter("file_header").get_parameter_value().string_value
        self.save_dir = self.get_parameter("save_dir").get_parameter_value().string_value
        self.topics = self.get_parameter("save_topics").get_parameter_value().string_array_value

        self.get_logger().info(f"Bag name {self.bag_name} \r Save dir {self.save_dir} \r Topic list {self.topics}")
        self.curr_log_path = ''
        self.log_topics = []
        for topic in self.topics:
            self.log_topics.append(f'{self.ns+topic}')

        self.get_logger().info(f"log topics are: {self.log_topics}")

        # Declare service with name 'control_rosbag' and type ControlRosbag
        self.start_srv = self.create_service(Trigger, 'start_log', self.start_rosbag_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_log', self.stop_rosbag_callback)

        self.get_logger().info('Rosbag Service Node has been started.')

        self.bytes_pub = self.create_publisher(UInt32, '/rosbag_service/bag_size', 10)
        self.duration_pub = self.create_publisher(Float64, '/rosbag_service/duration', 10)
        self.log_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timer_callback)
        # State of rosbag process
        self.rosbag_process = None

    def timer_callback(self):
        if self.rosbag_process is not None:
            # Publish the current size of the rosbag file
            try:
                self.bytes_pub.publish(UInt32(data=self.get_bag_file_size()))
                # Publish the current duration of the rosbag recording
                self.duration_pub.publish(Float64(data=self.get_bag_recording_duration()))
            except Exception as e:
                self.get_logger().error(f"Error Publishing Log Stats: {e}")

    def get_bag_file_size(self):
        if self.rosbag_process is not None:
            
            self.get_logger().info(f"Current log path: {self.curr_log_path}")
            bag_name_prefix = os.path.basename(self.bag_name)
            if os.path.exists(self.curr_log_path):

                files = os.listdir(self.curr_log_path)
                matching_files = [f for f in files if f.startswith(bag_name_prefix)]
                #self.get_logger().info(f"Matching files: {matching_files}")
                total_size = 0
                for f in matching_files:
                    size = os.path.getsize(os.path.join(self.curr_log_path, f))
                    total_size += size

                return total_size  # Convert to MB
        return 0
    
    def get_bag_recording_duration(self):
        if self.rosbag_process is not None:
            # Get the start time of the rosbag recording
            duration = self.get_clock().now() - self.log_time
            return float(duration.nanoseconds / 1e9)
        return 0.0
    
    def start_rosbag_callback(self,request,response):

        # Start the rosbag recording
        if self.rosbag_process is None:

            self.bag_name = self.get_parameter("file_header").get_parameter_value().string_value
            self.save_dir = self.get_parameter("save_dir").get_parameter_value().string_value
            self.topics = self.get_parameter("save_topics").get_parameter_value().string_array_value
            self.log_topics = []
            for topic in self.topics:
                if(self.ns != '/'):
                    self.log_topics.append(f'{self.ns+topic}')
                else:
                    self.log_topics.append(topic)

            try:

                date_time_str = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
                file_name = f"{self.bag_name}_{date_time_str}"
                full_path = os.path.join(self.save_dir,file_name)
                self.curr_log_path = full_path
                # self.curr_log_filename = file_name + '.mcap'
                self.log_time = self.get_clock().now()
                self.bag_name = file_name
                self.get_logger().info(f"Starting rosbag with topics: {self.log_topics} \r Saving to {self.curr_log_path}")
                # Start rosbag in subprocess (ensure the correct arguments for your system)
                self.rosbag_process = subprocess.Popen(['ros2', 'bag', 'record',
                                                        #'-s','mcap',
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