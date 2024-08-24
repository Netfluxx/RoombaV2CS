import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData
import numpy as np

class FakeOccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('fake_occupancy_grid_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_fake_map)

    def publish_fake_map(self):
        # Create the OccupancyGrid message
        msg = OccupancyGrid()

        # Set the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Set the map metadata
        msg.info = MapMetaData()
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = 0.05  # Each cell represents 5cm
        msg.info.width = 100  # 100 cells wide (5m)
        msg.info.height = 100  # 100 cells tall (5m)
        msg.info.origin = Pose()  # Origin of the map

        # Create the data for the occupancy grid
        # -1: unknown, 0: free, 100: occupied
        map_data = np.full((msg.info.height, msg.info.width), -1, dtype=np.int8)  # Initialize with unknowns
        
        # Create some fake obstacles and free space
        map_data[20:30, 20:30] = 100  # A block of occupied cells
        map_data[50:70, 50:70] = 0    # A block of free cells
        
        # Flatten the array to a list as required by OccupancyGrid
        msg.data = map_data.flatten().tolist()

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing fake occupancy grid')

def main(args=None):
    rclpy.init(args=args)
    node = FakeOccupancyGridPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
