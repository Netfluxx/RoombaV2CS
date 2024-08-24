import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
import tkinter as tk
import customtkinter as ctk
from threading import Thread
import cv2
from PIL import Image
import numpy as np
from customtkinter import CTkImage  # Import CTkImage

class RoverDashboard(Node):
    def __init__(self):
        super().__init__('rover_dashboard')

        # Create subscriber to the topic /wheel_speeds
        self.subscription = self.create_subscription(
            String,
            '/wheel_speeds',
            self.wheel_speeds_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.wheel_speeds_list = []

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Initialize the tkinter window
        self.root = ctk.CTk()
        self.root.title("Rover Dashboard")
        self.root.geometry("1000x800")

        # Create frames for different sections
        self.speed_frame = ctk.CTkFrame(self.root, corner_radius=10)
        self.speed_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

        self.linear_speed_frame = ctk.CTkFrame(self.root, corner_radius=10)
        self.linear_speed_frame.grid(row=0, column=1, padx=10, pady=10, sticky="ew")

        self.battery_frame = ctk.CTkFrame(self.root, corner_radius=10)
        self.battery_frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")

        self.control_frame = ctk.CTkFrame(self.root, corner_radius=10)
        self.control_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="ew")

        # Camera Frame using Label
        self.camera_frame = ctk.CTkFrame(self.root, corner_radius=10)
        self.camera_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=10, sticky="ew")

        # Initialize the CTkLabel with no default text
        self.camera_label = ctk.CTkLabel(self.camera_frame, width=640, height=480, text="")
        self.camera_label.pack()


        # Linear Speed of the rover
        self.linear_speed_var = tk.StringVar(value="Rover speed N/A")
        self.linear_speed_label = ctk.CTkLabel(self.linear_speed_frame, textvariable=self.linear_speed_var)
        self.linear_speed_label.pack()

        # Battery Level
        self.battery_var = tk.StringVar(value="battery: inchallah%")
        self.battery_label = ctk.CTkLabel(self.battery_frame, textvariable=self.battery_var)
        self.battery_label.pack()

        # Control Buttons
        self.autonomous_button = ctk.CTkButton(self.control_frame, text="Switch to Autonomous", command=self.switch_to_autonomous)
        self.autonomous_button.grid(row=0, column=0, padx=5, pady=5)

        self.manual_button = ctk.CTkButton(self.control_frame, text="Switch to Manual", command=self.switch_to_manual)
        self.manual_button.grid(row=0, column=1, padx=5, pady=5)

        # Initialize wheel speed labels
        self.wheel_names = ["Front Right", "Front Left", "Back Right", "Back Left"]
        self.wheel_speed_labels = {}
        for i, name in enumerate(self.wheel_names):
            label = ctk.CTkLabel(self.speed_frame, text=f"{name}: N/A")
            label.grid(row=i, column=0, padx=5, pady=5)
            self.wheel_speed_labels[name] = label

        # Start ROS2 in a separate thread
        ros_thread = Thread(target=self.run_ros2)
        ros_thread.start()

        # Start the GStreamer video stream in a separate thread
        self.video_thread = Thread(target=self.start_video_stream)
        self.video_thread.start()

    def run_ros2(self):
        rclpy.spin(self)

    def wheel_speeds_callback(self, msg):
        try:
            self.wheel_speeds_list = msg.data.split(",")
            if len(self.wheel_speeds_list) != 4:
                self.get_logger().error(f"Received incorrect number of wheel speeds: {self.wheel_speeds_list}")
                return

            self.wheel_speeds_list = [f"{float(speed):.2f}" for speed in self.wheel_speeds_list]
            self.wheel_speeds = dict(zip(self.wheel_names, self.wheel_speeds_list))
            # Update wheel speeds in GUI as soon as they are received
            self.root.after(0, self.update_wheel_speeds)

        except Exception as e:
            self.get_logger().error(f"Error processing wheel speeds: {str(e)}")

    def update_wheel_speeds(self):
        for name, speed in self.wheel_speeds.items():
            self.wheel_speed_labels[name].configure(text=f"{name}: {speed} m/s")

    def odom_callback(self, msg):
        # Process odometry data
        linear_speed = msg.twist.twist.linear.x
        self.linear_speed_var.set(f"odom: {linear_speed:.2f} m/s")

    def map_callback(self, msg):
        # Extract the width, height, and data of the occupancy grid
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data).reshape((height, width))

        # Convert the map data to an 8-bit grayscale image
        # -1 is unknown, 0 is free, 100 is occupied
        map_image = np.zeros((height, width), dtype=np.uint8)
        map_image[map_data == -1] = 127  # Unknown areas as gray
        map_image[map_data == 0] = 255   # Free space as white
        map_image[map_data == 100] = 0   # Occupied space as black

        # Optionally resize the map to fit your GUI's layout
        map_image_resized = cv2.resize(map_image, (640, 480), interpolation=cv2.INTER_NEAREST)

        # Convert the numpy array to a PIL image
        pil_image = Image.fromarray(map_image_resized)

        # Convert the PIL image to a CTkImage
        ctk_img = CTkImage(light_image=pil_image, dark_image=pil_image, size=(640, 480))

        # Display the map in the GUI
        self.camera_label.configure(image=ctk_img)
        self.camera_label.image = ctk_img


    def switch_to_autonomous(self):
        # Code to switch to autonomous mode
        print("Switched to Autonomous")

    def switch_to_manual(self):
        # Code to switch to manual mode
        print("Switched to Manual")

    def update_dashboard(self):
        # Update other dashboard elements in real-time if necessary
        self.root.after(60, self.update_dashboard)  # Update every 60ms (arbitrary)

    def start_video_stream(self):
        # Assuming you are receiving an RTP stream via GStreamer:
        gst_pipeline = (
            "udpsrc port=5000 ! "
            "application/x-rtp,encoding-name=JPEG,payload=26 ! "
            "rtpjpegdepay ! jpegdec ! videoconvert ! appsink"
        )

        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            self.get_logger().error("Error: Could not open video stream.")
            return

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # Resize the frame to fit the Label's dimensions
                frame = cv2.resize(frame, (640, 480))  # Match the label's size

                # Convert the frame to ImageTk format for displaying in tkinter
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)

                # Convert PIL image to CTkImage
                ctk_img = CTkImage(light_image=img, dark_image=img, size=(640, 480))

                # Update the Label with the new image
                self.camera_label.configure(image=ctk_img)

                # Prevent the image from being garbage collected
                self.camera_label.image = ctk_img

            else:
                self.get_logger().warning("Failed to capture video frame.")

        cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = RoverDashboard()

    # Start the tkinter main loop in the main thread
    node.update_dashboard()
    node.root.mainloop()

    # Clean up
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
