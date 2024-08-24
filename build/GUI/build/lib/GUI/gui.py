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
from customtkinter import CTkImage
import os


class RoverDashboard(Node):
    def __init__(self):
        super().__init__('rover_dashboard')

        # ROS2 subscriptions
        self.create_subscription(String, '/wheel_speeds', self.wheel_speeds_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Initialize the tkinter window
        self.root = ctk.CTk()
        self.root.title("Rover Dashboard")
        self.root.geometry("1600x900")

        # Configure grid
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=1)

        # Camera Frame
        self.camera_frame = ctk.CTkFrame(self.root, corner_radius=10)
        self.camera_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.camera_label = ctk.CTkLabel(self.camera_frame, width=640, height=480, text="")
        self.camera_label.pack()

        # Map Frame
        self.map_frame = ctk.CTkFrame(self.root, corner_radius=10)
        self.map_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self.map_label = ctk.CTkLabel(self.map_frame, width=640, height=480, text="")
        self.map_label.pack()

        # Terminal Frame for wavemon
        char_width = 8
        char_height = 16
        frame_width = 70 * char_width
        frame_height = 24 * char_height

        self.terminal_frame = ctk.CTkFrame(
            self.root, 
            corner_radius=10,
            width=frame_width, 
            height=frame_height, 
            fg_color="black"
        )

        self.terminal_frame.grid(row=1, column=0, columnspan=1, rowspan=1, padx=0, pady=0, sticky="nsew")
        self.terminal_frame.grid_propagate(False)

        # Embedding wavemon in terminal
        self.embed_terminal()

        # Other UI elements (Speed, Battery, Controls)
        self.linear_speed_var = tk.StringVar(value="Rover speed N/A")
        self.linear_speed_label = ctk.CTkLabel(self.root, textvariable=self.linear_speed_var)
        self.linear_speed_label.grid(row=3, column=1, padx=10, pady=5, sticky="se")

        self.battery_var = tk.StringVar(value="Battery: inchallah%")
        self.battery_label = ctk.CTkLabel(self.root, textvariable=self.battery_var)
        self.battery_label.grid(row=3, column=0, padx=10, pady=5, sticky="se")

        # Initialize wheel speed labels (Bottom Right)
        self.wheel_names = ["Front Right", "Front Left", "Back Right", "Back Left"]
        self.wheel_speed_labels = {}
        for i, name in enumerate(self.wheel_names):
            label = ctk.CTkLabel(self.root, text=f"{name}: N/A")
            label.grid(row=4, column=i, padx=10, pady=2, sticky="se")
            self.wheel_speed_labels[name] = label

        # Control Buttons
        self.autonomous_button = ctk.CTkButton(self.root, text="Switch to Autonomous", command=self.switch_to_autonomous)
        self.autonomous_button.grid(row=1, column=2, padx=5, pady=5, sticky="se")

        self.manual_button = ctk.CTkButton(self.root, text="Switch to Manual", command=self.switch_to_manual)
        self.manual_button.grid(row=1, column=3, padx=5, pady=5, sticky="se")

        # Start ROS2 in a separate thread
        ros_thread = Thread(target=self.run_ros2)
        ros_thread.start()

        # Start the GStreamer video stream in a separate thread
        self.video_thread = Thread(target=self.start_video_stream)
        self.video_thread.start()

    def embed_terminal(self):
        # Ensure the terminal frame is ready
        self.terminal_frame.update_idletasks()
        terminal_id = self.terminal_frame.winfo_id()
        command = f"xterm -into {terminal_id} -geometry 80x24 -fa 'Monospace' -e wavemon &"
        os.system(command)

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
            self.root.after(0, self.update_wheel_speeds)

        except Exception as e:
            self.get_logger().error(f"Error processing wheel speeds: {str(e)}")

    def update_wheel_speeds(self):
        for name, speed in self.wheel_speeds.items():
            self.wheel_speed_labels[name].configure(text=f"{name}: {speed} m/s")

    def odom_callback(self, msg):
        linear_speed = msg.twist.twist.linear.x
        self.linear_speed_var.set(f"odom: {linear_speed:.2f} m/s")

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data).reshape((height, width))

        map_image = np.zeros((height, width), dtype=np.uint8)
        map_image[map_data == -1] = 127  # Unknown areas as gray
        map_image[map_data == 0] = 255   # Free space as white
        map_image[map_data == 100] = 0   # Occupied space as black

        map_image_resized = cv2.resize(map_image, (640, 480), interpolation=cv2.INTER_NEAREST)

        pil_image = Image.fromarray(map_image_resized)
        ctk_img = CTkImage(light_image=pil_image, dark_image=pil_image, size=(640, 480))

        self.map_label.configure(image=ctk_img)
        self.map_label.image = ctk_img

    def switch_to_autonomous(self):
        print("Switched to Autonomous")

    def switch_to_manual(self):
        print("Switched to Manual")

    def update_dashboard(self):
        self.root.after(60, self.update_dashboard)

    def start_video_stream(self):
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
                frame = cv2.resize(frame, (640, 480))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)

                ctk_img = CTkImage(light_image=img, dark_image=img, size=(640, 480))

                self.camera_label.configure(image=ctk_img)
                self.camera_label.image = ctk_img

            else:
                self.get_logger().warning("Failed to capture video frame.")

        cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = RoverDashboard()

    node.update_dashboard()
    node.root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
