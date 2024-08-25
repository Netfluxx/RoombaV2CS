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
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random

class RoverDashboard(Node):
    def __init__(self):
        super().__init__('rover_dashboard')

        # ROS2 subscriptions
        self.create_subscription(String, '/wheel_speeds', self.wheel_speeds_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(String, '/battery_and_cpu', self.battery_callback, 10)
        
        #ROS2 publisher for buttons to switch between manual and autonomous
        self.rover_mode_publisher = self.create_publisher(String, '/rover_mode', 10)


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
        self.wheel_speeds = {name: 0 for name in self.wheel_names}  # Initialize wheel speeds
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

        # Initialize matplotlib figure for the wheel speeds graph
        self.fig, self.ax = plt.subplots(figsize=(5, 4))
        self.ax.set_title("Wheel Speeds Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.set_ylim(-1, 5)  # Adjust limits based on expected speeds
        self.time_data = []
        self.speed_data = {name: [] for name in self.wheel_names}
        self.lines = {name: self.ax.plot([], [], label=name)[0] for name in self.wheel_names}

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=1, padx=10, pady=10, sticky="nsew")
        self.canvas.draw()

        # Start ROS2 in a separate thread
        ros_thread = Thread(target=self.run_ros2)
        ros_thread.start()

        # Start the GStreamer video stream in a separate thread
        self.video_thread = Thread(target=self.start_video_stream)
        self.video_thread.start()

        # Test updating wheel speeds manually
        self.root.after(1000, self.test_update_speeds)

    def embed_terminal(self):
        # Ensure the terminal frame is ready
        self.terminal_frame.update_idletasks()
        terminal_id = self.terminal_frame.winfo_id()
        command = f"xterm -into {terminal_id} -geometry 76x24 -fa 'Monospace' -e wavemon &"
        os.system(command)

    def run_ros2(self):
        rclpy.spin(self)

    def wheel_speeds_callback(self, msg):
        try:
            print("Wheel speeds callback triggered")  # Debugging statement
            # Generate random speeds for testing
            self.wheel_speeds_list = [random.uniform(0, 5) for _ in range(4)]
            for i, name in enumerate(self.wheel_names):
                self.wheel_speeds[name] = self.wheel_speeds_list[i]

            # Update the wheel speed labels
            for name, speed in self.wheel_speeds.items():
                self.wheel_speed_labels[name].configure(text=f"{name}: {speed:.2f} m/s")

            # Update the graph with the new wheel speeds
            self.root.after(0, self.update_graph)

        except Exception as e:
            self.get_logger().error(f"Error processing wheel speeds: {str(e)}")

    def update_graph(self):
        time_point = len(self.time_data) / 10  # Simulate time (assuming 10Hz data rate)
        self.time_data.append(time_point)
        
        # Keep the graph focused on the last 10 seconds
        current_time = self.time_data[-1]
        time_window_start = max(0, current_time - 10)

        for name in self.wheel_names:
            self.speed_data[name].append(self.wheel_speeds[name])
            self.lines[name].set_data(self.time_data, self.speed_data[name])

        self.ax.set_xlim(time_window_start, current_time+2)  # Focus on the last 10 seconds
        
        # Update y-axis limits based on the current data range
        min_speed = min([min(speeds) for speeds in self.speed_data.values() if speeds])
        max_speed = max([max(speeds) for speeds in self.speed_data.values() if speeds])
        self.ax.set_ylim(min(min_speed, 0), max(max_speed, 5))

        self.ax.legend(loc="upper right")
        self.canvas.draw()


    def test_update_speeds(self):
        # Simulate wheel speeds to manually trigger updates
        print("Testing update speeds manually")
        self.wheel_speeds_callback(None)  # Passing None since we're not using the msg

        # Re-trigger every second for testing
        self.root.after(1000, self.test_update_speeds)

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
        msg = String()
        msg.data = "autonomous"
        self.rover_mode_publisher.publish(msg)

    def switch_to_manual(self):
        print("Switched to Manual")
        msg = String()
        msg.data = "manual"
        self.rover_mode_publisher.publish(msg)

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
