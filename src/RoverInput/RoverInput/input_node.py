import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses
import time

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(String, 'rover_input', 10)
        # self.timer_period = 0.1
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Input node started')

    def capture_input(self):
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(False)
        stdscr.nodelay(True)
        stdscr.refresh()

        user_input = "stop"
        
        try:
            while rclpy.ok():
                ch = stdscr.getch()
                if ch == ord('w'):
                    user_input = "forward"
                elif ch == ord('a'):
                    user_input = "left"
                elif ch == ord('s'):
                    user_input = "backwards"
                elif ch == ord('d'):
                    user_input = "right"
                else:
                    user_input = "stop"
                msg = String()
                msg.data = user_input
                self.publisher_.publish(msg)

        finally:
            curses.endwin()                    

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    try:
        node.capture_input()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
