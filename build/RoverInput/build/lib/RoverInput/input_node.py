import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        #reliability best effort qos profile for the publisher (UDP-like)
        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                               history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                               depth=1)
        self.publisher_ = self.create_publisher(String, 'rover_input', 10)
        # self.timer_period = 0.1
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Input node started')

    def capture_input(self):
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(False)

        user_input = "RV2stop"
        
        try:
            while rclpy.ok():
                ch = stdscr.getch()
                if ch == ord('w'):
                    user_input = "RV2forward"
                elif ch == ord('a'):
                    user_input = "RV2left"
                elif ch == ord('s'):
                    user_input = "RV2backwards"
                elif ch == ord('d'):
                    user_input = "RV2right"
                else:
                    user_input = "RV2stop"
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
