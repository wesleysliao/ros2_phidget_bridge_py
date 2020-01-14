import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header


class PhidgetBridgeNode(Node):


    def __init__(self):
        super().__init__('phidgetbridge_node')

        self.pub_force = self.create_publisher(WrenchStamped, 
                                               'load_cell_force', 
                                               1)
        self.timer_force = self.create_timer(1, self.timer_callback)


    def timer_callback(self):
        self.pub_force.publish(
            WrenchStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id="ball_knob_1"),
                wrench=Wrench(
                    force=Vector3(x=10.0, y=0.0, z=0.0),
                    torque=Vector3(x=0.0, y=0.0, z=0.0))))




def main(args = None):
    rclpy.init(args=args)

    pbnode = PhidgetBridgeNode()

    rclpy.spin(pbnode)

    rclpy.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
