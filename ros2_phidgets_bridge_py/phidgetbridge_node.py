import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header


from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput, BridgeGain


class PhidgetBridgeNode(Node):


    def __init__(self,
                 topic,
                 frame_id,

                 bridge_channel,
                 bridge_serial = None,
                 bridge_hubport= 0,

                 data_rate_Hz = 60,

                 gain = BridgeGain.BRIDGE_GAIN_128,

                 force_conversion_multiple = 1.0,
                 force_conversion_offset = 0.0):
        super().__init__('phidgetbridge_node')

        self.topic = topic
        self.frame_id = frame_id
        
        self.pb_ch = VoltageRatioInput()
        self.pb_ch.setChannel(int(bridge_channel))
        self.pb_ch.setHubPort(bridge_hubport)
        self.pb_ch.setIsHubPortDevice(bridge_hubportdevice)
        if bridge_serial is not None:
            self.pb_ch.setDeviceSerialNumber(bridge_serial)
        
        self.data_rate_Hz = data_rate_Hz
        self.data_interval_ms = int((1.0/self.data_rate_Hz)*1000)
        
        self.gain = gain

        self.force_conversion_multiple = float(force_conversion_multiple)
        self.force_conversion_offset = float(force_conversion_offset)


        self.pub_force = self.create_publisher(WrenchStamped, self.topic, 1)

        self.pb_ch.setOnAttachHandler(self.onAttachHandler)
        self.pb_ch.setOnDetachHandler(self.onDetachHandler)
        self.pb_ch.setOnVoltageRatioChangeHandler(self.onVoltageRatioChangeHandler)

        self.pb_ch.openWaitForAttachment(1000)

    def onAttatchHandler(self, channel):
        channel.setDataInterval(self.data_interval_ms)
        channel.setVoltageRatioChangeTrigger(0.0)
        channel.setBridgeGain(self.gain)
        channel.setBridgeEnabled(True)

    def voltageRatioToForce(self, voltageRatio):
        return ((voltage_ratio*self.force_conversion_multiple) 
                 + self.force_conversion_offset)

    def onVoltageRatioChangeHandler(self, channel, voltageRatio):
        self.pub_force.publish(
            WrenchStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self.frame_id),
                wrench=Wrench(
                    force=Vector3(x=self.voltageTatioToForce(voltageRatio),
                                  y=0.0,
                                  z=0.0),
                    torque=Vector3(x=0.0, y=0.0, z=0.0))))



def main(args = None):
    rclpy.init(args=args)

    pbnode = PhidgetBridgeNode(topic="load_cell_force",
                               frame_id="load_cell",
                               bridge_channel=0)

    rclpy.spin(pbnode)

    rclpy.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
