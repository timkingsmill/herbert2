from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState

from herbert2_robot.script.odrive_driver import ODriveDriver
from .sensor import Sensor 

# --------------------------------------------------------------

class JointSensors(Sensor):
    """
        Publish Joint State messages:
            Motor 0
            Motor 1
            Motor 2
    """
    def __init__(self, node: Node, driver: ODriveDriver) -> None:
        super().__init__(node)
        self._driver = driver
        self._joint_state_publisher = self._node_ref().create_publisher(
            JointState, '/joint_states', qos_profile = qos_profile_sensor_data)

    # ----------------------------------------------------------

    def update(self) -> None:
        msg = JointState()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self._node_ref().get_clock().now().to_msg()

        pos = self._driver.get_motor_positions()

        msg.name.append('motor_a_joint')
        msg.position.append(pos[0])

        msg.name.append('motor_b_joint')
        msg.position.append(pos[1])

        msg.name.append('motor_c_joint')
        msg.position.append(pos[2])


        self._publish(msg)

    # ----------------------------------------------------------

    def _publish(self, msg: JointState) -> None:
        self._joint_state_publisher.publish(msg) 

# --------------------------------------------------------------
# --------------------------------------------------------------
