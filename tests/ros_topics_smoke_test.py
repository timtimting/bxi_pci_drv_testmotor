import os, signal, subprocess, time, tempfile
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from bxi_motor_msgs.msg import MotorCommand, MotorStateArray
from std_msgs.msg import String
from communication.msg import ActuatorCmds, ActuatorStates
log = tempfile.NamedTemporaryFile(mode='w', prefix='bxi-ros-smoke-', suffix='.log')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/bxi-ros-test-logs')
binary = str(Path(get_package_prefix('bxi_motor_control')) / 'lib/bxi_motor_control/motor_node')
config = str(Path(__file__).resolve().parents[1] / 'config/motor_console.yaml')
process = subprocess.Popen([binary, '--ros-args', '-p', 'auto_scan:=false', '-p', 'config_file:=' + config], stdout=log, stderr=log)
node = None
try:
    rclpy.init()
    node = rclpy.create_node('bxi_ros_smoke_test')
    feedback, status, named_feedback = [], [], []
    subscriptions = [node.create_subscription(MotorStateArray, '/bxi_motor_control/states', feedback.append, qos_profile_sensor_data), node.create_subscription(String, '/bxi_motor_control/command_status', status.append, 10)]
    subscriptions.append(node.create_subscription(ActuatorStates, '/hardware/actuator_states', named_feedback.append, qos_profile_sensor_data))
    named_publisher = node.create_publisher(ActuatorCmds, '/hardware/actuators_cmds', qos_profile_sensor_data)
    publisher = node.create_publisher(MotorCommand, '/bxi_motor_control/command', 1)
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and (publisher.get_subscription_count() == 0 or not feedback or named_publisher.get_subscription_count() == 0 or not named_feedback):
        if process.poll() is not None: raise RuntimeError('node exited: ' + Path(log.name).read_text())
        rclpy.spin_once(node, timeout_sec=0.1)
    assert publisher.get_subscription_count() and feedback, 'missing command subscription or state publication'
    message = MotorCommand(); message.action = 255; publisher.publish(message)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not status: rclpy.spin_once(node, timeout_sec=0.1)
    assert status and 'rejected' in status[-1].data
    assert feedback[-1].motors == []
    assert named_feedback[-1].name == []
    status.clear()
    frame = ActuatorCmds(); frame.actuators_name = ['waist_y_joint', 'l_shoulder_y_joint']
    frame.pos = [0.0, 0.0]; frame.vel = [0.0, 0.0]; frame.torque = [0.0, 0.0]; frame.kp = [0.0, 0.0]; frame.kd = [0.0, 0.0]
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not status:
        named_publisher.publish(frame)
        rclpy.spin_once(node, timeout_sec=0.1)
    assert status and 'actuators_cmds rejected' in status[-1].data  # No active PDO in hardware-free test.
    print('PASS: ROS typed and ELF3 named-array topics, best-effort compatibility and safe rejection without PDO')
finally:
    if node: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    if process.poll() is None: process.send_signal(signal.SIGINT)
    process.wait(timeout=10)
assert process.returncode == 0, Path(log.name).read_text()

log.close()
