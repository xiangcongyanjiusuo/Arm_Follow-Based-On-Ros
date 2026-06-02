#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from arm_msg.msg import ArmJointAngles
import serial
import time
import threading


class ServoController:
    def __init__(self, port="/dev/ttyUSB1", baudrate=1000000, timeout=0.1):
        self.port_name = port
        self.serial_port = None
        self.lock = threading.Lock()
        
        self.ADDR_TORQUE_ENABLE = 40
        self.ADDR_GOAL_POSITION = 42
        self.ADDR_PRESENT_POSITION = 56
        self.INST_WRITE = 3
        self.INST_READ = 2
        self.COMM_SUCCESS = 0
        self.COMM_RX_TIMEOUT = -6
        self.COMM_RX_CORRUPT = -7
        
        try:
            self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
        except serial.SerialException as e:
            raise Exception(f"Could not open port {port}: {e}")

    def close(self):
        with self.lock:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

    def _calculate_checksum(self, data):
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, instruction, parameters=None):
        with self.lock:
            if not self.serial_port or not self.serial_port.is_open:
                return False
            if parameters is None:
                parameters = []
            length = len(parameters) + 2
            packet_core = [servo_id, length, instruction] + parameters
            checksum = self._calculate_checksum(packet_core)
            packet = bytes([0xFF, 0xFF] + packet_core + [checksum])
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.write(packet)
                self.serial_port.flush()
                return True
            except Exception:
                return False

    def _read_packet(self):
        with self.lock:
            start_time = time.time()
            packet = []
            while (time.time() - start_time) < self.serial_port.timeout:
                if self.serial_port.in_waiting > 0:
                    byte = self.serial_port.read(1)
                    if not byte:
                        continue
                    byte = byte[0]
                    
                    if not packet and byte != 0xFF:
                        continue
                    
                    packet.append(byte)
                    
                    if len(packet) >= 2 and packet[-2:] == [0xFF, 0xFF]:
                        if len(packet) > 2:
                            packet = [0xFF, 0xFF]
                        continue
                    
                    if len(packet) > 4:
                        pkt_len = packet[3]
                        if len(packet) == pkt_len + 4:
                            core_data = packet[2:-1]
                            calculated_checksum = self._calculate_checksum(core_data)
                            if calculated_checksum == packet[-1]:
                                return self.COMM_SUCCESS, packet[4], packet[5:-1]
                            else:
                                return self.COMM_RX_CORRUPT, 0, []
            return self.COMM_RX_TIMEOUT, 0, []

    def _write_register(self, servo_id, address, value, size=2):
        params = [address]
        if size == 1:
            params.append(value & 0xFF)
        elif size == 2:
            params.extend([value & 0xFF, (value >> 8) & 0xFF])
        else:
            return False
        return self._send_packet(servo_id, self.INST_WRITE, params)

    def enable_torque(self, servo_id):
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 1, size=1)

    def disable_torque(self, servo_id):
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 0, size=1)

    def get_servo_angle(self, servo_id):
        if not self._send_packet(servo_id, self.INST_READ, [self.ADDR_PRESENT_POSITION, 2]):
            return None
        
        result, error, data = self._read_packet()
        
        if result != self.COMM_SUCCESS or error != 0:
            return None
        
        if data and len(data) >= 2:
            position = data[0] | (data[1] << 8)
            angle = ((position - 1024.0) / (3072.0 - 1024.0)) * 180.0 - 90.0
            angle = max(-90.0, min(90.0, angle))
            return angle
        
        return None

    def set_servo_angle(self, servo_id, angle):
        position = int(((angle + 90.0) / 180.0) * (3072.0 - 1024.0) + 1024.0)
        position = max(1024, min(3072, position))
        return self._write_register(servo_id, self.ADDR_GOAL_POSITION, position, size=2)


class ArmJointNode(Node):
    def __init__(self):
        super().__init__('arm_joint_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('read_frequency', 50.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.servo_ids = self.get_parameter('servo_ids').value
        self.read_frequency = self.get_parameter('read_frequency').value

        self.torque_enabled = {servo_id: False for servo_id in self.servo_ids}
        self.current_angles = {servo_id: 0.0 for servo_id in self.servo_ids}
        
        self.callback_group = ReentrantCallbackGroup()
        
        try:
            self.controller = ServoController(port=self.port, baudrate=self.baudrate)
            self.get_logger().info(f'Successfully connected to {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize servo controller: {e}')
            return
        
        self._enable_all_torque_at_startup()
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            'arm_joint_states',
            10
        )
        
        self.arm_joint_angles_pub = self.create_publisher(
            ArmJointAngles,
            'arm_joint_angles',
            10
        )
        
        self.joint_command_sub = self.create_subscription(
            ArmJointAngles,
            'cmd_angles',
            self.joint_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.enable_torque_srv = self.create_service(
            SetBool,
            'enable_torque',
            self.enable_torque_callback,
            callback_group=self.callback_group
        )
        
        self.timer = self.create_timer(
            1.0 / self.read_frequency,
            self.timer_callback
        )
        
        self.get_logger().info('Arm Joint Node has been initialized.')
        self.get_logger().info(f'Controlling {len(self.servo_ids)} servos: {self.servo_ids}')

    def _enable_all_torque_at_startup(self):
        self.get_logger().info('Reading angles and enabling torque for all servos...')
        for servo_id in self.servo_ids:
            angle = self.controller.get_servo_angle(servo_id)
            if angle is not None:
                self.current_angles[servo_id] = angle
                self.get_logger().info(f'Servo {servo_id}: Current angle = {angle:.2f}°')
            time.sleep(0.05)
        self._enable_all()
        self.get_logger().info('Startup enable complete. Torque ON — arm is active.')

    # ── 批量 使能 / 失能 ──────────────────────────────────

    def _enable_all(self) -> bool:
        """使能全部舵机, 返回是否全部成功."""
        all_ok = True
        for servo_id in self.servo_ids:
            if not self.torque_enabled[servo_id]:
                if self.controller.enable_torque(servo_id):
                    self.torque_enabled[servo_id] = True
                else:
                    self.get_logger().error(f'Servo {servo_id}: Failed to enable torque')
                    all_ok = False
        return all_ok

    def _disable_all(self):
        """失能全部舵机."""
        for servo_id in self.servo_ids:
            if self.controller.disable_torque(servo_id):
                self.torque_enabled[servo_id] = False
            else:
                self.get_logger().warn(f'Servo {servo_id}: Failed to disable torque')

    def joint_command_callback(self, msg):
        if len(msg.angles) != len(self.servo_ids):
            self.get_logger().error(
                f'Received {len(msg.angles)} angles but have {len(self.servo_ids)} servos'
            )
            return

        # ── 写入目标角度 (扭矩启动时已使能, 无需重复) ──────
        for i, servo_id in enumerate(self.servo_ids):
            target_angle = msg.angles[i]
            if not self.controller.set_servo_angle(servo_id, target_angle):
                self.get_logger().warn(
                    f'Servo {servo_id}: Failed to set angle {target_angle:.2f}°')
            time.sleep(0.01)

    def enable_torque_callback(self, request, response):
        """SetBool 服务: True = 使能全部舵机, False = 失能全部舵机."""
        enable = request.data

        if enable:
            success = self._enable_all()
            response.success = success
            response.message = (
                'All servos: torque enabled' if success
                else 'Some servos failed to enable torque')
        else:
            self._disable_all()
            response.success = True
            response.message = 'All servos: torque disabled'

        self.get_logger().info(response.message)
        return response

    def timer_callback(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        arm_joint_angles_msg = ArmJointAngles()
        
        for i, servo_id in enumerate(self.servo_ids):
            angle = self.controller.get_servo_angle(servo_id)
            if angle is not None:
                self.current_angles[servo_id] = angle
                joint_state_msg.name.append(f'joint_{servo_id}')
                joint_state_msg.position.append(angle)
                if i < 6:
                    arm_joint_angles_msg.angles[i] = angle
            else:
                joint_state_msg.name.append(f'joint_{servo_id}')
                joint_state_msg.position.append(self.current_angles.get(servo_id, 0.0))
                if i < 6:
                    arm_joint_angles_msg.angles[i] = self.current_angles.get(servo_id, 0.0)
        
        self.joint_state_pub.publish(joint_state_msg)
        self.arm_joint_angles_pub.publish(arm_joint_angles_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down arm joint node...')
        self._disable_all()
        if hasattr(self, 'controller'):
            self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    arm_joint_node = None
    try:
        arm_joint_node = ArmJointNode()
        rclpy.spin(arm_joint_node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down...")
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if arm_joint_node:
            arm_joint_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
