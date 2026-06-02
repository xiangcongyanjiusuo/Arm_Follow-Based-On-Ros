#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import time
import sys

class ServoController:
    def __init__(self, port="COM6", baudrate=1000000, timeout=0.1):
        self.port_name = port
        self.serial_port = None
        self.ADDR_TORQUE_ENABLE = 40
        self.INST_WRITE = 3
        
        try:
            self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
            print(f"Successfully opened port {port}.")
        except serial.SerialException as e:
            print(f"Fatal: Could not open port {port}: {e}")
            sys.exit(1)

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print(f"Port {self.port_name} closed.")

    def _calculate_checksum(self, data):
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, instruction, parameters=None):
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

    def _write_register(self, servo_id, address, value, size=2):
        params = [address]
        if size == 1:
            params.append(value & 0xFF)
        elif size == 2:
            params.extend([value & 0xFF, (value >> 8) & 0xFF])
        else:
            return False
        return self._send_packet(servo_id, self.INST_WRITE, params)

    def disable_torque(self, servo_id):
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 0, size=1)

def main():
    STUDENT_PORT = "COM6"
    BAUDRATE = 1000000
    SERVO_IDS = list(range(1, 7))
    
    controller = None
    try:
        controller = ServoController(port=STUDENT_PORT, baudrate=BAUDRATE)
        
        print("Disabling torque for all servos...")
        for servo_id in SERVO_IDS:
            if controller.disable_torque(servo_id):
                print(f"Servo {servo_id}: Torque disabled (can now be moved manually)")
            else:
                print(f"Servo {servo_id}: Failed to disable torque")
            time.sleep(0.05)
        
        print("\nAll servos can now be moved manually!")
        
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if controller:
            controller.close()

if __name__ == '__main__':
    main()
