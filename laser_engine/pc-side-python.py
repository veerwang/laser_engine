#! /usr/bin/env python3
# coding=utf-8

"""
 description:
 author:		kevin.wang
 create date:	2024-07-17
 version:		1.0.0
"""

import serial
from serial.tools import list_ports

import struct
import time
import threading
import datetime
import logging

from zlib import crc32

# Configure logging
logging.basicConfig(filename='laser_engine.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class TeensyController:
    def __init__(self, port, baud_rate=115200):
        ports = [p.device for p in list_ports.comports() if port == p.device]
        if not ports:
            raise ValueError(f"No device found with serial number: {serial_number}")

        self.packet_serial = serial.Serial(ports[0], baudrate=baud_rate, timeout=1)

        self.lock = threading.RLock()
        self.query_interval = 1.0  # Query interval in seconds
        self.running = False
        self.query_thread = None
        self.thread_read_received_packet = None

        self.crc_missmatch = 0

    def query_status(self):
        with self.lock:
            packet = b'Q'
            crc = crc32(packet)
            self.packet_serial.write(packet + struct.pack('<I', crc))
            self.packet_serial.write(b'\x0A\x0D')

    def set_temperature_setpoints(self, setpoints):
        with self.lock:
            if len(setpoints) != 6:
                raise ValueError("Must provide 6 temperature setpoints")
            
            packet = b'S' + struct.pack('6f', *setpoints)
            crc = crc32(packet)
            try:
                self.packet_serial.write(packet + struct.pack('<I', crc))
                self.packet_serial.write(b'\x0A\x0D')
            except Exception as e:
                print(e)

    def reset_TCM_status(self):
        with self.lock:
            packet = b'R'
            crc = crc32(packet)
            self.packet_serial.write(packet + struct.pack('<I', crc))
            self.packet_serial.write(b'\x0A\x0D')

    def log_message(self, message):
        """
        Logs a message to the console and a file.

        :param message: The message to be logged.
        """
        logging.info(message)
        print(message)  # Optional: Print to console for real-time feedback

    def on_packet_received(self, packet):
        if len(packet) < 4:
            return

        received_crc = struct.unpack('<I', packet[-4:])[0]
        calculated_crc = crc32(packet[:-4])

        if received_crc != calculated_crc:
            self.crc_missmatch += 1
            print("CRC mismatch")
            return

        if packet[0] == ord('S'):  # Status packet
            laser_status = packet[1:6]
            temp_data = packet[6:-4]
            
            self.log_message('')
            self.log_message('New Round Query Data Start..')
            self.log_message("Laser TTL Status:" + str([bool(x) for x in laser_status]))
            
            for i in range(6):
                state = temp_data[i*7]
                temp = struct.unpack('>h', temp_data[i*7 + 1:i*7 + 3])[0] / 100.0
                tec_voltage = struct.unpack('>h', temp_data[i*7 + 3:i*7 + 5])[0] / 100.0
                tec_current = struct.unpack('>h', temp_data[i*7 + 5:i*7 + 7])[0] / 100.0
                
                state_str = ["WARMING_UP", "CHECK_ACTIVE", "ACTIVE", "WAKE_UP", "SLEEP", "PREPARE_SLEEP", "CHECK_ERROR", "ERROR"][state]
                self.log_message(f"Channel {i}: State: {state_str}, Temp: {temp:.2f}°C, TEC Voltage: {tec_voltage:.2f}, TEC Current: {tec_current:.2f}")

            for i in range(6):
                temp = struct.unpack('>h', temp_data[42 + i*2 + 0:42 + i*2 + 2])[0] / 100.0
                self.log_message(f"Channel {i}: DiffTemp: {temp:.2f}°C")

            for i in range(6):
                temp = struct.unpack('>h', temp_data[42 + 12 + i*2 + 0:42 + 12 + i*2 + 2])[0] / 100.0
                self.log_message(f"Channel {i}: Hi-Temp SetPoint: {temp:.2f}°C")

            self.log_message(f"CRC missmatch times: {self.crc_missmatch}")
        
        elif packet[0] == ord('A'):  # Acknowledgment packet
            print("Parameters set successfully")
        
        elif packet[0] == ord('N'):  # NAK packet
            print("Command not acknowledged")

    def query_loop(self):
        while self.running:
            self.query_status()
            time.sleep(self.query_interval)

    def received_loop(self):
        msg = []
        while self.running:
            #msg.append(ord(self.packet_serial.read()))
            if self.packet_serial.in_waiting == 0:
                continue

            char = self.packet_serial.read(1)
            if char == b'\r' and msg[-1] == 0x0A:
                self.on_packet_received(bytearray(msg[:-1]))
                msg = []
                continue
            msg += char

    def start(self):
        self.running = True
        self.query_thread = threading.Thread(target=self.query_loop)
        self.query_thread.start()

        self.thread_read_received_packet = threading.Thread(target=self.received_loop)
        self.thread_read_received_packet.start()

    def stop(self):
        self.running = False
        self.packet_serial.close()
        if self.query_thread:
            self.query_thread.join()
        if self.thread_read_received_packet:
            self.thread_read_received_packet.join()

    def run(self):
        try:
            self.start()
            while True:
                self.query_status()
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.stop()

def crc32_to_bytes(crc32_value):
    # Pack the CRC32 integer into bytes using little-endian format
    return struct.pack('<I', crc32_value)


if __name__ == "__main__":
    controller = TeensyController("/dev/ttyACM0")  # Adjust port as needed
    
    # Example usage in a separate thread
    def set_parameters_thread():
        time.sleep(2)  # Wait for 5 seconds before setting parameters
        # 402nm 470nm  638 735 550-1 550-2
        #new_setpoints = [25.0, 25.0, 25.0, 25.0, 25.0, 103.5]
        #controller.set_temperature_setpoints(new_setpoints)
        #controller.reset_TCM_status()

    parameter_thread = threading.Thread(target=set_parameters_thread)
    parameter_thread.start()

    controller.run()
