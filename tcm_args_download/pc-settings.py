#! /usr/bin/env python3
# coding=utf-8

"""
 description:   give an example to users how to set arguments to TCM
 author:		kevin.wang
 create date:	2025-02-24
 version:		1.0.0
"""

import serial
from serial.tools import list_ports

import struct
import time
import threading

import logging

from zlib import crc32

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class InstrumentStatus:
    def __init__(self):
        self.instrumentIndex = 0
        self.MAXinstrument = 0
        self.percent = 0
        self.reply = ''
        self.retry = 0
        self.MAXretry = 5
        self.value = 0
        # INIT:  
        # PROCESS:
        # OK
        # FAIL
        # FINISH
        # CONTINUE 
        self.status = 'FINISH'


class TeensyController:
    def __init__(self, instrumentstatus, port, baud_rate=115200):
        ports = [p.device for p in list_ports.comports() if port == p.device]
        if not ports:
            raise ValueError(f"No device found with serial number: {serial_number}")

        self.packet_serial = serial.Serial(ports[0], baudrate=baud_rate, timeout=1)

        self.instrumentstatus = instrumentstatus

        self.lock = threading.RLock()
        self.query_interval = 1.0  # Query interval in seconds
        self.running = False
        self.query_thread = None
        self.thread_read_received_packet = None

        # type A: means reply=1 is OK
        # type V: means reply is value
        # type S: means reply=8 is value save OK
        # type R: just display reply from TMC
        self.instruments = [
                #    uncomment this for query PID arguments 
                #    ['TC1:TCTD?@1', 'R'],
                #    ['TC1:TCTI?@1', 'R'],
                #    ['TC1:TCP?@1', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@1', 'R'],

                #    ['TC1:TCPIDP?@2', 'R'],
                #    ['TC1:TCPIDTI?@2', 'R'],
                #    ['TC1:TCPIDTD?@2', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@2', 'R'],

                #    ['TC1:TCTD?@3', 'R'],
                #    ['TC1:TCTI?@3', 'R'],
                #    ['TC1:TCP?@3', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@3', 'R'],

                #    ['TC1:TCTD?@4', 'R'],
                #    ['TC1:TCTI?@4', 'R'],
                #    ['TC1:TCP?@4', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@4', 'R'],

                #    you could compare PID arugments before 
                #    ['TC1:TCTD?@1', 'R'],
                #    ['TC1:TCTI?@1', 'R'],
                #    ['TC1:TCP?@1', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@1', 'R'],

                #    ['TC1:TCPIDP?@2', 'R'],
                #    ['TC1:TCPIDTI?@2', 'R'],
                #    ['TC1:TCPIDTD?@2', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@2', 'R'],

                #    ['TC1:TCTD?@3', 'R'],
                #    ['TC1:TCTI?@3', 'R'],
                #    ['TC1:TCP?@3', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@3', 'R'],

                #    ['TC1:TCTD?@4', 'R'],
                #    ['TC1:TCTI?@4', 'R'],
                #    ['TC1:TCP?@4', 'R'],
                #    ['TC1:TCCTRLINTERVAL?@4', 'R'],
                ]

        self.instrumentstatus.MAXinstrument = len(self.instruments)

    def set_instruments_sets(self, instruments_list):
        self.instruments = instruments_list 
        self.instrumentstatus.MAXinstrument = len(self.instruments)
        self.instrumentstatus.status = 'INIT'

    def get_instruments_return_value(self):
        return self.instrumentstatus.value

    def transparent_command(self, command):
        with self.lock:
            title_packet = b'C'
            bytes_command = command.encode('utf-8')
            packet = title_packet + bytes_command
            crc = crc32(packet)
            self.packet_serial.write(packet + struct.pack('<I', crc))
            self.packet_serial.write(b'\x0A\x0D')

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

    def analyze_TCM_reply(self, reply):
        # save reply
        self.instrumentstatus.reply = reply

        if self.instruments[self.instrumentstatus.instrumentIndex][1] == 'A':
            if reply[4:11] == 'REPLY=1':
                self.instrumentstatus.status = 'OK'
            else:
                self.instrumentstatus.status = 'FAIL'
        elif self.instruments[self.instrumentstatus.instrumentIndex][1] == 'P': 
            pos_1 = reply.find('=', 0)
            pos_2 = reply.find('@', 0)
            if pos_1 != -1 and pos_2 != -1:
                if reply[pos_1 + 1:pos_2] == '100':
                    self.instrumentstatus.percent = reply[pos_1 + 1:pos_2]
                    self.instrumentstatus.status = 'OK'
                else:
                    self.instrumentstatus.percent = reply[pos_1 + 1:pos_2]
                    self.instrumentstatus.status = 'CONTINUE'
            else:
                self.instrumentstatus.status = 'FAIL'
        elif self.instruments[self.instrumentstatus.instrumentIndex][1] == 'V': 
            pos_1 = reply.find('=', 0)
            pos_2 = reply.find('@', 0)
            if pos_1 != -1 and pos_2 != -1:
                self.instrumentstatus.value = reply[pos_1 + 1:pos_2]
                self.instrumentstatus.status = 'OK'
            else:
                self.instrumentstatus.status = 'FAIL'
        elif self.instruments[self.instrumentstatus.instrumentIndex][1] == 'S': 
            if reply[4:11] == 'REPLY=8':
                self.instrumentstatus.status = 'OK'
            else:
                self.instrumentstatus.status = 'FAIL'
        elif self.instruments[self.instrumentstatus.instrumentIndex][1] == 'R': 
            print(reply)
            self.instrumentstatus.status = 'OK'
        else:
            self.instrumentstatus.status = 'FAIL'

    def on_packet_received(self, packet):
        if len(packet) < 4:
            return

        received_crc = struct.unpack('<I', packet[-4:])[0]
        calculated_crc = crc32(packet[:-4])

        if received_crc != calculated_crc:
            print("CRC mismatch")
            return

        if packet[0] == ord('S'):  # Status packet
            laser_status = packet[1:6]
            temp_data = packet[6:-4]
            
            #print("Laser TTL Status:", [bool(x) for x in laser_status])
            
            for i in range(6):
                state = temp_data[i*7]
                temp = struct.unpack('>h', temp_data[i*7 + 1:i*7 + 3])[0] / 100.0
                tec_voltage = struct.unpack('>h', temp_data[i*7 + 3:i*7 + 5])[0] / 100.0
                tec_current = struct.unpack('>h', temp_data[i*7 + 5:i*7 + 7])[0] / 100.0
                
                state_str = ["IDLE", "WARM_UP", "ACTIVE", "ERROR"][state]
                #print(f"Channel {i}: State: {state_str}, Temp: {temp:.2f}°C, TEC Voltage: {tec_voltage:.2f}, TEC Current: {tec_current:.2f}")
        
        elif packet[0] == ord('A'):  # Acknowledgment packet
            print("Parameters set successfully")
        
        elif packet[0] == ord('N'):  # NAK packet
            print("Command not acknowledged")

        elif packet[0] == ord('T'):  # Transparent Command 
            reply_cmd = packet[1:-4].decode()
            self.analyze_TCM_reply(reply_cmd)

        elif packet[0] == ord('C'):  # Acknowledgment packet
            reply_cmd = packet[1:-4].decode()
            #print("Receive Acknowledged")

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
            while self.instrumentstatus.status != 'FINISH':
                self.transparent_command(
                        self.instruments[self.instrumentstatus.instrumentIndex][0])
                self.instrumentstatus.status = 'PROCESS'
                print('NO. ' + str(self.instrumentstatus.instrumentIndex + 1) 
                      + ' Retry: ' + str(self.instrumentstatus.retry + 1) + ' Instrument: ' + self.instruments[self.instrumentstatus.instrumentIndex][0])

                timeout = 10
                while self.instrumentstatus.status == 'PROCESS' and timeout != 0: 
                    time.sleep(0.5)
                    timeout = timeout - 1

                if self.instrumentstatus.status == 'OK':
                    self.instrumentstatus.instrumentIndex = self.instrumentstatus.instrumentIndex + 1
                    self.instrumentstatus.retry = 0
                    if self.instrumentstatus.instrumentIndex == self.instrumentstatus.MAXinstrument:
                        self.instrumentstatus.status = 'FINISH'
                        print('Tuning PID Arguments Successfully')
                elif self.instrumentstatus.status == 'FAIL':
                    if self.instrumentstatus.retry < self.instrumentstatus.MAXretry: 
                        self.instrumentstatus.retry += 1
                    else:
                        self.instrumentstatus.status = 'FINISH'
                        print('Tuning PID Arguments Fail: ' + self.instruments[self.instrumentstatus.instrumentIndex][0])
                        print('Reply: ' + self.instrumentstatus.reply)
                elif self.instrumentstatus.status == 'PROCESS':
                    self.instrumentstatus.status = 'FINISH'
                    print('Tuning PID Arguments Timeout')
                elif self.instrumentstatus.status == 'CONTINUE':
                    print('PID arguments tuning: %' + self.instrumentstatus.percent)
                else:
                    print('Tuning PID Arguments Unknown Error')

                time.sleep(1)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.stop()

# set commands sets into TCM
def set_command_sets(instruments):
    instrumentstatus = InstrumentStatus()
    controller = TeensyController(instrumentstatus, "/dev/ttyACM0")  # Adjust port as needed
    controller.set_instruments_sets(instruments)
    controller.run()

# get command return value from TCM
def get_command_return_value(instruments):
    instrumentstatus = InstrumentStatus()
    controller = TeensyController(instrumentstatus, "/dev/ttyACM0")  # Adjust port as needed
    controller.set_instruments_sets(instruments)
    controller.run()
    return controller.get_instruments_return_value()

# address: TCM address
def x207_PID_tunning_instruments_attach_address(address):
    PIDTuningInstruments = [
        [f'TC1:TCPIDCAL=4@{address}', 'A'],
        [f'TC1:TCSW=1@{address}', 'A'],
        [f'TC1:TCTUNESTATUS?@{address}', 'P'],
        [f'TC1:TCTD!@{address}', 'S'],
        [f'TC1:TCTI!@{address}', 'S'],
        [f'TC1:TCP!@{address}', 'S'],
        [f'TC1:TCCTRLINTERVAL!@{address}', 'S'],
    ]
    return PIDTuningInstruments

# address: TCM address : 6
def E1041_PID_tunning_instruments_attach_address(address):
    PIDTuningInstruments = [
        [f'TC1:TCPIDCAL=4@{address}', 'A'],
        [f'TC1:TCSW=1@{address}', 'A'],
        [f'TC1:TCTUNESTATUS?@{address}', 'P'],
        [f'TC1:TCPIDP!@{address}', 'S'],
        [f'TC1:TCPIDTI!@{address}', 'S'],
        [f'TC1:TCPIDTD!@{address}', 'S'],
        [f'TC1:TCCTRLINTERVAL!@{address}', 'S'],
    ]
    return PIDTuningInstruments

# voltage, current, temperature, max_temperature are arguments used by TCM
# address: TCM address
def set_TCM_arguments_attach_address(voltage, current, temperature, max_temperature, address):
    argumentsInstruments = [
        [f'TC1:TCOVPADJTHR={voltage}@{address}', 'R'],
        [f'TC1:TCOVPADJTHR!@{address}', 'S'],
        [f'TC1:TCOCPADJTHR={current}@{address}', 'R'],
        [f'TC1:TCOCPADJTHR!@{address}', 'S'],
        [f'TC1:TCSETTEMP={temperature}@{address}', 'R'],
        [f'TC1:TCSETTEMP!@{address}', 'S'],
        [f'TC1:TCMAXTEMP={max_temperature}@{address}', 'R'],
        [f'TC1:TCMAXTEMP!@{address}', 'S'],
    ]
    return argumentsInstruments

def set_example_TCM_arguments_attach_address(address):
    argumentsInstruments = [
        [f'TC1:TCOVPADJTHR=10.2@{address}', 'R'],
        [f'TC1:TCOVPADJTHR!@{address}', 'S'],

        [f'TC1:TCMAXV=10@{address}', 'R'],
        [f'TC1:TCMAXV!@{address}', 'S'],

        [f'TC1:TCOTPHT=45@{address}', 'R'],
        [f'TC1:TCOTPHT!@{address}', 'S'],

        [f'TC1:TCNTCBETA=3380@{address}', 'R'],
        [f'TC1:TCNTCBETA!@{address}', 'S'],

        [f'TC1:TCNTCR0=10@{address}', 'R'],
        [f'TC1:TCNTCR0!@{address}', 'S'],
    ]
    return argumentsInstruments

def enable_temperature_control_instruments_attach_address(address):
    operationInstruments = [
        [f'TC1:TCSW=1@{address}', 'A']
    ]
    return operationInstruments

def disable_temperature_control_instruments_attach_address(address):
    operationInstruments = [
        [f'TC1:TCSW=0@{address}', 'A']
    ]
    return operationInstruments

def get_instrument_return_value_attach_address(instrument, address):
    getinstruments = [
        [f'{instrument}@{address}', 'V']
    ]
    return getinstruments

def scripts_example():
    # TCM switch turn OFF
    # because some arguments setting need disable the TCM advancely
    set_command_sets(disable_temperature_control_instruments_attach_address(5))

    #setting new TCM arguments
    #real setting the arguments
    set_command_sets(set_example_TCM_arguments_attach_address(5))

    # print and check the value have been set
    commandsets = get_instrument_return_value_attach_address('TC1:TCOVPADJTHR?', 5)
    print(get_command_return_value(commandsets))

    commandsets = get_instrument_return_value_attach_address('TC1:TCMAXV?', 5)
    print(get_command_return_value(commandsets))

    commandsets = get_instrument_return_value_attach_address('TC1:TCOTPHT?', 5)
    print(get_command_return_value(commandsets))

    commandsets = get_instrument_return_value_attach_address('TC1:TCNTCBETA?', 5)
    print(get_command_return_value(commandsets))

    commandsets = get_instrument_return_value_attach_address('TC1:TCNTCR0?', 5)
    print(get_command_return_value(commandsets))

    # TURN PID
    set_command_sets(x207_PID_tunning_instruments_attach_address(5))


if __name__ == "__main__":
    #set_command_sets(set_TCM_arguments_attach_address(5.0, 1.0, 25.0, 35.0, 7))
    #set_command_sets(x207_PID_tunning_instruments_attach_address(7))
    #set_command_sets(enable_temperature_control_instruments_attach_address(7))
    #set_command_sets(disable_temperature_control_instruments_attach_address(7))
    #print(get_instrument_return_value_attach_address('TC1:TCACTTEMP?', 7))

    #TCM switch turn OFF
    #set_command_sets(disable_temperature_control_instruments_attach_address(6))

    #setting new TCM arguments
    #set_command_sets(set_example_TCM_arguments_attach_address(6))

    #TCM switch turn ON
    #set_command_sets(enable_temperature_control_instruments_attach_address(6))

    # TURN PID
    #set_command_sets(E1041_PID_tunning_instruments_attach_address(6))

    scripts_example()
