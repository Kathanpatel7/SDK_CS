#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 21 16:55:51 2024

@author: kathanpatel
"""

import socket
import struct
import math
import time

DEFAULT_TIMEOUT = 10

conname = ['total_message_len','total_message_type','mode_sub_len','mode_sub_type','timestamp','reserver','reserver',
'is_robot_power_on','is_emergency_stopped','is_robot_protective_stopped','is_program_running','is_program_paused',
'get_robot_mode','get_robot_control_mode','get_target_speed_fraction','get_speed_scaling','get_target_speed_fraction_limit',
'get_robot_speed_mode','is_robot_system_in_alarm','is_in_package_mode','reverse','joint_sub_len','joint_sub_type',
'actual_joint0','target_joint0','actual_velocity0','target_pluse0','actual_pluse0','zero_pluse0','current0',
'voltage0','temperature0','torques0','mode0','reverse0','actual_joint1','target_joint1','actual_velocity1',
'target_pluse1','actual_pluse1','zero_pluse1','current1','voltage1','temperature1','torques1','mode1',
'reverse1','actual_joint2','target_joint2','actual_velocity2','target_pluse2','actual_pluse2','zero_pluse2',
'current2','voltage2','temperature2','torques2','mode2','reverse2','actual_joint3','target_joint3','actual_velocity3',
'target_pluse3','actual_pluse3','zero_pluse3','current3','voltage3','temperature3','torques3','mode3',
'reverse3','actual_joint4','target_joint4','actual_velocity4','target_pluse4','actual_pluse4','zero_pluse4',
'current4','voltage4','temperature4','torques4','mode4','reverse4','actual_joint5','target_joint5','actual_velocity5',
'target_pluse5','actual_pluse5','zero_pluse5','current5','voltage5','temperature5','torques5','mode5',
'reverse5','cartesial_sub_len','cartesial_sub_type','tcp_x','tcp_y','tcp_z','rot_x','rot_y','rot_z','offset_px',
'offset_py','offset_pz','offset_rotx','offset_roty','offset_rotz','configuration_sub_len','configuration_sub_type',
'limit_min_joint_x0','limit_max_joint_x0','limit_min_joint_x1','limit_max_joint_x1','limit_min_joint_x2',
'limit_max_joint_x2','limit_min_joint_x3','limit_max_joint_x3','limit_min_joint_x4','limit_max_joint_x4',
'limit_min_joint_x5','limit_max_joint_x5','max_velocity_joint_x0','max_acc_joint_x0','max_velocity_joint_x1',
'max_acc_joint_x1','max_velocity_joint_x2','max_acc_joint_x2','max_velocity_joint_x3','max_acc_joint_x3',
'max_velocity_joint_x4','max_acc_joint_x4','max_velocity_joint_x5','max_acc_joint_x5','default_velocity_joint',
'default_acc_joint','default_tool_velocity','default_tool_acc','eq_radius','dh_a_joint_x0','dh_a_joint_x1',
'dh_a_joint_x2','dh_a_joint_x3','dh_a_joint_x4','dh_a_joint_x5','dh_d_joint_d0','dh_d_joint_d1','dh_d_joint_d2',
'dh_d_joint_d3','dh_d_joint_d4','dh_d_joint_d5','dh_alpha_joint_x0','dh_alpha_joint_x1','dh_alpha_joint_x2',
'dh_alpha_joint_x3','dh_alpha_joint_x4','dh_alpha_joint_x5','reserver0','reserver1','reserver2','reserver3',
'reserver4','reserver5','board_version','control_box_type','robot_type','robot_struct','masterboard_sub_len',
'masterboard_sub_type','digital_input_bits','digital_output_bits','standard_analog_input_domain0','standard_analog_input_domain1',
'tool_analog_input_domain','standard_analog_input_value0','standard_analog_input_value1','tool_analog_input_value',
'standard_analog_output_domain0','standard_analog_output_domain1','tool_analog_output_domain','standard_analog_output_value0',
'standard_analog_output_value1','tool_analog_output_value','bord_temperature','robot_voltage','robot_current',
'io_current','bord_safe_mode','is_robot_in_reduced_mode','get_operational_mode_selector_input','get_threeposition_enabling_device_input',
'masterboard_safety_mode','additional_sub_len','additional_sub_type','is_freedrive_button_pressed','reserve',
'is_freedrive_io_enabled','is_dynamic_collision_detect_enabled','reserver','tool_sub_len','tool_sub_type',
'tool_analog_output_domain','tool_analog_input_domain','tool_analog_output_value','tool_analog_input_value',
'tool_voltage','tool_output_voltage','tool_current','tool_temperature','tool_mode','safe_sub_len','safe_sub_type',
'safety_crc_num','safety_operational_mode','reserver','current_elbow_position_x','current_elbow_position_y',
'current_elbow_position_z','elbow_radius','tool_comm_sub_len','tool_comm_sub_type','is_enable','baudrate',
'parity','stopbits','tci_modbus_status','tci_usage','reserved0','reserved1',]
confmt = 'IBIBQ???????BBdddB??IIBdddiiiffffBidddiiiffffBidddiiiffffBidddiiiffffBidddiiiffffBidddiiiffffBiIBddddddddddddIBdddddddddddddddddddddddddddddddddddddddddddddddddddddIIIIIBIIBBBdddBBBdddffffB???BIB????BIBBBddfBffBIBIbBddddIB?III?Bff'

class RobotData():
    def connect(self, ip, port=30001):
        try:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.__sock.settimeout(DEFAULT_TIMEOUT)
            self.hostname = ip
            self.port = port
            self.__sock.connect((self.hostname, self.port))
            self.__buf = bytes()
        except (socket.timeout, socket.error):
            self.__sock = None
            raise

    def disconnect(self):
        if self.__sock:
            self.__sock.close()
            self.__sock = None

    def get_data(self):
        __buf = self.__sock.recv(4096)
        if len(__buf) <= 0:
            time.sleep(0.01)

        while(len(__buf) > 0):
            data_length = struct.unpack(">i", __buf[0:4])[0]  # First 4 bytes are the message length
            data_type = struct.unpack("B", __buf[4:5])[0]  # 5th byte is the message type
            data, __buf = __buf[0:data_length], __buf[data_length:]

            if data_type == 16:
                # Start parsing the data packet
                dic1 = {}
                for i in range(0, len(conname)):
                    fmtsize = struct.calcsize(confmt[i])
                    data1, data = data[0:fmtsize], data[fmtsize:]
                    fmt = ">" + confmt[i]
                    dic1[conname[i]] = struct.unpack(fmt, data1)[0]

                for key in dic1:
                    setattr(self, key, dic1[key])
                return self
        return None

    def send_data(self, mes):
        self.__sock.send(mes)

    def movel(self, position):
        # Validate position input
        if len(position) != 6:
            raise ValueError("Position must be a list of 6 elements: [x, y, z, rx, ry, rz]")
        
        # Create the movel command
        mes = f"def mov():\r\n    movel([{position[0]},{position[1]},{position[2]},{position[3]},{position[4]},{position[5]}],a=1.4,v=1.05,t=0,r=0)\r\nend\r\n"
        self.send_data(mes.encode())

if __name__ == "__main__":
    rb = RobotData()
    try:
        rb.connect('192.168.1.200')
        sample_position = [-0.41, -0.348, 0.318, -3.126, 0, 1.266]  # Example position in [x, y, z, rx, ry, rz]
        rb.movel(sample_position)
        time.sleep(0.1)
    except (socket.error, socket.timeout) as e:
        print(f"Socket error: {e}")
    finally:
        rb.disconnect()
