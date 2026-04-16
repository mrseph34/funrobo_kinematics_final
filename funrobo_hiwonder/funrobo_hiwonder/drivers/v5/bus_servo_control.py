#!/usr/bin/env python3
import sys
import time

from funrobo_hiwonder.drivers.v5.ros_robot_controller_sdk import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class BusServoControl:
    def __init__(self, board, time_out=1000):
        self.board = board
        self.time_out = time_out

    def setBusServoID(self, oldid, newid):
        """
        配置舵机id号, 出厂默认为1(configure servo id, factory default is set to 1)
        :param oldid: 原来的id， 出厂默认为1(original id, factory default is set to 1)
        :param newid: 新的id(new id)
        """
        self.board.bus_servo_set_id(oldid, newid)

    def getBusServoID(self, servo_id=None):
        """
        读取串口舵机id(read interface servo id)
        :param id: 默认为空(default is none)
        :return: 返回舵机id(return servo id)
        """
        count = 0 
        while True:
            if servo_id is None:
                res = self.board.bus_servo_read_id()
            else:
                res = self.board.bus_servo_read_id(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def setBusServoPulse(self, servo_id, position, duration):
        """
        驱动串口舵机转到指定位置(drive interface servo rotate to the designated position)
        :param servo_id: 要驱动的舵机id(servo id to be driven)
        :position: 位置(position)
        :duration: 转动需要的时间(time required for rotation)
        """
        position = 0 if position < 0 else position
        position = 1000 if position > 1000 else position
        duration = 0 if duration < 0 else duration
        duration = 30000 if duration > 30000 else duration
        
        print(f'duration, servo_id, position: {[duration/1000, servo_id, position]}')
        self.board.bus_servo_set_position(duration/1000, [[servo_id, position]])

    def getBusServoPulse(self, servo_id):
        '''
        读取舵机当前位置(read servo current position)
        :param servo_id:
        :return:
        '''
        count = 0 
        while True:
            res = self.board.bus_servo_read_position(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def stopBusServo(self, servo_id):
        '''
        停止舵机运行(stop running servo)
        :param servo_id:
        :return:
        '''
        self.board.bus_servo_stop(servo_id)

    def setBusServoDeviation(self, servo_id, offset=0):
        """
        调整偏差(adjust deviation)
        :param servo_id: 舵机id(servo id)
        :param offset:  偏差(deviation)
        """
        self.board.bus_servo_set_offset(servo_id, offset)

    def saveBusServoDeviation(self, servo_id):
        """
        保存偏差(save deviation)
        :param servo_id: 舵机id(servo id)
        """
        self.board.bus_servo_save_offset(servo_id)

    def getBusServoDeviation(self, servo_id):
        '''
        读取偏差值(read deviation value)
        :param servo_id: 舵机号(servo number)
        :return:
        '''
        # 发送读取偏差指令(send read deviation command)
        count = 0
        while True:
            res = self.board.bus_servo_read_offset(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def setBusServoAngleLimit(self, servo_id, low, high):
        '''
        设置舵机转动范围(set servo rotation range)
        :param servo_id:
        :param low:
        :param high:
        :return:
        '''
        self.board.bus_servo_set_angle_limit(servo_id, [low, high])

    def getBusServoAngleLimit(self, servo_id):
        '''
        读取舵机转动范围(read servo rotation range)
        :param servo_id:
        :return: 返回元祖 0： 低位  1： 高位(return tuple 0: low bit  1:high bit)
        '''
        count = 0
        while True:
            res = self.board.bus_servo_read_angle_limit(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def setBusServoVinLimit(self, servo_id, low, high):
        '''
        设置舵机电压范围(set servo voltage range)
        :param servo_id:
        :param low:
        :param high:
        :return:
        '''
        self.board.bus_servo_set_vin_limit(servo_id, [low, high])

    def getBusServoVinLimit(self, servo_id):
        '''
        读取舵机转动范围(read servo rotation range)
        :param servo_id:
        :return: 返回元祖 0： 低位  1： 高位(return tuple 0: low bit  1:high bit)
        '''
        count = 0
        while True:
            res = self.board.bus_servo_read_vin_limit(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def setBusServoMaxTemp(self, servo_id, m_temp):
        '''
        设置舵机最高温度报警(Set maximum temperature alarm for the servo)
        :param servo_id:
        :param m_temp:
        :return:
        '''
        self.board.bus_servo_set_temp_limit(servo_id, m_temp)

    def getBusServoTempLimit(self, servo_id):
        '''
        读取舵机温度报警范围(read servo temperature alarm range)
        :param servo_id:
        :return:
        '''
        count = 0
        while True:
            res = self.board.bus_servo_read_temp_limit(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def getBusServoTemp(self, servo_id):
        '''
        读取舵机温度(read servo temperature)
        :param servo_id:
        :return:
        '''
        count = 0
        while True:
            res = self.board.bus_servo_read_temp(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def getBusServoVin(self, servo_id):
        '''
        读取舵机电压(read servo voltage)
        :param servo_id:
        :return:
        '''
        count = 0
        while True:
            res = self.board.bus_servo_read_vin(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)

    def restBusServoPulse(self, oldid):
        # 舵机清零偏差和P值中位（500）(zeroing the deviation and setting the median P value for the servo (500))
        setBusServoDeviation(oldid, 0)    # 清零偏差(zeroing the deviation)
        time.sleep(0.1)
        setBusServoPulse(oldid, 500, 100)

##掉电(loss power)
    def unloadBusServo(self, servo_id):
        self.board.bus_servo_enable_torque(servo_id, 1)

##读取是否掉电(read whether there is power loss)
    def getBusServoLoadStatus(self, servo_id):
        count = 0
        while True:
            res = self.board.bus_servo_read_torque_state(servo_id)
            count += 1
            if res is not None:
                return res
            if count > self.time_out:
                return None
            time.sleep(0.01)
