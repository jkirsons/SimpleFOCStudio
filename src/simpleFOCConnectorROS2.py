import logging
import threading
import time

import serial
from PyQt5 import QtCore, QtWidgets
#from serial import SerialException
from collections import defaultdict

from .simpleFOCConnector import SimpleFOCDevice

import can # https://python-can.readthedocs.io

import rclpy
from rclpy.node import Node

from simplefoc_interfaces.srv import SetFloat
from simplefoc_interfaces.srv import SetDouble
from simplefoc_can_pubsub.can_handler import CanHandler


class SimpleFOCDeviceROS2(SimpleFOCDevice):
    __instance = None

    TORQUE_CONTROL  = 0
    VELOCITY_CONTROL = 1
    ANGLE_CONTROL = 2
    VELOCITY_OPENLOOP_CONTROL = 3
    ANGLE_OPENLOOP_CONTROL = 4

    SINE_PWM  = 0
    SPACE_VECTOR_PWM = 1
    TRAPEZOIDAL_120 = 2
    TRAPEZOIDAL_150 = 3

    VOLTAGE_TORQUE = 0
    DC_CURRENT_TORQUE  = 1
    FOC_CURRENT_TORQUE = 2

    VELOCITY_PID = 'V'
    ANGLE_PID = 'A'
    CURRENT_Q_PID = 'Q'
    CURRENT_D_PID = 'D'

    PULL_CONFIG_ON_CONNECT = 'Pull config'
    PUSH_CONFG_ON_CONNECT = 'Push config'
    ONLY_CONNECT = 'Only connect'

    @staticmethod
    def getInstance():
        """ Static access method. """
        if SimpleFOCDeviceROS2.__instance == None:
            SimpleFOCDeviceROS2()
        SimpleFOCDevice._SimpleFOCDevice__instance = SimpleFOCDeviceROS2.__instance
        return SimpleFOCDeviceROS2.__instance

    def __init__(self):
        if SimpleFOCDeviceROS2.__instance != None:
            raise Exception("This class is a singleton!")
        else:
            super().__init__()
            self.commProvider = ROS2ReceiveHandler()
            self.commProvider.commandDataReceived.connect(self.parseResponses)
            self.commProvider.stateMonitorReceived.connect(self.parseStateResponses)
            self.stateUpdater = ROS2StateUpdateRunner(self)
            self.connectionID = "1"
            SimpleFOCDeviceROS2.__instance = self

    def __initCommunications(self):
        self.commProvider.start()

    def __closeCommunication(self):
        pass

    def connect(self, connectionMode):
        print("ROS2 Connect")
        try:
            self.__initCommunications()
        except Exception as serEx:
            logging.warning('Is not possible to open serial port')
            logging.warning('Port =' + self.serialPortName)

            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Warning)
            msgBox.setText('Error while connecting to the ROS2 server')
            msgBox.setWindowTitle('ROS2 SimpleFOC ConfigTool')
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec()
            return False
        else:
            self.isConnected = True
            for listener in self.connectionStateListenerList:
                listener.connectionStateChanged(True)
            if connectionMode == SimpleFOCDevice.PULL_CONFIG_ON_CONNECT:
                self.pullConfiguration()
                if self.stateUpdater.stopped():
                    self.stateUpdater = ROS2StateUpdateRunner(self)
                self.stateUpdater.start()
            elif connectionMode == SimpleFOCDevice.PUSH_CONFG_ON_CONNECT:
                self.pushConfiguration()
                if self.stateUpdater.stopped():
                    self.stateUpdater = ROS2StateUpdateRunner(self)
                self.stateUpdater.start()
                pass
            return True

    def disConnect(self):
        self.isConnected = False
        self.__closeCommunication()
        self.stateUpdater.stop()
        for listener in self.connectionStateListenerList:
            listener.connectionStateChanged(False)

    def addConnectionStateListener(self, listener):
        self.connectionStateListenerList.append(listener)

    def sendCommand(self, command):
        pass

    def setCommand(self, command, value):
        if self.isConnected:
            if command in CanHandler.commands_reverse:
                self.commProvider.ros2_client.send_request(CanHandler.commands_reverse[command], value, 1, self.devCommandID)
            else:
                print("Command not in lookup: ", command)

    def getCommand(self, command):
        if self.isConnected:
           self.sendCommand(str(self.devCommandID) + str(command) )

    def sendControlType(self, loop_control_type):
        if self.isConnected:
            if loop_control_type  != '':
                self.controlType = loop_control_type    
            self.setCommand('C', str(loop_control_type))
            
    def sendTorqueType(self, torque_type):
        if self.isConnected:
            if torque_type  != '':
                self.torqueType = torque_type    
            self.setCommand('T', str(torque_type))
            
    def sendMotionDownsample(self, value):
        if self.isConnected:
            if value  != '':
                self.motionDownsample = value    
            self.setCommand('CD', str(value))

    def sendProportionalGain(self, pid, value ):
        if self.isConnected:
            if value  != '':
                pid.P = value
            self.setCommand(str(pid.cmd)+'P',  str(value))

    def sendIntegralGain(self, pid, value):
        if self.isConnected:
            if value  != '':
                pid.I = value
            self.setCommand(str(pid.cmd)+'I', str(value))

    def sendDerivativeGain(self, pid,  value):
        if self.isConnected:
            if value  != '':
                pid.D = value
            self.setCommand(str(pid.cmd)+'D', str(value))

    def sendOutputRamp(self, pid, value):
        if self.isConnected:
            if value  != '':
                pid.outputRamp = value
            self.setCommand(str(pid.cmd)+'R', str(value))

    def sendOutputLimit(self, pid, value):
        if self.isConnected:
            if value  != '':
                pid.outputLimit = value
            self.setCommand(str(pid.cmd)+'L', str(value))

    def sendLowPassFilter(self, lpf, value):
        if self.isConnected:
            if value  != '':
                lpf.Tf = value
            self.setCommand(str(lpf.cmd)+'F', str(value))

    def sendVelocityLimit(self, value):
        if self.isConnected:
            if value  != '':
                self.velocityLimit = value
            self.setCommand('LV', str(value))

    def sendVoltageLimit(self, value):
        if self.isConnected:
            if value  != '':
                self.voltageLimit = value
            self.setCommand('LU', str(value))

    def sendCurrentLimit(self, value):
        if self.isConnected:
            if value  != '':
                self.currentLimit = value
            self.setCommand('LC', str(value))

    def sendPhaseResistance(self, value):
        if self.isConnected:
            if value  != '':
                self.phaseResistance = value
            self.setCommand('R', str(value))

    def sendTargetValue(self, targetvalue):
        if self.isConnected:
            if targetvalue  != '':
                self.target = targetvalue
            self.setCommand('',self.target)

    def sendSensorZeroOffset(self, targetvalue):
        if self.isConnected:
            if targetvalue  != '':
                self.sensorZeroOffset = targetvalue
            self.setCommand('SM', str(targetvalue))

    def sendSensorZeroElectrical(self, targetvalue):
        if self.isConnected:
            if targetvalue  != '':
                self.sensorElectricalZero = targetvalue
            self.setCommand('SE', str(targetvalue))

    def sendDeviceStatus(self, targetvalue):
        if self.isConnected:
            if targetvalue != '':
                self.deviceStatus = targetvalue
            self.setCommand('E', str(targetvalue))

            
    def sendModulationCentered(self, targetvalue):
        if self.isConnected:
            if targetvalue != '':
                self.modulationCentered = targetvalue
            self.setCommand('WC', str(targetvalue))
            
    def sendModulationType(self, targetvalue):
        if self.isConnected:
            if targetvalue != '':
                self.modulationType = targetvalue
            self.setCommand('WT', str(targetvalue))
            
    def sendDeviceStatus(self, targetvalue):
        if self.isConnected:
            if targetvalue != '':
                self.deviceStatus = targetvalue
            self.setCommand('E', str(targetvalue))

    def sendMonitorDownsample(self, targetvalue):
        if self.isConnected:
            if targetvalue != '':
                self.monitorDownsample = targetvalue
            self.setCommand('MD', str(targetvalue))

    def sendMonitorClearVariables(self):
        if self.isConnected:
            self.monitorVariables = 0
            self.getCommand('MC')

    def sendMonitorVariables(self, vararray):
        if self.isConnected:
            if vararray != '':
                val = 0
                m  = 10**6
                for var in vararray:
                    val = val+ int(var)*m
                    m = m/10
                self.monitorVariables = vararray
                self.setCommand('MS', "{:07d}".format(int(val)))
            else:
                self.getCommand('MS')


    def updateStates(self):
        if self.isConnected:
            self.getCommand('MG0')
            time.sleep(100 / 1000)
            self.getCommand('MG1')
            time.sleep(100 / 1000)
            self.getCommand('MG2')
            time.sleep(100 / 1000)
            self.getCommand('MG3')
            time.sleep(100 / 1000)
            self.getCommand('MG4')
            time.sleep(100 / 1000)
            self.getCommand('MG5')
            time.sleep(100 / 1000)
            self.getCommand('MG6')
            time.sleep(100 / 1000)


    def pushConfiguration(self):
        print("push")
        # self.sendControlType(self.controlType)
        # self.sendProportionalGain(self.PIDVelocity, self.self)
        # self.sendIntegralGain(self.PIDVelocity, self.integralGainPID)
        # self.sendDerivativeGain(self.PIDVelocity, self.derivativeGainPID)
        # self.sendOutputRamp(self.PIDVelocity, self.voltageRampPID)
        # self.sendLowPassFilter(self.LPFVelocity,self.lowPassFilter)
        # self.sendPGain(self.anglePGain)
        # self.sendVelocityLimit(self.velocityLimit)
        # self.sendVoltageLimit(self.voltageLimit)
        # self.sendTargetValue(self.initialTarget)

    def pullPIDConf(self, pid, lpf):
        self.sendProportionalGain(pid,'')
        time.sleep(5 / 1000)
        self.sendIntegralGain(pid,'')
        time.sleep(5 / 1000)
        self.sendDerivativeGain(pid,'')
        time.sleep(5 / 1000)
        self.sendOutputRamp(pid,'')
        time.sleep(5 / 1000)
        self.sendOutputLimit(pid,'')
        time.sleep(5 / 1000)
        self.sendLowPassFilter(lpf,'')

    def pullConfiguration(self):
        time.sleep(5 / 1000)
        self.sendControlType('')
        time.sleep(5 / 1000)
        self.sendTorqueType('')
        time.sleep(5 / 1000)
        self.pullPIDConf( self.PIDVelocity, self.LPFVelocity)
        time.sleep(5 / 1000)
        self.pullPIDConf( self.PIDAngle, self.LPFAngle)
        time.sleep(5 / 1000)
        self.pullPIDConf( self.PIDCurrentD, self.LPFCurrentD)
        time.sleep(5 / 1000)
        self.pullPIDConf( self.PIDCurrentQ, self.LPFCurrentQ)
        time.sleep(5 / 1000)
        self.sendVelocityLimit('')
        time.sleep(5 / 1000)
        self.sendVoltageLimit('')
        time.sleep(5 / 1000)
        self.sendCurrentLimit('')
        time.sleep(5 / 1000)
        self.sendSensorZeroElectrical('')
        time.sleep(5 / 1000)
        self.sendSensorZeroOffset('')
        time.sleep(5 / 1000)
        self.sendMotionDownsample('')
        time.sleep(5 / 1000)
        self.sendPhaseResistance('')
        time.sleep(5 / 1000)
        self.sendModulationCentered('')
        time.sleep(5 / 1000)
        self.sendModulationCentered('')
        time.sleep(5 / 1000)
        self.sendDeviceStatus('')

    def parsePIDFResponse(self, pid, lpf, comandResponse):
        if 'P' in comandResponse:
            pid.P = float(comandResponse.replace('P: ', ''))
        if 'I' in comandResponse:
            pid.I = float(comandResponse.replace('I: ', ''))
        if 'D' in comandResponse:
            pid.D = float(comandResponse.replace('D: ', ''))
        if 'ramp' in comandResponse:
            val = comandResponse.replace('ramp:', '')
            if 'ovf' in val:
                pid.outputRamp = 0
            else:
                pid.outputRamp = float(comandResponse.replace('ramp:', ''))
        if 'limit' in comandResponse:
            pid.outputLimit = float(comandResponse.replace('limit:', ''))
        if 'Tf' in comandResponse:
            lpf.Tf = float(comandResponse.replace('Tf: ', ''))

    def parseLimitsResponse(self, comandResponse):
        if 'vel:' in comandResponse:
            self.velocityLimit = float(comandResponse.replace('vel:', ''))
        elif 'volt:' in comandResponse:
            self.voltageLimit = float(comandResponse.replace('volt:', ''))
        elif 'curr:' in comandResponse:
            self.currentLimit = float(comandResponse.replace('curr:', ''))

    def parseMotionResponse(self, comandResponse):
        if 'downsample' in comandResponse:
            self.motionDownsample = float(comandResponse.replace('downsample:', ''))
        elif 'torque' in comandResponse:
            self.controlType = 0
        elif 'angle open' in comandResponse:
            self.controlType = 4
        elif 'angle' in comandResponse:
            self.controlType = 2
        elif 'vel open' in comandResponse:
            self.controlType = 3
        elif 'vel' in comandResponse:
            self.controlType = 1
            
    def parsePWMModResponse(self, comandResponse):
        if 'center' in comandResponse:
            self.modulationCentered = float(comandResponse.replace('center:', ''))
        elif 'type' in comandResponse:
            comandResponse = comandResponse.replace('type:', '')
            if 'Sine' in comandResponse:
                self.modulationType = self.SINE_PWM
            elif 'SVPWM' in comandResponse:
                self.modulationType = self.SPACE_VECTOR_PWM
            elif 'Trap 120' in comandResponse:
                self.modulationType = self.TRAPEZOIDAL_120
            elif 'Trap 150' in comandResponse:
                self.modulationType = self.TRAPEZOIDAL_150
            
    def parseTorqueResponse(self, comandResponse):
        if 'volt' in comandResponse:
            self.torqueType = 0
        elif 'dc curr' in comandResponse:
            self.torqueType = 1
        elif 'foc curr' in comandResponse:
            self.torqueType = 2

    def parseSensorResponse(self, comandResponse):
        if 'el. offset' in comandResponse:
            self.sensorElectricalZero = float(comandResponse.replace('el. offset:', ''))
        elif 'offset' in comandResponse:
            self.sensorZeroOffset = float(comandResponse.replace('offset:', ''))

    def parseMonitorResponse(self, comandResponse):
        if 'all' in comandResponse:
            varStr = comandResponse.replace('all:', '')
            states = varStr.rstrip().split('\t', 7)
            self.targetNow = states[0]
            self.voltageQNow = states[1]
            self.voltageDNow = states[2]
            self.currentQNow = states[3]
            self.currentDNow = states[4]
            self.velocityNow = states[5]
            self.angleNow = states[6]
        if 'target' in comandResponse:
            self.targetNow = float(comandResponse.replace('target:', ''))
        elif 'Vq' in comandResponse:
            self.voltageQNow = float(comandResponse.replace('Vq:', ''))
        elif 'Vd' in comandResponse:
            self.voltageDNow = float(comandResponse.replace('Vd:', ''))
        elif 'Cq' in comandResponse:
            self.currentQNow = float(comandResponse.replace('Cq:', ''))
        elif 'Cd' in comandResponse:
            self.currentDNow = float(comandResponse.replace('Cd:', ''))
        elif 'vel' in comandResponse:
            self.velocityNow = float(comandResponse.replace('vel:', ''))
        elif 'angle' in comandResponse:
            self.angleNow = float(comandResponse.replace('angle:', ''))

    def parseResponses(self, comandResponse):
        if 'PID vel' in comandResponse:
            comandResponse = comandResponse.replace('PID vel|', '')
            self.parsePIDFResponse(self.PIDVelocity, self.LPFVelocity, comandResponse)
        elif 'PID angle' in comandResponse:
            comandResponse = comandResponse.replace('PID angle|', '')
            self.parsePIDFResponse(self.PIDAngle, self.LPFAngle, comandResponse)
        elif 'PID curr q' in comandResponse:
            comandResponse = comandResponse.replace('PID curr q|', '')
            self.parsePIDFResponse(self.PIDCurrentQ, self.LPFCurrentQ, comandResponse)
        elif 'PID curr d' in comandResponse:
            comandResponse = comandResponse.replace('PID curr d|', '')
            self.parsePIDFResponse(self.PIDCurrentD, self.LPFCurrentD, comandResponse)
        elif 'Limits' in comandResponse:
            comandResponse = comandResponse.replace('Limits|', '')
            self.parseLimitsResponse(comandResponse)
        elif 'Motion' in comandResponse:
            comandResponse = comandResponse.replace('Motion:', '')
            self.parseMotionResponse(comandResponse)
        elif 'Torque' in comandResponse:
            comandResponse = comandResponse.replace('Torque:', '')
            self.parseTorqueResponse(comandResponse)
        elif 'Sensor' in comandResponse:
            comandResponse = comandResponse.replace('Sensor |', '')
            self.parseSensorResponse(comandResponse)
        elif 'Monitor' in comandResponse:
            comandResponse = comandResponse.replace('Monitor |', '')
            self.parseMonitorResponse(comandResponse)
        elif 'Status' in comandResponse:
            self.deviceStatus = float(comandResponse.replace('Status:', ''))
        elif 'R phase' in comandResponse:
            self.phaseResistance = float(comandResponse.replace('R phase:', ''))
        elif 'PWM Mod' in comandResponse:
            comandResponse = comandResponse.replace('PWM Mod | ', '')
            self.parsePWMModResponse(comandResponse)

    def parseStateResponses(self, comandResponse):
        if 'Monitor' in comandResponse:
            comandResponse = comandResponse.replace('Monitor |', '')
            self.parseMonitorResponse(comandResponse)


class ROS2ClientAsync(Node):

    def __init__(self):
        super().__init__('simplefoc_client_async')
        self.cli = self.create_client(SetDouble, 'set_double')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetDouble.Request()    

    def send_request(self, command, value, motor_id = 0, bus_id = 0):
        self.req.command = int(command)
        self.req.motor_id = int(motor_id)
        self.req.bus_id = int(bus_id)
        if value != '':
            self.req.value = float(value)
        self.future = self.cli.call_async(self.req)

class ROS2ReceiveHandler(QtCore.QThread):
    monitoringDataReceived = QtCore.pyqtSignal(list)
    commandDataReceived = QtCore.pyqtSignal(str)
    stateMonitorReceived = QtCore.pyqtSignal(str)
    rawDataReceived = QtCore.pyqtSignal(str)

    def __init__(self, serial_port=None, *args,**kwargs):        
        super(ROS2ReceiveHandler, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()
        rclpy.init()
        self.ros2_client = ROS2ClientAsync()

    #def send_request(self):
        #self.req.a = int(sys.argv[1])
        #self.future = self.cli.call_async(self.req)

    def handle_received_data(self, data):
        if not data:
            return
            
        if self.isDataReceivedMonitoring(data):
            try:
                v = data.rstrip().split('\t')
                self.monitoringDataReceived.emit(v)
            except ValueError as error:
                logging.error(error, exc_info=True)
                logging.error('data =' + str(data), exc_info=True)
            except IndexError as error:
                logging.error(error, exc_info=True)
                logging.error('data =' + str(data), exc_info=True)
        elif self.isDataReceivedStates(data):
            self.stateMonitorReceived.emit(data.rstrip())
        else:
            self.commandDataReceived.emit(data.rstrip())
        self.rawDataReceived.emit(data.rstrip())

    def isDataReceivedMonitoring(self, data):
        if data[0].isdigit() or data[0] == '-':
            return True
        else:
            return False
            
    def isDataReceivedStates(self, data):
        if 'Monitor' in data:
            return True
        else:
            return False

    def run(self):
        try:
            #self.ros2_client.send_request()
            while rclpy.ok():
                rclpy.spin_once(self.ros2_client)
                if self.ros2_client.future.done():
                    try:
                        response = self.ros2_client.future.result()
                    except Exception as e:
                        self.ros2_client.get_logger().info('Service call failed %r' % (e,))
                    else:
                        #minimal_client.get_logger().info('Result of add_two_ints: for %d + %d = %d' % (minimal_client.req.a, minimal_client.req.b, response.sum))
                        pass
                    break
        except TypeError as typeError:
            logging.error(typeError, exc_info=True)
        except AttributeError as ae:
            logging.error(ae, exc_info=True)            

    def stop(self):
        self._stop_event.set()
        print("ROS2 shutting down")        
        self.ros2_client.destroy_node()
        rclpy.shutdown()
        
    def stopped(self):
        return self._stop_event.is_set()


class ROS2StateUpdateRunner(QtCore.QThread):

    def __init__(self, connector=None, *args,**kwargs):
        super(ROS2StateUpdateRunner, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()
        self.deviceConnector = connector
    def run(self):
        try:
            while not self.stopped():
                if self.deviceConnector is not None:
                    #if self.deviceConnector.commProvider.serialComm.isOpen():
                    self.deviceConnector.updateStates()
                    time.sleep(1)
        except Exception as serialException:
            logging.error(serialException, exc_info=True)

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()