# Standard Library imports
import math
import sys
import errno
import rospy

# Thread / Locking imports
from collections import deque
from threading import Thread
from collections import defaultdict

# Dynamixel Messages import for JointState, DynamixelState, MotorStateList, and MotorState
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState as DynamixelState
from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import MotorState

# Port and Packet Handler imports
import dynamixel_sdk.port_handler as port_h
import dynamixel_sdk.packet_handler as packet_h

# Custom Driver imports
from dynomix_driver import sdk_serial_wrapper
from dynomix_tools.dynamixel_control_table import MODEL_NUMBER_2_MOTOR_NAME
from dynomix_tools import dynamixel_tools
from dynomix_driver import sdk_serial_wrapper
from dynomix_driver.dynamixel_const import *

# Leave here for debugging
  # import roslib
  # roslib.load_manifest('dynamixel_drive')


class DynomixSerialProxy():
  """
  Serial proxy that initializes dynamixel_sdk  port handler and packet handler
  """
  def __init__(self, 
    port_name='/dev/ttyUSB0', 
    port_namespace='ttyUSB0', 
    baud_rate=1000000, 
    min_motor_id=1,
    max_motor_id=25,
    update_rate=5,
    diagnostics_rate=1,
    error_level_temp=75,
    warn_level_temp=70,
    readback_echo=False,
    protocol_version=2.0):
  
    # Initialize variables for Serial Proxy
    self.port_name          = port_name
    self.port_namespace     = port_namespace
    self.baud_rate          = baud_rate
    self.min_motor_id       = min_motor_id
    self.max_motor_id       = max_motor_id
    self.update_rate        = update_rate
    self.diagnostics_rate   = diagnostics_rate
    self.error_level_temp   = error_level_temp
    self.warn_level_temp    = warn_level_temp
    self.readback_echp      = readback_echo
    self.protocol_version   = protocol_version

    self.actual_rate        = update_rate 
    self.error_counts       = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
    self.current_state = MotorStateList()
    self.num_ping_retries = 5
    self.dynotools = dynamixel_tools.DynamixelTools()
    self.sdk_io = sdk_serial_wrapper.SDKSerialWrapper(port_name, baud_rate)

    # Start to publish motor states
    self.motor_states_pub = rospy.Publisher('motor_states/%s' % self.port_namespace, MotorStateList, queue_size=1)
    #self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticsArray, queue_size=1)


  def connect(self):
    """
    Connects up physical port with the port handler, and initialize packet handler
    """
    try:
      # Port and packet handler set up
      self.port_handler   = port_h.PortHandler(self.port_name)
      self.packet_handler = packet_h.PacketHandler(self.protocol_version)

      # Set up port and baud rate
      self.port_handler.openPort()
      self.port_handler.setBaudRate(self.baud_rate)
      self.__find_motors()
    except rospy.ROSInterruptException: pass

    self.running = True

    # TODO: Implement
    if self.update_rate > 0: Thread(target=self.__update_motor_states).start()
    # if self.diagnostics_rate > 0: Thread(target=self.__publish_diagnostic_information).start()


  def disconnect(self):
    """
    Disconnects motors
    """
    self.running = False


  # TODO: IRVIN SWITCH THIS TO USE sdk_serial_wrapper.py (I agree, Dr. Kim, Shawn, or whoever
  #      put this here. -Marcus)
  def __fill_motor_parameters(self, motor_id, model_number):
    """
    Stores some extra information about each motor on the parameter server.
    Some of these paramters are used in joint controller implementation.
    """

    # If statements for filling each motor specifically from control table
    if model_number == H54_200_S500_R_MODEL_NUMBER:
      """
      angle_l_res = self.packet_handler.readTxRx(self.port_handler, motor_id, H54_200_S500_R_ANGLE_LIMIT_L, H54_200_S500_R_ANGLE_LIMIT_L_LENGTH)
      angle_l = angle_l_res[0][0]
      angle_h_res = self.packet_handler.readTxRx(self.port_handler, motor_id, H54_200_S500_R_ANGLE_LIMIT_H, H54_200_S500_R_ANGLE_LIMIT_H_LENGTH)
      angle_h = angle_h_res[0][0]
      angles = {'min': angle_l, 'max': angle_h}
      """
      

      # Get the Motor Name
      model_name = self.dynotools.getModelNameByModelNumber(model_number)

      # TODO: Get Max and Min angles
      angles = self.sdk_io.get_angle_limits(motor_id, model_name)
    
      rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
      rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
      rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
      rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
      

      torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
      rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
      rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
      
      velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
      rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
      rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
      rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
      rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
      
      encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
      range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
      range_radians = math.radians(range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
      rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
      rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
      
      # keep some parameters around for diagnostics
      self.motor_static_info[motor_id] = {}
      self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
      self.motor_static_info[motor_id]['min_angle'] = angles['min']
      self.motor_static_info[motor_id]['max_angle'] = angles['max']



    elif model_number == XM540_W270_T_NUMBER:
      
      angle_l = self.packet_handler.readTxRx(self.port_handler, motor_id, XM540_W270_T_ANGLE_LIMIT_L, XM540_W270_T_ANGLE_LIMIT_L_LENGTH)
      angle_h = self.packet_handler.readTxRx(self.port_handler, motor_id, XM540_W270_T_ANGLE_LIMIT_H, XM540_W270_T_ANGLE_LIMIT_H_LENGTH)
      angles = {'min': angle_l, 'max': angle_h}

      voltage = self.packet_handler.readTxRx(self.port_handler, motor_id, XM540_W270_T_PRESENT_VOLTAGE, XM540_W270_T_PRESENT_VOLTAGE_LENGTH)
      voltage_l = self.packet_handler.readTxRx(self.port_handler, motor_id, XM540_W270_T_PRESENT_VOLTAGE_L, XM540_W270_T_PRESENT_VOLTAGE_L_LENGTH)
      voltage_h = self.packet_handler.readTxRx(self.port_handler, motor_id, XM540_W270_T_PRESENT_VOLTAGE_H, XM540_W270_T_PRESENT_VOLTAGE_H_LENGTH) 
      voltages = {'min': voltage_l, 'max': voltage_h}
    
      rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
      rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
      rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
      rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
      

      torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
      rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
      rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
      
      velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
      rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
      rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
      rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
      rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
      
      encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
      range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
      range_radians = math.radians(range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
      rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
      rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
      
      # keep some parameters around for diagnostics
      self.motor_static_info[motor_id] = {}
      self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
      self.motor_static_info[motor_id]['firmware'] = self.dxl_io.get_firmware_version(motor_id)
      self.motor_static_info[motor_id]['delay'] = self.dxl_io.get_return_delay_time(motor_id)
      self.motor_static_info[motor_id]['min_angle'] = angles['min']
      self.motor_static_info[motor_id]['max_angle'] = angles['max']
      self.motor_static_info[motor_id]['min_voltage'] = voltages['min']
      self.motor_static_info[motor_id]['max_voltage'] = voltages['max']


    elif model_number == MX_106_T_2_NUMBER:
      
      angle_l_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_106_T_2_ANGLE_LIMIT_L, MX_106_T_2_ANGLE_LIMIT_L_LENGTH)
      angle_h_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_106_T_2_ANGLE_LIMIT_H, MX_106_T_2_ANGLE_LIMIT_H_LENGTH)
      angles = {'min': angle_l_res[0][0], 'max': angle_h_res[0][0]}

      voltage_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_106_T_2_PRESENT_VOLTAGE, MX_106_T_2_PRESENT_VOLTAGE_LENGTH)
      voltage = voltage_res[0][0]
      voltage_l_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_106_T_2_LIMIT_VOLTAGE_L, MX_106_T_2_LIMIT_VOLTAGE_L_LENGTH)
      voltage_l = voltage_l_res[0][0]
      voltage_h_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_106_T_2_LIMIT_VOLTAGE_H, MX_106_T_2_LIMIT_VOLTAGE_H_LENGTH) 
      voltage_h = voltage_h_res[0][0]
      voltages = {'min': voltage_l, 'max': voltage_h}
    
      rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
      rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
      rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
      rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
      

      torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
      rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
      print voltage
      rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
      
      velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
      rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
      rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
      rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
      rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
      
      encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
      range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
      range_radians = math.radians(range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
      rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
      rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
      
      # keep some parameters around for diagnostics
      self.motor_static_info[motor_id] = {}
      self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
      self.motor_static_info[motor_id]['min_angle'] = angles['min']
      self.motor_static_info[motor_id]['max_angle'] = angles['max']
      self.motor_static_info[motor_id]['min_voltage'] = voltages['min']
      self.motor_static_info[motor_id]['max_voltage'] = voltages['max']

    elif model_number == MX_64_T_2_NUMBER:

      angle_l_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_64_T_2_ANGLE_LIMIT_L, MX_64_T_2_ANGLE_LIMIT_L_LENGTH)
      angle_l = angle_l_res[0][0]
      angle_h_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_64_T_2_ANGLE_LIMIT_H, MX_64_T_2_ANGLE_LIMIT_H_LENGTH)
      angle_h = angle_h_res[0][0]
      angles = {'min': angle_l, 'max': angle_h}

      voltage_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_64_T_2_PRESENT_VOLTAGE, MX_64_T_2_PRESENT_VOLTAGE_LENGTH)
      voltage = voltage_res[0][0]
      voltage_l_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_64_T_2_LIMIT_VOLTAGE_L, MX_64_T_2_LIMIT_VOLTAGE_L_LENGTH)
      voltage_l = voltage_l_res[0][0]
      voltage_h_res = self.packet_handler.readTxRx(self.port_handler, motor_id, MX_64_T_2_LIMIT_VOLTAGE_H, MX_64_T_2_LIMIT_VOLTAGE_H_LENGTH) 
      voltage_h = voltage_h_res[0][0]
      rospy.logwarn("DEBUG VOLTAGE_H MX64: " + str(voltage_h_res[0]))
      voltages = {'min': voltage_l, 'max': voltage_h}
    
      rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
      rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
      rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
      rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
      

      torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
      rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
      rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
      
      velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
      rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
      rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
      rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
      rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
      
      encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
      range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
      range_radians = math.radians(range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
      rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
      rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
      rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
      rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
      
      # keep some parameters around for diagnostics
      self.motor_static_info[motor_id] = {}
      self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
      self.motor_static_info[motor_id]['min_angle'] = angles['min']
      self.motor_static_info[motor_id]['max_angle'] = angles['max']
      self.motor_static_info[motor_id]['min_voltage'] = voltages['min']
      self.motor_static_info[motor_id]['max_voltage'] = voltages['max']

    # Have in here for the purposes of having reference code
    # angles = self.dxl_io.get_angle_limits(motor_id)
    # voltage = self.dxl_io.get_voltage(motor_id)
    # voltages = self.dxl_io.get_voltage_limits(motor_id)
    
    # rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
    # rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
    # rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
    # rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
    
    # torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
    # rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
    # rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
    
    # velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
    # rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
    # rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
    # rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
    # rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
    
    # encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
    # range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
    # range_radians = math.radians(range_degrees)
    # rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
    # rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
    # rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
    # rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
    # rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
    # rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
    # rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
    
    # # keep some parameters around for diagnostics
    # self.motor_static_info[motor_id] = {}
    # self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
    # self.motor_static_info[motor_id]['firmware'] = self.dxl_io.get_firmware_version(motor_id)
    # self.motor_static_info[motor_id]['delay'] = self.dxl_io.get_return_delay_time(motor_id)
    # self.motor_static_info[motor_id]['min_angle'] = angles['min']
    # self.motor_static_info[motor_id]['max_angle'] = angles['max']
    # self.motor_static_info[motor_id]['min_voltage'] = voltages['min']
    # self.motor_static_info[motor_id]['max_voltage'] = voltages['max']


  def __find_motors(self):
    """
    Function to add motors into motor container with servo id, model name and model number
    """
    # Begin by adding 
    rospy.loginfo('%s: Pinging motor IDs %d through %d...' % (self.port_namespace, self.min_motor_id, self.max_motor_id))
    self.motors = []
    self.motor_static_info = {}

    self.motor_info = {} # TODO: contain information on model_num and IDs

    for motor_id in range(self.min_motor_id, self.max_motor_id + 1):
      for trial in range(self.num_ping_retries):
        try:
          result = self.packet_handler.ping(self.port_handler, motor_id)
          # if self.packet_handler.getRxPacketError(error):
          #   rospy.loginfo(self.packet_handler.getRxPacketError(error))
          # rospy.loginfo(self.packet_handler.getTxRxResult(result[0]))
        except Exception as ex:
          rospy.logerr('Exception thrown while pinging motor %d - %s' % (motor_id, ex))
          continue

        if not result[1]: # If no error was returned
          self.motors.append(motor_id)
          break

    if not self.motors:
      rospy.logfatal('%s: No motors found.' % self.port_namespace)
      sys.exit(1)
            
    counts = defaultdict(int)

    to_delete_if_error = []
    rospy.loginfo("Getting motor numbers.......")
    for motor_id in self.motors:
      for trial in range(self.num_ping_retries):
        # try:
        #   # model_number = self.packet_handler.ping(self.port_handler, motor_id) # TODO: Change to user sdk_serial_wrapper
        #   # result = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, 0) # TODO: IRVIN Implement wrapper to get model number
         
        #   rospy.logwarn("MOTOR_ID: " + motor_id)
        #   model_number = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, 0)
        #   # rospy.loginfo(result)
        #   # rospy.loginfo("ID: %d MOTOR_NUMBER: %d", motor_id, result[0])
        #   # rospy.loginfo("RESULT VALUE: " + self.packet_handler.getTxRxResult(result[0]))
        #   # rospy.loginfo("ERROR VALUE: " + self.packet_handler.getRxPacketError(result[1]))
        #   # model_number = result[0]
        #   rospy.logwarn("[dynomix_serial_proxy MODEL_NUMBER: %d ", model_number)
        #   self.__fill_motor_parameters(motor_id, model_number)
        # except Exception as ex:
        #   rospy.logerr('Exception thrown while getting attributes for motor %d - %s' % (motor_id, ex))
        #   if trial == self.num_ping_retries - 1: to_delete_if_error.append(motor_id)
        #   continue
        model_number = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, 0)
        rospy.logwarn("MOTOR_ID: " + str(motor_id))
        rospy.logwarn("MODEL_NUMBER: " + str(model_number[0]))
        rospy.logwarn("ERROR: " + str(model_number[1]))

        self.__fill_motor_parameters(motor_id, model_number[0])

        # counts[model_number] += 1
        counts[model_number[0]] += 1

        self.motor_info[str(motor_id)] = {"model_number": model_number[0]} # IRVIN

        break
 
    for motor_id in to_delete_if_error:
      self.motors.remove(motor_id)

    rospy.set_param('dynamixel/%s/connected_ids' % self.port_namespace, self.motors)

    rospy.set_param('dynamixel/%s/motor_info' % self.port_namespace, self.motor_info) # IRVIN

    status_str = '%s: Found %d motors - ' % (self.port_namespace, len(self.motors))
    for model_number,count in counts.items():
      if count:
        if model_number in MODEL_NUMBER_2_MOTOR_NAME:
          model_name = MODEL_NUMBER_2_MOTOR_NAME[model_number]['name']
          status_str += '%d %s [' % (count, model_name)
            
          for motor_id in self.motors:
            if motor_id in self.motor_static_info and self.motor_static_info[motor_id]['model'] == model_name:
              status_str += '%d, ' % motor_id  
          
          status_str = status_str[:-2] + '], '

    rospy.loginfo('%s, initialization complete.' % status_str[:-2])


  def __update_motor_states(self):
    num_events = 50
    rates = deque([float(self.update_rate)]*num_events, maxlen=num_events)
    last_time = rospy.Time.now()
    
    rate = rospy.Rate(self.update_rate)
    while not rospy.is_shutdown() and self.running:
      # get current state of all motors and publish to motor_states topic
      motor_states = []
      for motor_id in self.motors:
        try:
          # state = self.sdk_io.get_feedback(motor_id)      # TODO: IRVIN needs to Update to use new Serial proxy
          state = self.get_feedback(motor_id)
          if state:
            motor_states.append(MotorState(**state))
            # if dynamixel_io.exception: raise dynamixel_io.exception
        # except dynamixel_io.FatalErrorCodeError, fece:
        #   rospy.logerr(fece)
        # except dynamixel_io.NonfatalErrorCodeError, nfece:
        #   self.error_counts['non_fatal'] += 1
        #   rospy.logdebug(nfece)
        # except dynamixel_io.ChecksumError, cse:
        #   self.error_counts['checksum'] += 1
        #   rospy.logdebug(cse)
        # except dynamixel_io.DroppedPacketError, dpe:
        #   self.error_counts['dropped'] += 1
        #   rospy.logdebug(dpe.message)
        except OSError, ose:
          if ose.errno != errno.EAGAIN:
            rospy.logfatal(errno.errorcode[ose.errno])
            rospy.signal_shutdown(errno.errorcode[ose.errno])
            
      if motor_states:
        msl = MotorStateList()
        msl.motor_states = motor_states
        self.motor_states_pub.publish(msl)
        
        self.current_state = msl
        
        # calculate actual update rate
        current_time = rospy.Time.now()
        rates.append(1.0 / (current_time - last_time).to_sec())
        self.actual_rate = round(sum(rates)/num_events, 2)
        last_time = current_time
          
      rate.sleep()

  def get_feedback(self, servo_id):
      """
      Returns the id, goal, position, error, speed, load, voltage, temperature
      and moving values from the specified servo.
      """

    # TODO: multiple packet handler calls for the specificed packets.
    #       - Build up sdk serial wrapper to read from functions
    #       - Goal Position, Present Velocity, Present Current,
    #         Present Temperature, Position Trajectory
    # TODO: return library of called registers
      model_number = self.motor_info[str(servo_id)]['model_number']
      model_name = self.dynotools.getModelNameByModelNumber(model_number)
      goal = self.sdk_io.get_goal(servo_id, model_name)
      position = self.sdk_io.get_position(servo_id, model_name)
      error = position - goal
      speed = self.sdk_io.get_speed(servo_id, model_name)
      temperature = self.sdk_io.get_temperature(servo_id, model_name)
      moving = self.sdk_io.get_moving(servo_id, model_name)
      

      return { 'timestamp': 0,
          'id': servo_id,
          'goal': goal,
          'position': position,
          'error': error,
          'speed': speed,
          'load': 0,
          'voltage': 0,
          'temperature': temperature,
          'moving': bool(moving) }

      """
      model_number = self.motor_info[str(servo_id)]['model_number']
      model_name = self.dynotools.getModelNameByModelNumber(model_number)
      start_address = self.dynotools.getAddressSizeByModel(model_number, 'goal_position')
      summary_size = self.dynotools.getSumaryAddressSizeByModel(model_number)

      # read in X consecutive bytes starting with low value for goal position
      # response = self.read(servo_id, DXL_GOAL_POSITION_L, 17)
      raw_response  = self.packet_handler.readTxRx(self.port_handler, servo_id, start_address, summary_size)
      response = raw_response[0]
      # if response:
          # self.exception_on_error(response[4], servo_id, 'fetching full servo status')

      # rospy.logwarn("RESPONSE RAW: " + str(response))

      # rospy.logwarn("RESPONSE LENGTH: " + str(len(response)))

      if model_name == "MX_106_2" and len(response) == 31:
        # extract data values from the raw data
        goal = response[0] + (response[6] << 8)
        position = response[11] + (response[12] << 8)
        error = position - goal
        speed = response[13] + ( response[14] << 8)
        if speed > 1023: speed = 1023 - speed
        load_raw = response[15] + (response[16] << 8)
        load_direction = 1 if self.dynotools.test_bit(load_raw, 10) else 0
        load = (load_raw & int('1111111111', 2)) / 1024.0
        if load_direction == 1: load = -load
        voltage = response[17] / 10.0
        temperature = response[18]
        moving = response[21]
        timestamp = response[-1]  

        return { 'timestamp': timestamp,
          'id': servo_id,
          'goal': goal,
          'position': position,
          'error': error,
          'speed': speed,
          'load': load,
          'voltage': voltage,
          'temperature': temperature,
          'moving': bool(moving) }  


      elif model_name == "H54_200_S500_R_2" and len(response) == 31:
        # extract data values from the raw data
        goal = response[5] + (response[6] << 8)
        position = response[11] + (response[12] << 8)
        error = position - goal
        speed = response[13] + ( response[14] << 8)
        if speed > 1023: speed = 1023 - speed
        load_raw = response[15] + (response[16] << 8)
        load_direction = 1 if self.dynotools.test_bit(load_raw, 10) else 0
        load = (load_raw & int('1111111111', 2)) / 1024.0
        if load_direction == 1: load = -load
        voltage = response[17] / 10.0
        temperature = response[18]
        moving = response[21]
        timestamp = response[-1] 

        return { 'timestamp': timestamp,
          'id': servo_id,
          'goal': goal,
          'position': position,
          'error': error,
          'speed': speed,
          'load': load,
          'voltage': voltage,
          'temperature': temperature,
          'moving': bool(moving) }  


      if model_name == "XM540_w270_R_2" and len(response) == 31:
        # extract data values from the raw data
        goal = response[5] + (response[6] << 8)
        position = response[11] + (response[12] << 8)
        error = position - goal
        speed = response[13] + ( response[14] << 8)
        if speed > 1023: speed = 1023 - speed
        load_raw = response[15] + (response[16] << 8)
        load_direction = 1 if self.dynotools.test_bit(load_raw, 10) else 0
        load = (load_raw & int('1111111111', 2)) / 1024.0
        if load_direction == 1: load = -load
        voltage = response[17] / 10.0
        temperature = response[18]
        moving = response[21]
        timestamp = response[-1] 

        return { 'timestamp': timestamp,
          'id': servo_id,
          'goal': goal,
          'position': position,
          'error': error,
          'speed': speed,
          'load': load,
          'voltage': voltage,
          'temperature': temperature,
          'moving': bool(moving) }  


      elif model_name == "MX_64_T_2" and len(response) == 31:
        # extract data values from the raw data
        goal = response[5] + (response[6] << 8)
        position = response[11] + (response[12] << 8)
        error = position - goal
        speed = response[13] + ( response[14] << 8)
        if speed > 1023: speed = 1023 - speed
        load_raw = response[15] + (response[16] << 8)
        load_direction = 1 if self.dynotools.test_bit(load_raw, 10) else 0
        load = (load_raw & int('1111111111', 2)) / 1024.0
        if load_direction == 1: load = -load
        voltage = response[17] / 10.0
        temperature = response[18]
        moving = response[21]
        timestamp = response[-1]

        return { 'timestamp': timestamp,
          'id': servo_id,
          'goal': goal,
          'position': position,
          'error': error,
          'speed': speed,
          'load': load,
          'voltage': voltage,
          'temperature': temperature,
          'moving': bool(moving) }  

      # if len(response) == 24:
      #     # extract data values from the raw data
      #     goal = response[5] + (response[6] << 8)
      #     position = response[11] + (response[12] << 8)
      #     error = position - goal
      #     speed = response[13] + ( response[14] << 8)
      #     if speed > 1023: speed = 1023 - speed
      #     load_raw = response[15] + (response[16] << 8)
      #     load_direction = 1 if self.test_bit(load_raw, 10) else 0
      #     load = (load_raw & int('1111111111', 2)) / 1024.0
      #     if load_direction == 1: load = -load
      #     voltage = response[17] / 10.0
      #     temperature = response[18]
      #     moving = response[21]
      #     timestamp = response[-1]

          # return the data in a dictionary
    """
if __name__ == '__main__':
  try:
    serial_proxy = DynomixSerialProxy()
    serial_proxy.connect()
    rospy.spin()
    serial_proxy.disconnect()
  except rospy.ROSInterruption: pass