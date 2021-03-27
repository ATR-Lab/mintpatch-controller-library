
import time
import serial
from array import array
from binascii import b2a_hex
from threading import Lock

import dynamixel_sdk.port_handler as port_h
import dynamixel_sdk.packet_handler as packet_h
from dynamixel_sdk import *
from dynamixel_const import *

import rospy

class SDKSerialWrapper:
  def __init__(self, port_name, baudrate, feedback_echo=False, protocol_version=2.0):
    # TODO: Should try/catch in case of errors
    self.serial_mutex   = Lock()
    self.port_handler   = port_h.PortHandler(port_name)
    self.packet_handler = packet_h.PacketHandler(protocol_version)
    self.port_handler.openPort()
    self.port_handler.setBaudRate(baudrate)


  # def __read_response(self, servo_id):
  #   data = []

  #   try:
  #     data.extend(self.ser.read(4))
  #     if not data[0:2] == ['\xff', '\xff']: raise Exception('Wrong packet prefix %s' % data[0:2])
  #     data.extend(self.ser.read(ord(data[3])))
  #     data = array('B', ''.join(data)).tolist() # [int(b2a_hex(byte), 16) for byte in data]
  #   except Exception, e:
  #     raise DroppedPacketError('Invalid response received from motor %d. %s' % (servo_id, e))

  #   # verify checksum
  #   checksum = 255 - sum(data[2:-1]) % 256
  #   if not checksum == data[-1]: raise ChecksumError(servo_id, data, checksum)

  #   return data

  # TODO: IRVIN NEEDS TO REALLY IMPLEMEMT THIS
  def read(self, servo_id, address, size):
    """Read "size" bytes of data from servo with a given "servo_id" at
    the register with "address". "address" is an integer between 0 and 57.
    It is recommended to use the constant in module for readability

    e.g: to read from servo with 1,
      read(1, MX_106_GOAL_POSITION, 2)
    """
    # IRVIN READ with the SDK packet handler
    with self.serial_mutex:
      result = self.packet_handler.readTxRx(self.port_handler, servo_id, address, size)

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.0013)#0.00235)

      # read response
      # data = self.packet_handler.getTxRxResult(response)
      # data.append(timestamp)
    return result

  # def __read_response(self, servo_id):
  #   data = []

  #   try:
  #     data.extend(self.ser.read(4))
  #     if not data[0:2] == ['\xff', '\xff']: raise Exception('Wrong packet prefix %s' % data[0:2])
  #     data.extend(self.ser.read(ord(data[3])))
  #     data = array('B', ''.join(data)).tolist() # [int(b2a_hex(byte), 16) for byte in data]
  #   except Exception, e:
  #     raise DroppedPacketError('Invalid response received from motor %d. %s' % (servo_id, e))

  #   # verify checksum
  #   checksum = 255 - sum(data[2:-1]) % 256
  #   if not checksum == data[-1]: raise ChecksumError(servo_id, data, checksum)

  #   return data

  def write(self, servo_id, address, data):
    """ Write the values from the "data" list to the servo with "servo_id"
    starting with data[0] at "address", continuing through data[n-1] at
    "address" + (n-1), where n = len(data). "address" is an integer between
    0 and 49. It is recommended to use the constants in module dynamixel_const
    for readability. "data" is a list/tuple of integers.
    To set servo with id 1 to position 276, the method should be called
    like:
        write(1, DXL_GOAL_POSITION_L, (20, 1))
    """
    raise NotImplementedError
    # Number of bytes following standard header (0xFF, 0xFF, id, length)
    # length = 3 + len(data)  # instruction, address, len(data), checksum

    # # directly from AX-12 manual:
    # # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    # # If the calculated value is > 255, the lower byte is the check sum.
    # checksum = 255 - ((servo_id + length + DXL_WRITE_DATA + address + sum(data)) % 256)

    # # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
    # packet = [0xFF, 0xFF, servo_id, length, DXL_WRITE_DATA, address]
    # packet.extend(data)
    # packet.append(checksum)

    # packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

    # with self.serial_mutex:
    #   self.__write_serial(packetStr)

    #   # wait for response packet from the motor
    #   timestamp = time.time()
    #   time.sleep(0.0013)

    #   # read response
    #   data = self.__read_response(servo_id)
    #   data.append(timestamp)

    #   return data


  def sync_write(self, address, data):
    """ Use Broadcast message to send multiple servos instructions at the
    same time. No "status packet" will be returned from any servos.
    "address" is an integer between 0 and 49. It is recommended to use the
    constants in module dynamixel_const for readability. "data" is a tuple of
    tuples. Each tuple in "data" must contain the servo id followed by the
    data that should be written from the starting address. The amount of
    data can be as long as needed.
    To set servo with id 1 to position 276 and servo with id 2 to position
    550, the method should be called like:
        sync_write(DXL_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
    """
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    with self.serial_mutex:
      self.__write_serial(packetStr)


  def ping(self, servo_id):
    """ Ping the servo with "servo_id". This causes the servo to return a
    "status packet". This can tell us if the servo is attached and powered,
    and if so, if there are any errors.
    """
    with self.serial_mutex:
      result = self.packet_handler(self.port_handler, servo_id)

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.0013)

      # # read response
      # try:
      #   response = self.__read_response(servo_id)
      #   response.append(timestamp)
      # except Exception, e:
      #   response = []

    if error:
      rospy.logerror("Servo ID: %d, 'PING', %s", servo_id, response)
    return result


  ##########################################
  # SINGLE SERVO FUNCTIONS
  ##########################################
  def set_torque_enabled(self, servo_id, enabled):
    """
    Sets the value of the torque enabled register to 1 or 0. When the
    torque is disabled the servo can be moved manually while the motor is
    still powered.
    """
    response = self.write(servo_id, DXL_TORQUE_ENABLE, [enabled])
    # if response:
      # self.exception_on_error(response[4], servo_id, '%sabling torque' % 'en' if enabled else 'dis')
    return response


  #################################
  # Servo status access functions #
  #################################
  def get_model_number(self, servo_id):
    """ Reads the servo's model number (e.g. 12 for AX-12+). """
    response = self.read(servo_id, DXL_MODEL_NUMBER_L, 2)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching model number')
    return response[5] + (response[6] << 8)

  def get_firmware_version(self, servo_id):
    """ Reads the servo's firmware version. """
    response = self.read(servo_id, DXL_VERSION, 1)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching firmware version')
    return response[5]

  def get_return_delay_time(self, servo_id):
    """ Reads the servo's return delay time. """
    response = self.read(servo_id, DXL_RETURN_DELAY_TIME, 1)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching return delay time')
    return response[5]

  def get_angle_limits(self, servo_id):
    """
    Returns the min and max angle limits from the specified servo.
    """
    # read in 4 consecutive bytes starting with low value of clockwise angle limit
    response = self.read(servo_id, DXL_CW_ANGLE_LIMIT_L, 4)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching CW/CCW angle limits')
    # extract data valus from the raw data
    cwLimit = response[5] + (response[6] << 8)
    ccwLimit = response[7] + (response[8] << 8)

    # return the data in a dictionary
    return {'min':cwLimit, 'max':ccwLimit}

  def get_drive_mode(self, servo_id):
    """ Reads the servo's drive mode. """
    response = self.read(servo_id, DXL_DRIVE_MODE, 1)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching drive mode')
    return response[5]

  def get_voltage_limits(self, servo_id):
    """
    Returns the min and max voltage limits from the specified servo.
    """
    response = self.read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 2)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching voltage limits')
    # extract data valus from the raw data
    min_voltage = response[5] / 10.0
    max_voltage = response[6] / 10.0

    # return the data in a dictionary
    return {'min':min_voltage, 'max':max_voltage}

  def get_position(self, servo_id):
    """ Reads the servo's position value from its registers. """
    response = self.read(servo_id, DXL_PRESENT_POSITION_L, 2)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present position')
    position = response[5] + (response[6] << 8)
    return position

  def get_speed(self, servo_id):
    """ Reads the servo's speed value from its registers. """
    response = self.read(servo_id, DXL_PRESENT_SPEED_L, 2)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present speed')
    speed = response[5] + (response[6] << 8)
    if speed > 1023:
      return 1023 - speed
    return speed

  def get_voltage(self, servo_id):
    """ Reads the servo's voltage. """
    response = self.read(servo_id, DXL_PRESENT_VOLTAGE, 1)
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching supplied voltage')
    return response[5] / 10.0

  def get_current(self, servo_id):
    """ Reads the servo's current consumption (if supported by model) """
    model = self.get_model_number(servo_id)
    if not model in DXL_MODEL_TO_PARAMS:
      raise UnsupportedFeatureError(model, DXL_CURRENT_L)

    if DXL_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
      response = self.read(servo_id, DXL_CURRENT_L, 2)
      # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching sensed current')
      current = response[5] + (response[6] << 8)
      return 0.0045 * (current - 2048)

    if DXL_SENSED_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
      response = self.read(servo_id, DXL_SENSED_CURRENT_L, 2)
      # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching sensed current')
      current = response[5] + (response[6] << 8)
      return 0.01 * (current - 512)

    else:
      raise UnsupportedFeatureError(model, DXL_CURRENT_L)



  def get_feedback(self, servo_id):
    """
    Returns the id, goal, position, error, speed, load, voltage, temperature
    and moving values from the specified servo.
    """
    # read in 17 consecutive bytes starting with low value for goal position
    response = self.read(servo_id, DXL_GOAL_POSITION_L, 17)

    # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching full servo status')
    if len(response) == 31:
        # extract data values from the raw data
        goal = response[5] + (response[6] << 8)
        position = response[11] + (response[12] << 8)
        error = position - goal
        speed = response[13] + ( response[14] << 8)
        if speed > 1023: speed = 1023 - speed
        load_raw = response[15] + (response[16] << 8)
        load_direction = 1 if self.test_bit(load_raw, 10) else 0
        load = (load_raw & int('1111111111', 2)) / 1024.0
        if load_direction == 1: load = -load
        voltage = response[17] / 10.0
        temperature = response[18]
        moving = response[21]
        timestamp = response[-1]

        # return the data in a dictionary
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
