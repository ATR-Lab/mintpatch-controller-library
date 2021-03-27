#!/usr/bin/env python

from dynomix_tools.dynamixel_control_table import MODEL_NUMBER_2_MOTOR_NAME
from dynomix_tools.dynamixel_control_table import MOTOR_CONTROL_TABLE

class DynamixelTools:
  def __init__(self):
    self.x = None

  def convertPosition2Radian(self, actuator_id, position_val):
    """
    Converts a position value of a given actuator to its radian representation
    Return -1 if the model is invalid
    """
    # TODO: Get Model Info
    # model_info = dynamixel_info.getInfo(actuator_id);
    model_info = None
    if model_info == None:
      return -1

    if position_val > model_info.val_zero_radian_position:
      return (float(position_val) - model_info.val_zero_radian_position) * model_info.max_radian / (float(model_info.val_max_radian_position) - model_info.val_zero_radian_position)
    elif position_val < model_info.val_zero_radian_position:
      return (float(position_val) - model_info.val_zero_radian_position) * model_info.min_radian / (float(model_info.val_min_radian_position) - model_info.val_zero_radian_position)


  def convertPosition2Radian(self, position_val, min_position, max_position, min_radian, max_radian):
    """
    Converts a position value of a given actuator to its radian representation
    """
    zero_position = (max_position + min_position) / 2

    if position_val > zero_position:
      return ((float(position_val) - zero_position) * max_radian / (float(max_position) - zero_position))
    elif position_val < zero_position:
      return ((float(position_val) - zero_position) * min_radian / (float(min_position) - zero_position))

  def convertRadian2Position(self, radian_val, min_position, max_position, min_radian, max_radian):
    """
    Converts a radian value to a position representation given the desired min and max position and radian values
    """
    zero_position = (max_position + min_position) / 2

    if radian_val > 0:
      return (radian_val * (max_position - zero_position) / max_radian) * zero_position
    elif radian_val < 0:
      return (radian_val * (min_position - zero_position) / min_radian) * zero_position
    else: 
      return zero_position

  ########################################################################
  #
  ########################################################################
  def getAddressSizeByModel(self, raw_servo_model, register_name):
    servo_model = MODEL_NUMBER_2_MOTOR_NAME[raw_servo_model]['name']
    return MOTOR_CONTROL_TABLE[str(servo_model)][register_name]['size']

  def getRegisterAddressByModel(self, raw_servo_model, register_name):
    servo_model = MODEL_NUMBER_2_MOTOR_NAME[raw_servo_model]['name']
    return  MOTOR_CONTROL_TABLE[str(servo_model)][register_name]['address']

  def getModelNameByModelNumber(self, raw_servo_model):
    return MODEL_NUMBER_2_MOTOR_NAME[raw_servo_model]['name']

  def getSumaryAddressSizeByModel(self, raw_servo_model):
    """
    The n bits of the registers that allows for a single read
    {'timestamp': timestamp,
    'id': servo_id,
    'goal': goal,
    'position': position,
    'error': error,
    'speed': speed,
    'load': load,
    'voltage': voltage,
    'temperature': temperature,
    'moving': bool(moving) }
    """
    servo_model = MODEL_NUMBER_2_MOTOR_NAME[raw_servo_model]['name']
    return  MOTOR_CONTROL_TABLE[str(servo_model)]['summary_size']

  def test_bit(self, number, offset):
    mask = 1 << offset
    return (number & mask)