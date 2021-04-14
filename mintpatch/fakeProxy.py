#Nathan Moder

from emulatedMotor import tm1
from emulatedMotor import tm2
from emulatedMotor import tm3

class DynomixSerialProxy():
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
    self.motors=[]
    self.port_name=port_name
    if(port_namespace=='port_1'):
      #self.motors[0]=tm1
      self.motors.append(1)
      #self.motors[1]=tm2
      self.motors.append(15)
    if(port_namespace=='port_2'):
      self.motors.append(110)
    #print("proxy init")
  def connect(self):
    self.__find_motors()
  def __find_motors(self):
    a=1
  def set_goal_position(self, servo_id, goal):
        #print(self.port)
        if self.port_name=="/dev/port_1":
            #print(servo_id)
            if 1==int(servo_id):
                #print("into if")
                tm1.set_goal_speed(goal)
            elif int(servo_id)==15:
                tm2.set_goal_speed(goal)
        elif self.port_name=="/dev/port_2":
            if int(servo_id)==110:
                tm3.set_goal_speed(goal)

  def get_feedback(self,servo_id):
    if self.port_name=="/dev/port_1":
      if int(servo_id)==1:
        if tm1.moving:
          tm1.check_while_running()
        return {
              'id':tm1.id,
              'goal': 0,
              'position': tm1.angle,
              'error' : 0,
              'speed' : tm1.speed,
              'load' : 0,
              'voltage' : tm1.voltage,
              'temperature' : tm1.temperature,
              'moving' : tm1.moving
              }
      if int(servo_id)==15:
        if tm2.moving:
          tm2.check_while_running()
        return {
              'id':tm2.id,
              'goal': 0,
              'position': tm2.angle,
              'error' : 0,
              'speed' : tm2.speed,
              'load' : 0,
              'voltage' : tm2.voltage,
              'temperature' : tm2.temperature,
              'moving' : tm2.moving
              }
    if self.port_name=="/dev/port_2":
      if int(servo_id)==110:
        if tm3.moving:
          tm3.check_while_running()
        return {
          'id':tm3.id,
          'goal': 0,
          'position': tm3.angle,
          'error' : 0,
          'speed' : tm3.speed,
          'load' : 0,
          'voltage' : tm3.voltage,
          'temperature' : tm3.temperature,
          'moving' : tm3.moving
          }
    return 'no such servo'
  def set_torque_enabled(self, sid, thing):
    a=1