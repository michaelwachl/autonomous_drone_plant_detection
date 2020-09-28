import rospy        # use Time
from math import pi

PRINT_VALUES = False

class PID(object):
   """This class treats controllers created to have PID control on a system."""

   # initialization of flight PID controller
   def __init__(self, name, kp, kd, ki, minOutput, maxOutput, integratorMin, integratorMax, angleControl):

      self.name = name                     # name of controller
      self.m_kp = kp                       # weight of proportional control
      self.m_kd = kd                       # weight of derivative control
      self.m_ki = ki                       # weight of integral control
      self.m_minOutput = minOutput         # largest output from controller
      self.m_maxOutput = maxOutput         # smallest output from controller
      self.m_integratorMin = integratorMin    # maximum integral value
      self.m_integratorMax = integratorMax    # minimum integral value
      self.angle_control = angleControl      # if True, discontinuity in error is addressed

      # initial variable values
      self.m_integral = 0
      self.m_previousError = 0
      self.m_previousTime = float(rospy.Time.to_sec(rospy.Time.now()))

      # current error
      self.error = None

   # reset of essential PID parameters
   def reset (self):
      self.m_integral = 0
      self.m_previousError = 0
      self.m_previousTime = float(rospy.Time.to_sec(rospy.Time.now()))

   # sets (or resets) the integral value of the PID controller 
   def setIntegral (self, integral):
      self.m_integral = integral

   # returns the value of ki
   def set_ki (self):
      return self.m_ki

   # applies proportional, derivative, and integral control to determine optimal control value
   def update (self, value, targetValue):

      # record current value and target value and find current time
      if PRINT_VALUES:
         rospy.loginfo("values are %f and %f", value, targetValue)
      time = float(rospy.Time.to_sec(rospy.Time.now()))

      # delta time is the time since the last update
      dt = time - self.m_previousTime

      # error is the difference between the current value and the target value
      self.error = targetValue - value
      # check for discontinuity
      if self.angle_control:
         if self.error < -pi:
            print("error smaller PI")
            # proportional error will flip sign, but the integral error won't and the
            # filtered derivative will be poorly defined.
            # reset needed!
            self.reset()
            self.error += 2*pi
         if self.error > pi:
            print("error bigger PI")
            # proportional error will flip sign, but the integral error won't and the
            # filtered derivative will be poorly defined.
            # reset needed!
            self.reset()
            self.error -= 2*pi

      # determine the integral value based on the delta time and current error
      self.m_integral += self.error * dt

      # assure that integral is within the max and min otherwise use the central value (max or min)
      self.m_integral = max(min(self.m_integral, self.m_integratorMax), self.m_integratorMin)
      if PRINT_VALUES:
         rospy.loginfo("%s: dt is %f, error is %f and m_integral is %f",self.name, dt, self.error, self.m_integral)

      # calculate proportional based on kp and current error
      p = self.m_kp * self.error

      # calculate derivative based on kd and the differenced in the current error and the last error
      # and divide by delta time (check is made that delta time is not zero to prevent divide by 0)
      d = 0
      if dt > 0:
         d = self.m_kd * (self.error - self.m_previousError) / dt

      # calculate integral based on ki and integral value of PID controller
      i = self.m_ki * self.m_integral

      # sum the proportional, derivative and integral to obtain controller output value
      output = p + d + i
      if PRINT_VALUES:
         rospy.loginfo("%s: P is %f, D is %f, I is %f", self.name, p, d, i)

      # save current error and time for the next cycle
      self.m_previousError = self.error
      self.m_previousTime = time

      # assure that output value is within the max and min otherwise use the central value (max or min)
      returned_value = max(min(output, self.m_maxOutput), self.m_minOutput)
      # print(self.name, ' return: ', returned_value)
      return returned_value
