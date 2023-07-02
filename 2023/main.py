#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
import time, _thread, math

ArmMotor = Motor(Port.A)
RightMotor = Motor(Port.B)
LeftMotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
BoatMotor = Motor(Port.D)

ColorA = ColorSensor(Port.S1) # Name TBD
LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
# ColorB = ColorSensor(Port.S4) # Name TBD

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=157)
robot.settings(straight_speed=700)

markingblocks = [0, 0] # 0 = blue, 1 = green, 2 = None

def main():
  # # ** START **
  # straight(115)
  # sweep(sensor=LeftColor, direction="left")
  # _thread.start_new_thread(markingblockscanthread, ())
  # lfpidBlack(sensor=LeftColor, sideofsensor='in', startdistance=100, blackthreshold=10, whitethreshold=25, startncap=350, kp=0.4)
  # lfpidDistance(distance=50, sensor=LeftColor, sideofsensor='in')
  # straightuntilstop(150)
  # straightuntilstop(-150)
  # durn(turn=-110, type="tank")
  # straight(395)
  # durn(turn=-225, type="tank")
  # straightwithadjust(scandistance=200, movedistance=200, scanspeed=150, movespeed=400, changefactor=2)
  _thread.start_new_thread(recordrli, (400, ))
  straight(400)

  # ** CONTAINER PICKUP **
  # armGrab(ud="down")
  # time.sleep(2)
  # armGrab(ud="uptomid")
  # markingblockscanthread()
  


def markingblockscanthread():
  while True:
    print(ColorA.rgb(), rgbtocolor(ColorA.rgb()))

def recordrli(distance):
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < abs(distance):
    print(RightColor.reflection(), LeftColor.reflection())

def rgbtocolor(rgb): # None = 0, green = 1, blue = 2
  if sum(rgb) < 4:
    return 0
  elif rgb[2] > rgb[0] + rgb[1]:
    return 2
  elif rgb[1] > (rgb[0] + rgb[2]) * 0.75 or (rgb[2] / (rgb[1] + 1)) > 0.6:
    return 1
  else:
    return 0

def straight(distance, speed=400):
  if distance < 0:
    speed *= -1
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < abs(distance):
    robot.drive(speed, 0)
  robot.stop()

'''
def straightwithadjust(scandistance, movedistance, scanspeed=150, movespeed=400, changefactor):
  if movedistance < 100:
    raise Exception('movedistance must be greater than 100')
  if scandistance < 0:
    scanspeed *= -1
  if movedistance < 0:
    movespeed *= -1
  out = []
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < abs(scandistance):
    robot.drive(scanspeed, 0)
    out.append([RightColor.reflection(), LeftColor.reflection()])
  robot.stop()
  shift = getdriveoverangle(np.array(out))
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < 100:
    robot.drive(movespeed, shift * changefactor)
  robot.stop()
  while abs(robot.distance() - startdistance) < abs(movedistance) - 100:
    robot.drive(movespeed, 0)
  robot.stop()
'''

def square(threshold, speed):
  leftBlack = False
  rigthtBlack = False
  if LeftColor.reflection() <= threshold:
      leftBlack = True
  if RightColor.reflection() <= threshold:
    rigthtBlack = True
  if not leftBlack and not rigthtBlack:
    RightMotor.run(speed)
    LeftMotor.run(speed)
  while not leftBlack and not rigthtBlack:
    if LeftColor.reflection() <= threshold:
      leftBlack = True
      LeftMotor.hold()
    elif RightColor.reflection() <= threshold:
      rigthtBlack = True
      RightMotor.hold()
  
  if not rigthtBlack:
    RightMotor.run(speed)
  while not rigthtBlack:
    if RightColor.reflection() <= threshold:
      rigthtBlack = True
      RightMotor.hold()
  
  if not leftBlack:
    LeftMotor.run(speed)
  while not leftBlack:
    if LeftColor.reflection() <= threshold:
      leftBlack = True
      LeftMotor.hold()

def durn(turn, circleradius=30, type='tank', fb='forward', speed=200): # durn = degree turn
  if type not in ['tank', 'pivot', 'circle']:
    raise Exception('type must be "tank" or "pivot" or "circle"')
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')
  
  if fb == 'backward':
    speed *= -1

  startangle = robot.angle()
  if type == 'tank':
    if turn < 0:
      robot.drive(0, speed)
    else:
      robot.drive(0, -speed)
  elif type == 'pivot':
    if turn < 0:
      LeftMotor.run(speed)
    else:
      RightMotor.run(speed)
  elif type == 'circle':
    if turn < 0:
      robot.drive(speed, -circleradius)
    else:
      robot.drive(speed, circleradius)

  while abs(startangle - robot.angle()) < abs(turn):
    pass

  if type == 'tank' or type == 'circle':
    robot.stop()
  elif type == 'pivot':
    LeftMotor.hold()
    RightMotor.hold()



def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=25, startdistance=0, kp=0.25, ki=0, kd=0.5, startncap=[], estdistance=0, blackthreshold=10, whitethreshold=None): # wait distance is the # of mm after a black it waits until continue detecting blacks
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  if not isinstance(blackthreshold, int) and isinstance(blackthreshold, float):
    raise Exception('blackthreshold must be an integer')
  if not isinstance(whitethreshold, int) and isinstance(whitethreshold, float) and whitethreshold != None:
    raise Exception('whitethreshold must be an integer')

  if sensor == LeftColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

  if startncap == []:
    speed = [160]
  elif type(startncap) == int:
    speed = [startncap]
  else:
    speed = list(range(startncap[0], startncap[1], 1 if startncap[0] < startncap[1] else -1))
  
  target = (8 + 76) / 2 # Black  = 8, White = 76
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0
  count = 0

  robot.reset()
  lastdistance = abs(robot.distance())
  lastdistancechange = 0
  num = 0
  white = 0
  while count < blacks:
    if (LeftColor.reflection() + RightColor.reflection()) / 2 > whitethreshold:
      white += 1
    if (LeftColor.reflection() + RightColor.reflection()) / 2 < blackthreshold and lastdistance + waitdistance < abs(robot.distance()) and startdistance < abs(robot.distance()) and white > 0:
      count += 1
      lastdistance = abs(robot.distance())

    if abs(lastdistancechange - robot.distance()) > estdistance / len(speed) and num < len(speed) - 1:
      lastdistancechange = robot.distance()
      num += 1

    if sideofsensor == 'out':
      error = target - sensor.reflection()
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    integral += error
    derivative = error - lasterror
    turn = kp * error
    lasterror = error

    robot.drive(speed[num], turn)

  robot.stop()

def lfpidDistance(distance, sensor=RightColor, sideofsensor='in', startncap=[], kp=0.25, ki=0, kd=0.5):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  
  if sensor == LeftColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

  if startncap == []:
    speed = [160]
    one = True
  elif type(startncap) == int:
    speed = [startncap]
    one = True
  else:
    speed = list(range(startncap[0], startncap[1]))
    one = False

  target = (8 + 76) / 2 # Black  = 8, White = 76
  gyrodev = []
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0

  robot.reset()

  lastdistance = 0
  num = 0
  while abs(robot.distance()) < distance:
    if abs(lastdistance - robot.distance()) > distance / len(speed):
      lastdistance = robot.distance()
      num += 1
    if sideofsensor == 'out':
      error = target - sensor.reflection()
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    integral += error
    derivative = error - lasterror
    turn = kp * error + ki * integral + kd * derivative
    lasterror = error

    if one:
      robot.drive(speed[0], turn)
    else:
      robot.drive(speed[num], turn)

  robot.stop()

def boatGrab(movement="open", percentage=1):
  if movement == "open":
    BoatMotor.run_angle(-400, 95 * percentage)
    time.sleep(0.01)
  elif movement == "close":
    BoatMotor.run(400)
    time.sleep(0.3 * percentage)
    BoatMotor.stop()
    time.sleep(0.01)

def armGrab(ud='up'):
  if ud == 'down':
    time.sleep(0.001)
    ArmMotor.run_angle(300, 200)
  elif ud == 'uptomid':
    ArmMotor.run(-400)
    time.sleep(0.6)
    ArmMotor.run_angle(400, 50)
    ArmMotor.stop()
  elif ud == "full":
    ArmMotor.run_angle(250, 200)
    ArmMotor.run(-400)
    time.sleep(0.6)
    ArmMotor.run_angle(200, 200)
    ArmMotor.stop()


def sweep(sensor, direction, speed=50):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if direction not in ['right', 'left']:
    raise Exception('direction must be "right" or "left"')

  startangle = robot.angle()
  info = []
  target = (8 + 76) / 2 # Black  = 8, White = 76
  
  if direction == 'right':
    robot.drive(0, speed)
  else:
    robot.drive(0, -speed)
  
  while sensor.reflection() > 15:
    info.append([sensor.reflection(), robot.angle()])

  robot.stop()

  if info == []:
    return None # maybe add opposite later

  reflection = []
  for i in info:
    reflection.append(i[0])
 
  maxindex = reflection.index(max(reflection))
  info, reflection = info[maxindex:], reflection[maxindex:]

  closest = float('inf')
  closestindex = 0
  for i in reflection:
    if abs(i - target) < closest:
      closest = abs(i - target)
      closestindex = reflection.index(i)

  targetangle = info[closestindex][1]

  durn(robot.angle() - targetangle, type='tank', fb='forward', speed=speed)

def sTurn(rl, fb, turn, type='pivot', drive=0, turnSpeed=100): # rl = right-left, fb = forward-backward, turn = turn degrees(posotive), drive = drive between turns(positive)
  if rl not in ['right', 'left']:
    raise Exception('rl must be "right" or "left"')
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')
  
  if rl == 'right':
    turn *= -1
  if fb == 'backward':
    drive *= -1
    conificient = 1
  else:
    conificient = -1

  startangle = robot.angle()

  durn(turn, type=type, fb=fb, speed=turnSpeed)
  if drive != 0:
    straight(drive)
  durn(conificient * (startangle - robot.angle()), type=type, fb=fb, speed=turnSpeed)

'''
def fixdata(data, window_size=21, order=5):
  # prep data
  out = []
  start = []
  end = []
  for i in range(len(data[0, :])):
    deriv=0
    rate=1
    y = data[:,i].copy()
    order_range = range(order + 1)
    half_window = (window_size - 1) // 2
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * math.factorial(deriv)
    firstvals = y[0] - np.abs(y[1:half_window+1][::-1] - y[0])
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    a = np.convolve(m[::-1], y, mode='valid')

    for j in range(len(a) - 1):
      if a[j + 1] - a[j] > 3:
        f = [a[j]] * len(a[:j])
        g = a[j:]
        f.extend(g)
        a = f.copy()
        break

    for j in reversed(range(len(a) - 1)):
      if a[j] - a[j + 1] > 3:
        f = a[:j + 1]
        g = [a[j + 1]] * len(a[j + 1:])
        f.extend(g)
        a = f.copy()
        break

    out.append(a)

  # plot data
  maxandmin = []
  for i in range(len(out)):
    x = np.arange(0, len(out[i]), 1)

    fixeddata = out[i] / np.max(out[i])
    try:
      maxandmin.append([argrelextrema(fixeddata, np.greater)[0][0], argrelextrema(fixeddata, np.greater)[0][1], argrelextrema(fixeddata, np.less)[0][0]])
    except:
      return None

  # get differnce
  return sum([a_i - b_i for a_i, b_i in zip(maxandmin[0], maxandmin[1])]) / len(maxandmin[0])

def getdriveoverangle(data):
  a = fixdata(data)
  b = 7
  while a == None:
    a = fixdata(data, 21, b)
    b += 2
  return a
'''

def localMaxList(data):
  order=1

  locs = list(range(0, len(data), step))

  results = [True] * len(data)
  main = take(data, locs)
  for shift in range(1, order + 1):
    plus = take(data, [i + shift for i in locs])
    minus = take(data, [i - shift for i in locs])
    results = andequal(results, [main[i] > plus[i] for i in range(len(main))])
    results = andequal(results, [main[i] > minus[i] for i in range(len(main))])
  return [i for i, val in enumerate(results) if val]

def localMinList(data):
  order=1

  locs = list(range(0, len(data), step))

  results = [True] * len(data)
  main = take(data, locs)
  for shift in range(1, order + 1):
    plus = take(data, [i + shift for i in locs])
    minus = take(data, [i - shift for i in locs])
    results = andequal(results, [main[i] < plus[i] for i in range(len(main))])
    results = andequal(results, [main[i] < minus[i] for i in range(len(main))])
  return [i for i, val in enumerate(results) if val]

def take(array, indices):
  out = []
  for i in indices:
    if i < len(array):
      out.append(array[i])
    else:
      out.append(array[len(array) - 1])
  return out

def andequal(list1, list2):
  out = []
  for i in range(len(list1)):
    if list1[i] == list2[i]:
      out.append(list1[i])
    else:
      out.append(False)
  return out

starttime = time.time()
main()
print(time.time() - starttime)