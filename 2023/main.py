#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
import time, _thread, math, json

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

markingblocks = [3, 3] # 0 = blue, 1 = green, 2 = none, 3 = not scaned
containers = [3, 3, 3, 3] # 0 = blue, 1 = green, 2 = none, 3 = not scaned
# SavitzkyGolayFilterData = json.load(open("data.json", "r"))

def main():
  '''
  # ** START **
  straight(115)
  sweep(sensor=LeftColor, direction="left")
  _thread.start_new_thread(markingblockscanthread, (300,))
  lfpidBlack(sensor=LeftColor, sideofsensor='out', startdistance=100, blackthreshold=10, whitethreshold=25, speed=350, kp=0.4)
  lfpidDistance(distance=50, sensor=LeftColor, sideofsensor='out')
  straight(150, speed=300)
  straight(-150)
  durn(turn=-110, type="tank")
  durn(turn=112, circleradius=-50, type="circle", speed=300)
  durn(turn=-160, type="tank")
  straight(-190, speed=250)
  boatGrab(movement="close")

  # ** MOVE BOAT TO CONTAINER PICKUP **
  straight(50)
  durn(turn=-140, type="tank", speed=150)
  straight(260)
  durn(turn=80, type="tank", speed=150)
  sweep(sensor=RightColor, direction="left", whiteFirst=True)
  lfpidBlack(sensor=RightColor, sideofsensor='in', blackthreshold=10, whitethreshold=25, speed=150, kp=0.4)
  lfpidDistance(distance=110, sensor=RightColor, sideofsensor='in', speed=150, kp=0.4)
  durn(turn=120, type="tank", speed=150)
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lfpidDistance(distance=185, sensor=LeftColor, sideofsensor='out', speed=150, kp=0.4)
  boatGrab(movement="open")

  # ** CONTAINER SCAN **
  durn(turn=-160, type="pivot", speed=300)
  straight(-120, speed=300)
  containers[3] = colorScan(acceptable=[1, 2], direction="out", speed=300)
  straight(-100, speed=300)
  containers[2] = colorScan(acceptable=[1, 2], direction="out", speed=300)
  straight(-100, speed=300)
  containers[1] = colorScan(acceptable=[1, 2], direction="out", speed=300)
  if None in containers:
    straight(-100, speed=300)
    containers[0] = colorScan(acceptable=[1, 2], direction="out", speed=300)
  print(containers)
  '''
  
  # ** CONTAINER PICKUP **

  # blue
  # armGrab(ud="down")
  # armGrab(ud="uptomid", speed=200)

  # green
  armGrab(ud="down")
  armGrab(ud="uptomid", speed=300)


def markingblockscanthread(distance):
  colorList = []                      
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < abs(distance):
    colorList.append(rgbtocolor(ColorA.rgb()))
  print(colorList)

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

def straightwithadjust(scandistance, movedistance, changefactor, scanspeed=150, movespeed=400):
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
  newout = [[out[0][0]], [out[0][1]]]
  for i in range(1, len(out)):
    newout[0].append(out[i][0])
    newout[1].append(out[i][1])
  print(newout)
  shift = getdriveoverangle(newout)
  print(shift)
  quit()
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < 100:
    robot.drive(movespeed, shift * changefactor)
  robot.stop()
  while abs(robot.distance() - startdistance) < abs(movedistance) - 100:
    robot.drive(movespeed, 0)
  robot.stop()

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



def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=25, startdistance=0, kp=0.25, ki=0, kd=0.5, speed=[], estdistance=0, blackthreshold=10, whitethreshold=None): # wait distance is the # of mm after a black it waits until continue detecting blacks
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  if not isinstance(blackthreshold, int) and isinstance(blackthreshold, float):
    raise Exception('blackthreshold must be an integer')
  if not isinstance(whitethreshold, int) and isinstance(whitethreshold, float) and whitethreshold != None:
    raise Exception('whitethreshold must be an integer')

  if sensor == RightColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

  if speed == []:
    speed = [160]
  elif type(speed) == int:
    speed = [speed]
  else:
    speed = list(range(speed[0], speed[1], 1 if speed[0] < speed[1] else -1))
  
  target = (8 + 76) / 2 # Black  = 8, White = 76
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0
  count = 0

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

def lfpidDistance(distance, sensor=RightColor, sideofsensor='in', speed=[], kp=0.25, ki=0, kd=0.5):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  
  if sensor == RightColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

  if speed == []:
    speed = [150]
    one = True
  elif type(speed) == int:
    speed = [speed]
    one = True
  else:
    speed = list(range(speed[0], speed[1]))
    one = False

  target = (8 + 76) / 2 # Black  = 8, White = 76
  gyrodev = []
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0

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
    turn = kp * error
    lasterror = error

    if one:
      robot.drive(speed[0], turn)
    else:
      robot.drive(speed[num], turn)

  robot.stop()

def boatGrab(movement="open", percentage=1):
  if movement == "open":
    BoatMotor.stop()
    BoatMotor.run_angle(-400, 95 * percentage)
    time.sleep(0.01)
  elif movement == "close":
    BoatMotor.run(400)
    time.sleep(0.3 * percentage)

def armGrab(ud, speed=None):
  if ud == 'down':
    time.sleep(0.001)
    if speed == None:
      speed = 300
    ArmMotor.run_angle(speed, 250)
  elif ud == 'uptomid':
    if speed == None:
      speed = 400
    ArmMotor.run(-speed)
    time.sleep(0.6 * (400 / speed))
    ArmMotor.run_angle(speed, 50)
    ArmMotor.stop()
  elif ud == "full":
    if speed == None:
      speed = 200
    ArmMotor.run_angle(speed, 200)
    ArmMotor.run(-400)
    time.sleep(0.6)
    ArmMotor.run_angle(speed, 200)
    ArmMotor.stop()


def sweep(sensor, direction, speed=100, whiteFirst=False):
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
  
  if whiteFirst:
    while sensor.reflection() < 60:
      info.append([sensor.reflection(), robot.angle()])

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

def fixdata(data, window_size=21, order=5):
  a = SavitzkyGolayFilter(data, window_size=window_size, order=order)

  for j in range(len(a) - 1):
    if a[j + 1] - a[j] > 4: 	
      f = [a[j]] * len(a[:j])
      g = a[j:]
      f.extend(g)
      a = f.copy()
      break

  for j in reversed(range(len(a) - 1)):
    if a[j] - a[j + 1] > 4:
      f = a[:j + 1]
      g = [a[j + 1]] * len(a[j + 1:])
      f.extend(g)
      a = f.copy()
      break

  fixeddata = []
  for j in range(len(a)):
    fixeddata.append(a[j] / max(a))
  localMaxOut = localMaxList(fixeddata)
  localMinOut = localMinList(fixeddata)

  if len(localMaxOut) != 2 or len(localMinOut) != 1:
    return None
  return [localMaxOut[0], localMaxOut[1], localMinOut[0]]

def SavitzkyGolayFilter(data, window_size=21, order=5):
  half_window = (window_size - 1) // 2
  firstvals = [-abs(i - data[0]) + data[0] for i in data[1:half_window+1][::-1]]
  lastvals = [abs(i - data[-1]) + data[-1] for i in data[-half_window-1:-1][::-1]]
  data = firstvals + data + lastvals

  return convolve(data, SavitzkyGolayFilterData[str(window_size) + "," + str(order)])
  
def convolve(data, f):
  out = []
  for i in range(len(data) - len(f) + 1):
    out.append(sum([data[i + j] * f[j] for j in range(len(f))]))
  return out

def localMaxList(data):
  order=1

  locs = list(range(0, len(data)))                                                                                                                                                                                                                                                                                                                                                                           

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

  locs = list(range(0, len(data)))

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

def getdriveoverangle(data):
  allminmax = []
  for i in range(len(data)):
    breakOn = False
    for j in range(7, 15, 2):
      if breakOn:
        break
      for k in range(21, 31, 2):
        if breakOn:
          break
        print(i, j, k)
        a = fixdata(data[i], k, j)
        if a != None:
          breakOn = True
          allminmax.append(a)
          break
  out = [allminmax[0][i] - allminmax[1][i] for i in range(len(allminmax[0]))]
  return sum(out) / len(out)

def colorScan(acceptable, direction, speed=200):
  if rgbtocolor(ColorA.rgb()) in acceptable:
    return rgbtocolor(ColorA.rgb())
  else:
    startangle = robot.angle()

    if direction == 'in':
      RightMotor.run(-speed)
    elif direction == 'out':
      RightMotor.run(speed)

    color = None
    colorList = []
    while color not in acceptable and abs(startangle - robot.angle()) < 35:
      outColor = rgbtocolor(ColorA.rgb())
      if outColor in acceptable:
        colorList.append(outColor)
        if len(colorList) >= 10:
          color = mode(colorList)

    if color == None and len(colorList) > 0:
      color = mode(colorList)

    robot.stop()

    if direction == 'in':
      RightMotor.run(speed / 2)
      while robot.angle() > startangle:
        pass
    elif direction == 'out':
      RightMotor.run(-speed / 2)
      while robot.angle() < startangle:
        pass

    robot.stop()
    return color

def mode(List):
  counter = 0
  num = List[0]
    
  for i in List:
    curr_frequency = List.count(i)
    if(curr_frequency> counter):
      counter = curr_frequency
      num = i
  return num

starttime = time.time()
main()
robot.stop()
print(time.time() - starttime)