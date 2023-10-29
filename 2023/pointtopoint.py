#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
import time, _thread, random


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

times = {}

def timefunc(func):
  def inner(*args, **kwargs):
    timeA = time.time()
    out = func(*args, **kwargs)
    # print(func.__name__ + " time:", time.time() - timeA)
    if func.__name__ in times:
      times[func.__name__] += time.time() - timeA
    else:
      times[func.__name__] = time.time() - timeA
    return out
  return inner

def main():
  '''
  National Colors:
  Black - 9
  Dark Blue - 13
  Light Blue - 12
  White - 62
  Gray - 42
  '''
  containerColors = [3, 3, 3, 3] # 1 = green, 2 = blue, 3 = not scaned / error
  containerPositions = [225, 115, 25, -85]
  largeBoatPositions = [185, 105, 0, -40] # largeBoatPositions[2] is not accurate because it is never used
  smallBoatPositions = [55, -20]
  largeBoatAvailable = [True, True, False, True]
  smallBoatAvailable = [True, True]
  whitePosition = 800
  markingBlocks = [3, 3]
  greenArmMidSpeed = 300
  greenArmUpSpeed = 200
  greenArmDownSpeed = 350
  blueArmMidSpeed = 150
  blueArmUpSpeed = 100
  blueArmDownSpeed = 375

  # ** Base to Intersection A (Includes Marking Block Scan) **
  '''
  straight(115)
  sweep(sensor=LeftColor, direction="left")
  lineFollowingDistance(distance=140, sensor=LeftColor, sideofsensor='out', speed=200, proportion=0.8)
  time.sleep(0.1)
  markingBlocks[0] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=15, whitethreshold=45, speed=200)
  lineFollowingDistance(distance=10, sensor=LeftColor, sideofsensor='out', speed=200, proportion=0.2)
  time.sleep(0.1)
  markingBlocks[1] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
  print("markingBlocks:", markingBlocks)
  markingBlocks = fixWithRandom(markingBlocks)
  print("markingBlocks + random if poorly scaned:", markingBlocks)
  '''

  # ** Intersection A to Fuel **
  '''
  straight(500)
  '''

  # ** Intersection A (from Base) to Intersection B **
  '''
  straight(170)
  durn(turn=-180, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True, threshold=(0, 10))
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=300, proportion=0.8)
  '''

  # ** Intersection B (from Intersection A) to Intersection C (No Sweep) **
  '''
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=300)
  '''

  # ** Intersection C (from Intersection B) to Intersection D (No Sweep) **
  '''
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='out', speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=300)
  '''

  # ** Intersection B (from Intersection A) to Small Ship **
  '''
  lineFollowingDistance(distance=180, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=-180, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=225, sensor=LeftColor, sideofsensor='out', speed=300, proportion=0.8)
  '''

  # ** Intersection B (from Intersection A) to Green/Blue Containers or Intersection C (from Intersection B) to White Containers **
  '''
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=160, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=300, sensor=LeftColor, sideofsensor='out', speed=300, proportion=0.8)
  '''
  
  # ** Intersection C (from Intersection B) to Crane A **
  '''
  lineFollowingDistance(distance=170, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=-180, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=350, sensor=LeftColor, sideofsensor='out', speed=300, proportion=0.8)
  '''

  # ** Intersection D (from Intersection B) to Crane B **
  '''
  straightUntilBlack(direction=-1, speed=200)
  straight(170)
  durn(turn=-180, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=350, sensor=LeftColor, sideofsensor='out', speed=300, proportion=0.8)
  '''

  # ** Intersection D (from Intersection C) to Open Ocean Base **
  '''
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=160, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=250, sensor=LeftColor, sideofsensor='out', speed=300, proportion=0.8)
  straight(200)
  '''

  # ** Open Ocean Base to Intersection D **
  '''
  straight(115)
  sweep(sensor=LeftColor, direction="left")
  lineFollowingDistance(distance=140, sensor=LeftColor, sideofsensor='out', speed=200, proportion=0.8)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=15, whitethreshold=45, speed=200)
  '''

  # ** Intersection D (from Open Ocean Base, No Sweep) to Crane B **
  '''
  lineFollowingDistance(distance=480, sensor=LeftColor, sideofsensor='out')
  '''

  # Intersection D (from Open Ocean Base) to Intersection C **
  '''
  straight(130)
  durn(turn=-155, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True, threshold=(0, 10))
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=300, proportion=0.8)
  '''

  # ** Intersection C (from Intersection D) to Crane A **
  '''
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=160, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=350, sensor=LeftColor, sideofsensor='out', speed=300)
  '''

  # ** Intersection C (from Intersection D) to White Containers **
  '''
  lineFollowingDistance(distance=180, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=-160, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=380, sensor=LeftColor, sideofsensor='out', speed=300, proportion=0.8)
  '''

  # ** Intersection C (from Intersection D) to Intersection B **
  '''
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='out', speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=12, whitethreshold=45, speed=300)
  '''

  # ** Intersection B (from Intersection C) to Green/Blue Containers **
  '''
  lineFollowingDistance(distance=160, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=-175, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=280, sensor=LeftColor, sideofsensor='out', speed=300, proportion=0.8)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=15, speed=100)
  lineFollowingDistance(distance=55, sensor=LeftColor, sideofsensor='out', speed=100)
  '''

  # ** Intersection B (from Intersection C) to Small Ship **
  '''
  lineFollowingDistance(distance=95, sensor=LeftColor, sideofsensor='in', speed=400)
  durn(turn=150, type="tank")
  sweep(sensor=LeftColor, direction="left", whiteFirst=True, speed=200)
  lineFollowingDistance(distance=150, sensor=LeftColor, sideofsensor='out', speed=300)
  '''
  
  # ** Intersection B (from Intersection C) to Intersection A **
  '''
  sturn(rl="left", fb="forward", turn=70)
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=200, sensor=LeftColor, sideofsensor='out', speed=400, proportion=0.8)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=300)
  '''

  # ** Intersection A (from Intersection B) to Base **
  '''
  straight(100)
  durn(turn=165, type="tank")
  straight(260)
  '''

  # ** Intersection A (from Intersection B) to Fuel **
  '''
  straight(120)
  durn(turn=-165, type="tank", speed=200)
  straight(480)
  '''

def fixWithRandom(scan):
  for i in range(len(scan)):
    if scan[i] == 3 and scan[0 if i == 1 else 1] == 1:
      if random.random() >= 0.66:
        scan[i] = 2
      else:
        scan[i] = 1
    elif scan[i] == 3 and scan[0 if i == 1 else 1] == 2:
      if random.random() >= 0.66:
        scan[i] = 1
      else:
        scan[i] = 2
  return scan

@timefunc
def calibratePos(position):
  if position > 600:
    straightUntilBlack(direction=1, speed=100)
    return 770
  elif position > 0:
    straightUntilBlack(direction=-1, speed=200)
    return 0
  else:
    straightUntilBlack(direction=1, speed=200)
    return -15

def closestBoatDropoff(position, boatPositions, boatAvailable):
  distances = []
  for i, container in enumerate(boatPositions):
    if boatAvailable[i]:
      distances.append((abs(container - position), i))
  distances.sort(key=lambda x: x[0])
  return boatPositions[distances[0][1]], distances[0][1]

def accurateSmallBoatDropoff(position, boatPositions, boatAvailable, containerColor):
  if containerColor == 1 and boatAvailable[0]:
    return boatPositions[0], 0
  elif containerColor == 2 and boatAvailable[1]:
    return boatPositions[1], 1
  else:
    return closestBoatDropoff(position, boatPositions, boatAvailable)

def closestContainerPickup(position, containerPositions, containerColors, markingBlocks, useMarkingBlocks=True):
  distances = []
  for i, container in enumerate(containerPositions):
    if containerColors[i] in markingBlocks or (not useMarkingBlocks and containerColors[i] != 3):
      distances.append((abs(container - position), i))
  distances.sort(key=lambda x: x[0])
  return containerPositions[distances[0][1]], distances[0][1]

def calculateColors(colorList, markingBlocks):
  if colorList.count(3) == 2 and (colorList.count(1) == 2 or colorList.count(2) == 2):
    colorList = list(map(lambda x: (1 if colorList.count(2) == 2 else 2) if x == 3 else x, colorList))
    return (colorList, True)
  elif colorList.count(3) == 1:
    if colorList.count(1) == 2:
      colorList = list(map(lambda x: 2 if x == 3 else x, colorList))
    else:
      colorList = list(map(lambda x: 1 if x == 3 else x, colorList))
    return (colorList, True)
  elif colorList.count(3) == 0:
    return (colorList, True)
  else:
    return (colorList, False)

def replaceWithRandom(colorList):
  start = colorList.copy()
  for i in range(4):
    if colorList[i] == 3:
      colorList[i] = random.randint(1, 2)
  while colorList.count(2) != colorList.count(1):
    colorList = start.copy()
    for i in range(4):
      if colorList[i] == 3:
        colorList[i] = random.randint(1, 2)
  return colorList

def rgbtocolor(rgb): # None = 0, green = 1, blue = 2
  if sum(rgb) < 2:
    return 0
  elif rgb[2] > (rgb[0] + rgb[1]) * 1.25:
    return 2
  elif rgb[1] > (rgb[0] + rgb[2]) * 0.75 or (rgb[2] / (rgb[1] + 1)) > 0.6:
    return 1
  else:
    return 0

@timefunc
def straight(distance, speed=400, deceleration=False):
  if speed != "dc":
    if distance < 0:
      speed *= -1
    startdistance = robot.distance()
    while abs(robot.distance() - startdistance) < abs(distance):
      if deceleration and abs(distance) - abs(robot.distance() - startdistance) < 50:
        robot.drive(speed / 4, 0)
      elif deceleration and abs(distance) - abs(robot.distance() - startdistance) < 100:
        robot.drive(speed / 2, 0)
      else:
        robot.drive(speed, 0)
    robot.stop()
  else:
    startdistance = robot.distance()
    while abs(robot.distance() - startdistance) < abs(distance):
      RightMotor.dc(100 * (1 if distance > 0 else -1))
      LeftMotor.dc(100 * (1 if distance > 0 else -1))
    RightMotor.hold()
    LeftMotor.hold()
  return distance

@timefunc
def straightUntilBlack(direction=1, speed=400, angled=False, colorSensor=None):
  if colorSensor == None:
    if not angled:
      while (RightColor.reflection() + LeftColor.reflection()) / 2 < 45:
        robot.drive(speed * direction, 0)
      while (RightColor.reflection() + LeftColor.reflection()) / 2 > 15:
        robot.drive(speed * direction / 2, 0)
      robot.stop()
    else:
      white = [False, False]
      line = [False, False]
      percentage = 1
      while not (white[0] and white[1] and line[0], line[1]):
        if (RightColor.reflection() + LeftColor.reflection()) / 2 > 45:
          white[0] = True
        if (RightColor.reflection() + LeftColor.reflection()) / 2 > 45:
          white[1] = True
        if (white[0] and white[1]):
          percentage = 0.5
        if (RightColor.reflection() + LeftColor.reflection()) / 2 < 15:
          line[0] = True
        if (RightColor.reflection() + LeftColor.reflection()) / 2 < 15:
          line[1] = True
        robot.drive(speed * direction * percentage, 0)
  else:
    while colorSensor.reflection() < 45:
      robot.drive(speed * direction, 0)
    while colorSensor.reflection() > 15:
      robot.drive(speed * direction / 2, 0)
    robot.stop()


@timefunc
def durn(turn, circleradius=30, type='tank', fb='forward', speed=400, deceleration=False): # durn = degree turn
  if type not in ['tank', 'pivot', 'circle']:
    raise Exception('type must be "tank" or "pivot" or "circle"')
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')
  
  if fb == 'backward':
    speed *= -1

  startangle = robot.angle()
  if type == 'tank':
    if turn < 0:
      RightMotor.run(-speed)
      LeftMotor.run(speed)
    else:
      RightMotor.run(speed)
      LeftMotor.run(-speed)
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

  if type == 'circle':
    robot.stop()
  elif type == 'tank' or type == 'pivot':
    LeftMotor.hold()
    RightMotor.hold()

@timefunc
def lineFollowingBlack(sensor, sideofsensor, blacks=1, proportion=0.4, inprop=None, outprop=None, speed=[], estdistance=0, blackthreshold=10, whitethreshold=None):
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
    speed = [150]
    speedType = "int"
  elif type(speed) == int:
    speed = [speed]
    speedType = "int"
  elif speed == "dc":
    speedType = "dc"
  else:
    speed = list(range(speed[0], speed[1]))
    speedType = "list"

  inprop = inprop if inprop != None else proportion
  outprop = outprop if outprop != None else proportion

  target = (8 + 76) / 2 # Black  = 8, White = 76
  count = 0

  lastdistance = abs(robot.distance())
  lastdistancechange = 0
  num = 0
  white = 0
  oppositeColor = LeftColor if sensor == RightColor else RightColor
  while count < blacks:
    print(oppositeColor.reflection())
    if whitethreshold == None:
      if oppositeColor.reflection() < blackthreshold:
        count += 1
        lastdistance = abs(robot.distance())
    else:
      if oppositeColor.reflection() > whitethreshold:
        white += 1
      if oppositeColor.reflection() < blackthreshold and white > 0:
        count += 1
        white = 0
        lastdistance = abs(robot.distance())
    if abs(lastdistancechange - robot.distance()) > estdistance / len(speed) and num < len(speed) - 1:
      lastdistancechange = robot.distance()
      num += 1

    if sideofsensor == 'out':
      error = target - sensor.reflection()
      if error < 0:
        turn = inprop * error
      else:
        turn = outprop * error
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
      if error < 0:
        turn = outprop * error
      else:
        turn = inprop * error

    if speedType == "dc":
      if turn < 0:
        RightMotor.dc(100)
        LeftMotor.dc(100 - turn)
      else:
        RightMotor.dc(100 - turn)
        LeftMotor.dc(100)
    elif speedType == "int":
      robot.drive(speed[0], turn)
    elif speedType == "list":
      robot.drive(speed[num], turn)

  robot.stop()

@timefunc
def lineFollowingDistance(distance, sensor=RightColor, sideofsensor='in', speed=[], proportion=0.4, inprop=None, outprop=None):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  
  if sensor == RightColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

  if speed == []:
    speed = [400]
    speedType = "int"
  elif type(speed) == int:
    speed = [speed]
    speedType = "int"
  elif speed == "dc":
    speedType = "dc"
  else:
    speed = list(range(speed[0], speed[1]))
    speedType = "list"

  inprop = inprop if inprop != None else proportion
  outprop = outprop if outprop != None else proportion

  target = (8 + 76) / 2 # Black  = 8, White = 76

  lastdistance = 0
  num = 0
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < abs(distance):
    if speed != "dc" and abs(lastdistance - robot.distance()) > distance / len(speed):
      lastdistance = robot.distance()
      num += 1
    if sideofsensor == 'out':
      error = target - sensor.reflection()
      if error < 0:
        turn = inprop * error
      else:
        turn = outprop * error
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
      if error < 0:
        turn = outprop * error
      else:
        turn = inprop * error

    if speedType == "dc":
      if turn < 0:
        RightMotor.dc(100)
        LeftMotor.dc(100 - turn)
      else:
        RightMotor.dc(100 - turn)
        LeftMotor.dc(100)
    elif speedType == "int":
      robot.drive(speed[0], turn)
    elif speedType == "list":
      robot.drive(speed[num], turn)

  robot.stop()

@timefunc
def boatGrab(movement="open", percentage=1, hold=False, stop=False, speed=400):
  if movement == "open":
    BoatMotor.stop()
    BoatMotor.run_angle(-speed, 95 * percentage)
    time.sleep(0.01)
  elif movement == "close":
    BoatMotor.run(speed)
    time.sleep(0.3 * percentage * (400 / speed))
    if hold:
      BoatMotor.hold()
    if stop:
      BoatMotor.stop()

@timefunc
def armGrab(movement, speed=None):
  if movement not in ['up->down', 'down->mid', 'mid->up']:
    raise Exception('movement must be "up->down", "down->mid", or "mid->up"')
  if movement == 'up->down':
    time.sleep(0.001)
    if speed == None:
      speed = 400
    ArmMotor.run_angle(speed, 225)
    ArmMotor.hold()
  elif movement == 'down->mid':
    if speed == None:
      speed = 400
    ArmMotor.run(-speed)
    time.sleep(0.23 * (400 / speed))
    ArmMotor.hold()
  elif movement == 'mid->up':
    if speed == None:
      speed = 400
    ArmMotor.run(-speed)
    time.sleep(0.29 * (400 / speed))
    ArmMotor.run_angle(400, 18)
    ArmMotor.run(-400)
    time.sleep(0.2)
    ArmMotor.hold()

@timefunc
def bigArmGrab(movement="open", percentage=1, hold=False, stop=False, speed=400):
  if movement == "open":
    BoatMotor.stop()
    BoatMotor.run_angle(-speed, 80 * percentage)
    time.sleep(0.01)
  elif movement == "close":
    BoatMotor.run(speed)
    time.sleep(0.15 * percentage * (400 / speed))
    if hold:
      BoatMotor.hold()
    if stop:
      BoatMotor.stop()

@timefunc
def liftGrab(movement="down", percentage=1, hold=False, stop=False, speed=400):
  if movement == "down":
    BoatMotor.stop()
    BoatMotor.run_angle(-speed, 120 * percentage)
    time.sleep(0.01)
  elif movement == "up":
    BoatMotor.run(speed)
    time.sleep(0.05 * percentage * (400 / speed))
    if hold:
      BoatMotor.hold()
    if stop:
      BoatMotor.stop()
      
@timefunc
def sweep(sensor, direction, speed=100, whiteFirst=False, threshold=(0, 10), reverse=False):
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
  
  if reverse:
    while threshold[0] > sensor.reflection() or sensor.reflection() > threshold[1]:
      info.append([sensor.reflection(), robot.angle()])

    if whiteFirst:
      while sensor.reflection() < 45:
        info.append([sensor.reflection(), robot.angle()])
  else:
    if whiteFirst:
      while sensor.reflection() < 45:
        info.append([sensor.reflection(), robot.angle()])

    while threshold[0] > sensor.reflection() or sensor.reflection() > threshold[1]:
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

@timefunc
def sturn(rl, fb, turn, type='pivot', drive=0, turnSpeed=400): # rl = right-left, fb = forward-backward, turn = turn degrees(posotive), drive = drive between turns(positive)
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

@timefunc
def colorScan(acceptable, direction, errorNum, outTurnIncrease=1, speed=200):
  outColor, outRGB = rgbtocolor(ColorA.rgb()), ColorA.rgb()
  # print(outRGB, outColor)
  if outColor in acceptable:
    return outColor
  else:
    startangle = robot.angle()

    if direction == 'in':
      RightMotor.run(-speed)
    elif direction == 'out':
      RightMotor.run(speed)

    color = None
    colorList = []
    while color not in acceptable and abs(startangle - robot.angle()) < 40:
      outColor = rgbtocolor(ColorA.rgb())
      # print(ColorA.rgb(), rgbtocolor(ColorA.rgb()))
      if outColor in acceptable:
        colorList.append(outColor)
        if len(colorList) >= 10:
          color = mode(colorList)

    if color == None:
      if len(colorList) > 0:
        color = mode(colorList)
      else:
        color = errorNum

    RightMotor.stop()

    if direction == 'in':
      RightMotor.run(speed)
      while robot.angle() > startangle * outTurnIncrease:
        pass
    elif direction == 'out':
      RightMotor.run(-speed)
      while robot.angle() < startangle * outTurnIncrease:
        pass

    RightMotor.stop()
    return color

@timefunc
def turnColorScan(acceptable, direction, errorNum, speed=200):
  outColor, outRGB = rgbtocolor(ColorA.rgb()), ColorA.rgb()
  if outColor in acceptable:
    return outColor
  else:
    startAngle = robot.angle()

    if direction == 'backward':
      RightMotor.run(-speed)
    elif direction == 'forward':
      RightMotor.run(speed)

    color = None
    colorList = []
    while color not in acceptable and abs(startAngle - robot.angle()) < 40:
      outColor = rgbtocolor(ColorA.rgb())
      if outColor in acceptable:
        colorList.append(outColor)
        if len(colorList) >= 5:
          color = mode(colorList)

    if color == None:
      if len(colorList) > 0:
        color = mode(colorList)
      else:
        color = errorNum
  
    RightMotor.stop()

    if direction == 'backward':
      RightMotor.run(speed)
      while robot.angle() > startAngle:
        pass
    elif direction == 'forward':
      RightMotor.run(-speed)
      while robot.angle() < startAngle:
        pass

    RightMotor.stop()
    return color

def mode(inputList):
  counter = 0
  num = inputList[0]
  for i in inputList:
    if inputList.count(i) > counter:
      counter = inputList.count(i)
      num = i
  return num

ev3 = EV3Brick()
ev3.screen.clear()
while Button.CENTER not in ev3.buttons.pressed():
  pass

starttime = time.time()
main()
robot.stop()
for i in times:
  print("total " + i + " time:", round(times[i], 2))
print("\ntotal time:", round(round(round(time.time() - starttime, 2), 2), 2))