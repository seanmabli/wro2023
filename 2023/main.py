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
  containerColors = [3, 3, 3, 3] # 1 = green, 2 = blue, 3 = not scaned / error
  containerPositions = [225, 130, 25, -85]
  largeBoatPositions = [170, 110, 0, -40] # largeBoatPositions[2] is not accurate because it is never used
  smallBoatPositions = [80, -10]
  largeBoatAvailable = [True, True, False, True]
  smallBoatAvailable = [True, True]
  whitePosition = 805
  markingBlocks = [3, 3]

  # ** START **
  straight(115)
  sweep(sensor=LeftColor, direction="left")
  lineFollowingDistance(distance=140, sensor=LeftColor, sideofsensor='out', speed=200)
  markingBlocks[0] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=15, whitethreshold=45, speed=200)
  lineFollowingDistance(distance=10, sensor=LeftColor, sideofsensor='out', speed=200)
  markingBlocks[1] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
  print("markingBlocks:", markingBlocks)
  markingBlocks = fixWithRandom(markingBlocks)
  print("markingBlocks + random if poorly scaned:", markingBlocks)
  lineFollowingDistance(distance=40, sensor=LeftColor, sideofsensor='out', speed=300)
  straight(130, speed=300)
  straight(-130, deceleration=True)
  durn(turn=-110, type="tank")
  durn(turn=115, circleradius=-70, type="circle", speed=400)
  durn(turn=-160, type="tank")
  straight(-150)
  boatGrab(movement="close")

  # ** MOVE BOAT TO CONTAINER PICKUP **
  straight(80)
  durn(turn=-160, type="tank", speed=100)
  straight(150)
  straightUntilBlack(direction=1, speed=200)
  straight(10)
  durn(turn=120, type="tank", speed=300)
  sweep(sensor=RightColor, direction="left", whiteFirst=True)
  lineFollowingBlack(sensor=RightColor, sideofsensor='in', blackthreshold=10, whitethreshold=45, speed=200, proportion=0.6)
  lineFollowingDistance(distance=90, sensor=RightColor, sideofsensor='in', speed=200)
  durn(turn=120, type="tank", speed=300)
  sweep(sensor=LeftColor, direction="left", whiteFirst=True)
  lineFollowingDistance(distance=30, sensor=LeftColor, sideofsensor='out', speed=100)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=15, speed=100)
  lineFollowingDistance(distance=60, sensor=LeftColor, sideofsensor='out', speed=100)
  boatGrab(movement="open")
  durn(turn=-157, type="pivot", speed=300)

  # ** CONTAINER SCAN **
  straight(-125, deceleration=True)
  containerColors[0] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
  straight(-105, deceleration=True)
  containerColors[1] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
  straightUntilBlack(direction=-1, speed=200)
  position = 0
  if calculateColors(containerColors)[1] == False:
    position += straight(-60, deceleration=True)
    containerColors[2] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
    if calculateColors(containerColors)[1] == False:
      position += straight(-105, deceleration=True)
      containerColors[3] = colorScan(acceptable=[1, 2], direction="out", errorNum=3, speed=300)
    position = calibratePos(position)
  containerColors = calculateColors(containerColors)[0]
  print("real scan:", containerColors)
  containerColors = calculateColors(containerColors, replaceRandomly=True)[0]
  print("real scan + random if poorly scaned:", containerColors)

  # ** CONTAINER PICKUP **
  _thread.start_new_thread(boatGrab, ("open", 0.2))
  for i in range(2):
    newPosition, containerIndex = closestContainer(position, containerPositions, containerColors, markingBlocks)
    position += straight(newPosition - position, deceleration=True)
    if containerColors[containerIndex] == 1: # green
      armGrab("up->down")
      armGrab("down->midup")
      newPosition, boatIndex = closestBoat(position, largeBoatPositions, largeBoatAvailable)
      position += straight(newPosition - position, deceleration=True)
      boatGrab(movement="close", hold=True, speed=160) # if stuck on ramp or overshooting adjust this vaue
      armGrab("midup->up")
      time.sleep(0.3)
    else: # blue
      armGrab("up->down")
      armGrab("down->midup", speed=200)
      newPosition, boatIndex = closestBoat(position, largeBoatPositions, largeBoatAvailable)
      position += straight(newPosition - position, deceleration=True)
      boatGrab(movement="close", hold=True, speed=250)
      armGrab("midup->up")
      time.sleep(0.3)
    _thread.start_new_thread(boatGrab, ("open", 1.3))
    markingBlocks.remove(containerColors[containerIndex])
    containerColors[containerIndex] = 3
    largeBoatAvailable[boatIndex] = False
    if i == 0:
      position = calibratePos(position)

  # ** WHITE CONTAINER PICKUP **
  position += straight(700 - position)
  position = calibratePos(position)
  position += straight(whitePosition - position, deceleration=True)
  armGrab("up->down")
  armGrab("down->midup")
  position += straight(50 - position)
  position = calibratePos(position)
  newPosition, boatIndex = closestBoat(position, largeBoatPositions, largeBoatAvailable)
  position += straight(newPosition - position, deceleration=True)
  largeBoatAvailable[boatIndex] = False
  boatGrab(movement="close", hold=True, speed=160)
  armGrab("midup->up")
  _thread.start_new_thread(boatGrab, ("open", 1.3))

  # ** MOVE LARGE BOAT OUT TO SEA **
  position += straight(265 - position, deceleration=True)
  durn(turn=-180, fb="backward", type="pivot", speed=400)
  straight(-40, deceleration=True)
  boatGrab(movement="close")
  straight(-300)
  straightUntilBlack(direction=-1, speed=200, colorSensor=RightColor)
  straight(160)
  durn(turn=-150, type="tank", speed=200)
  sweep(sensor=LeftColor, direction="right", whiteFirst=True, speed=100, threshold=(10, 15))
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400, proportion=0.6)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='in', blackthreshold=10, whitethreshold=45, speed=400)
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='in', blackthreshold=10, whitethreshold=45, speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='in', blackthreshold=10, whitethreshold=45, speed=400) # change to distance if not detecint cloud
  straightUntilBlack(direction=-1, speed=150)
  straight(90)
  durn(turn=200, type="tank", speed=100)
  straight(-500)
  _thread.start_new_thread(boatGrab, ("open",))

  # ** PICKUP SMALL BOAT **
  lineFollowingDistance(distance=370, sensor=LeftColor, sideofsensor='out', speed=400)
  durn(turn=120, type="tank", speed=200)
  sweep(sensor=LeftColor, direction="left", whiteFirst=True, speed=100)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=400)
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='out', speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=400)
  straight(140)
  durn(turn=-160, type="tank", speed=200)
  straight(-420)
  boatGrab(movement="close")
  time.sleep(2)
  sturn(rl="left", fb="forward", turn=60)
  time.sleep(2)
  sweep(sensor=LeftColor, direction="left", whiteFirst=True, speed=100)
  time.sleep(2)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=10, whitethreshold=45, speed=400)
  lineFollowingDistance(distance=340, sensor=LeftColor, sideofsensor='out', speed=200)

  lineFollowingBlack(sensor=LeftColor, sideofsensor='out', blackthreshold=15, whitethreshold=45, speed=100)
  lineFollowingDistance(distance=60, sensor=LeftColor, sideofsensor='out', speed=100)
  boatGrab(movement="open")
  durn(turn=-157, type="pivot", speed=300)

  # ** CONTAINER PICKUP **
  _thread.start_new_thread(boatGrab, ("open", 0.2))
  position = calibratePos(position)
  for i in range(2):
    newPosition, containerIndex = closestContainer(position, containerPositions, containerColors, markingBlocks, useMarkingBlocks=False)
    position += straight(newPosition - position, deceleration=True)
    if containerColors[containerIndex] == 1: # green
      armGrab("up->down")
      armGrab("down->midup")
      newPosition, boatIndex = closestBoat(position, smallBoatPositions, smallBoatAvailable)
      position += straight(newPosition - position, deceleration=True)
      boatGrab(movement="close", hold=True, speed=160) # if stuck on ramp or overshooting adjust this vaue
      armGrab("midup->up")
      time.sleep(0.3)
    else: # blue
      armGrab("up->down")
      armGrab("down->midup", speed=200)
      newPosition, boatIndex = closestBoat(position, smallBoatPositions, smallBoatAvailable)
      position += straight(newPosition - position, deceleration=True)
      boatGrab(movement="close", hold=True, speed=250)
      armGrab("midup->up")
      time.sleep(0.3)
    _thread.start_new_thread(boatGrab, ("open", 1.3))
    containerColors[containerIndex] = 3
    smallBoatAvailable[boatIndex] = False

  # ** SMALL BOAT OUT TO SEA **
  position += straight(265 - position, deceleration=True)
  durn(turn=-180, fb="backward", type="pivot", speed=400)
  straight(-40, deceleration=True)
  boatGrab(movement="close")
  straight(-300)
  straightUntilBlack(direction=-1, speed=200, colorSensor=RightColor)
  straight(160)
  durn(turn=-150, type="tank", speed=200)
  sweep(sensor=LeftColor, direction="right", whiteFirst=True, speed=100, threshold=(10, 15))
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400, proportion=0.6)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='in', blackthreshold=10, whitethreshold=45, speed=400)
  lineFollowingDistance(distance=100, sensor=LeftColor, sideofsensor='in', speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='in', blackthreshold=10, whitethreshold=45, speed=400)
  lineFollowingBlack(sensor=LeftColor, sideofsensor='in', blackthreshold=10, whitethreshold=45, speed=400) # change to distance if not detecint cloud
  _thread.start_new_thread(boatGrab, ("open",))
  durn(turn=165, type="tank")
  straight(500)

@timefunc
def fixWithRandom(colorScan):
  for i in range(len(colorScan)):
    if colorScan[i] == 3:
      colorScan[i] = random.randint(1, 2)
  return colorScan

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

@timefunc
def closestBoat(position, boatPositions, boatAvailable):
  distances = []
  for i, container in enumerate(boatPositions):
    if boatAvailable[i]:
      distances.append((abs(container - position), i))
  distances.sort(key=lambda x: x[0])
  return boatPositions[distances[0][1]], distances[0][1]

@timefunc
def closestContainer(position, containerPositions, containerColors, markingBlocks, useMarkingBlocks=True):
  distances = []
  for i, container in enumerate(containerPositions):
    if containerColors[i] in markingBlocks or (not useMarkingBlocks and containerColors[i] != 3):
      distances.append((abs(container - position), i))
  distances.sort(key=lambda x: x[0])
  return containerPositions[distances[0][1]], distances[0][1]

@timefunc
def calculateColors(colorList, replaceRandomly=False):
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
    if replaceRandomly:
      start = colorList.copy()
      for i in range(4):
        if colorList[i] == 3:
          colorList[i] = random.randint(1, 2)
      while colorList.count(2) != colorList.count(1):
        colorList = start.copy()
        for i in range(4):
          if colorList[i] == 3:
            colorList[i] = random.randint(1, 2)
    return (colorList, False)

def rgbtocolor(rgb): # None = 0, green = 1, blue = 2
  if sum(rgb) < 4:
    return 0
  elif rgb[2] > rgb[0] + rgb[1]:
    return 2
  elif rgb[1] > (rgb[0] + rgb[2]) * 0.75 or (rgb[2] / (rgb[1] + 1)) > 0.6:
    return 1
  else:
    return 0

@timefunc
def straight(distance, speed=400, deceleration=False):
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

@timefunc
def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=50, startdistance=0, kp=0.25, ki=0, kd=0.5, speed=[], estdistance=0, blackthreshold=10, whitethreshold=None): # wait distance is the # of mm after a black it waits until continue detecting blacks
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
  derivative = 0
  count = 0

  lastdistance = abs(robot.distance())
  lastdistancechange = 0
  num = 0
  white = 0
  oppositeColor = LeftColor if sensor == RightColor else RightColor
  while count < blacks:
    if whitethreshold == None:
      if oppositeColor.reflection() < blackthreshold and lastdistance + waitdistance < abs(robot.distance()) and startdistance < abs(robot.distance()):
        count += 1
        lastdistance = abs(robot.distance())
    else:
      if oppositeColor.reflection() > whitethreshold:
        white += 1
        quit()
      if oppositeColor.reflection() < blackthreshold and lastdistance + waitdistance < abs(robot.distance()) and startdistance < abs(robot.distance()) and white > 0:
        count += 1
        white = 0
        lastdistance = abs(robot.distance())

    if abs(lastdistancechange - robot.distance()) > estdistance / len(speed) and num < len(speed) - 1:
      lastdistancechange = robot.distance()
      num += 1

    if sideofsensor == 'out':
      error = target - sensor.reflection()
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    derivative = error - lasterror
    turn = kp * error
    lasterror = error

    robot.drive(speed[num], turn)

  robot.stop()

@timefunc
def lineFollowingBlack(sensor, sideofsensor, blacks=1, proportion=0.4, speed=[], estdistance=0, blackthreshold=10, whitethreshold=None):
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
  count = 0

  lastdistance = abs(robot.distance())
  lastdistancechange = 0
  num = 0
  white = 0
  oppositeColor = LeftColor if sensor == RightColor else RightColor
  while count < blacks:
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
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    turn = proportion * error

    robot.drive(speed[num], turn)

  robot.stop()

@timefunc
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
  derivative = 0

  lastdistance = 0
  num = 0
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < abs(distance):
    if abs(lastdistance - robot.distance()) > distance / len(speed):
      lastdistance = robot.distance()
      num += 1
    if sideofsensor == 'out':
      error = target - sensor.reflection()
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    derivative = error - lasterror
    turn = kp * error
    lasterror = error

    if one:
      robot.drive(speed[0], turn)
    else:
      robot.drive(speed[num], turn)

  robot.stop()

@timefunc
def lineFollowingDistance(distance, sensor=RightColor, sideofsensor='in', speed=[], proportion=0.4):
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

  lastdistance = 0
  num = 0
  startdistance = robot.distance()
  while abs(robot.distance() - startdistance) < abs(distance):
    if abs(lastdistance - robot.distance()) > distance / len(speed):
      lastdistance = robot.distance()
      num += 1
    if sideofsensor == 'out':
      error = target - sensor.reflection()
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    turn = proportion * error

    if one:
      robot.drive(speed[0], turn)
    else:
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
  if movement not in ['up->down', 'down->up', 'down->mid', 'mid->up', 'up->mid', 'mid->down', 'down->midup', 'midup->up', 'midup->down', 'mid->midup']:
    raise Exception('movement must be "up->down", "down->up", "down->mid", "mid->up", "up->mid", or "mid->down"')
  if movement == 'up->down':
    time.sleep(0.001)
    if speed == None:
      speed = 400
    ArmMotor.run_angle(speed, 220)
    ArmMotor.hold()
  elif movement == 'midup->down':
    time.sleep(0.001)
    if speed == None:
      speed = 400
    ArmMotor.run_angle(speed, 180)
    ArmMotor.hold()
  elif movement == 'down->midup':
    if speed == None:
      speed = 400
    ArmMotor.run(-speed)
    time.sleep(0.6 * (400 / speed))
    ArmMotor.run_angle(400, 40)
    ArmMotor.hold()
  elif movement == 'midup->up':
    ArmMotor.run(-400)
    time.sleep(0.15)
    ArmMotor.stop()
  elif movement == 'down->mid':
    if speed == None:
      speed = 400
    ArmMotor.run(-speed)
    time.sleep(0.2 * (400 / speed))
    ArmMotor.hold()
  elif movement == 'mid->up':
    if speed == None:
      speed = 400
    ArmMotor.run(-speed)
    time.sleep(0.4 * (400 / speed))
    ArmMotor.run_angle(400, 10)
    ArmMotor.run(-400)
    time.sleep(0.15)
    ArmMotor.hold()
  elif movement == 'mid->midup':
    ArmMotor.run(-speed)
    time.sleep(0.3 * (400 / speed))
    ArmMotor.run_angle(400, 40)
    ArmMotor.hold()

@timefunc
def sweep(sensor, direction, speed=100, whiteFirst=False, threshold=(0, 10)):
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

    if color == None:
      if len(colorList) > 0:
        color = mode(colorList)
      else:
        color = errorNum

    robot.stop()

    if direction == 'in':
      RightMotor.run(speed / 2)
      while robot.angle() * outTurnIncrease > startangle:
        pass
    elif direction == 'out':
      RightMotor.run(-speed / 2)
      while robot.angle() * outTurnIncrease < startangle:
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
for i in times:
  print("total " + i + " time:", round(times[i], 2))
print("\ntotal time:", round(time.time() - starttime, 2))