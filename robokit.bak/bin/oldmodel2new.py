#!/usr/bin/env python3

import sys
import re
import array
import math
import os
import json as js

if(2 == len(sys.argv)):
    old_file_name = sys.argv[1]
else:
    old_file_name = "robot.model"

file_object = open(old_file_name, encoding='UTF-8')
new_file_object = open("new_robot.model", encoding='UTF-8', mode='w+')

temp = ""
array_new = {}

while True:
    text = file_object.readline()
    if not text: break

    m = re.match("(.*)//(.*)", text)
    if m:
        if m.group(1):
            temp = temp + m.group(1) + "\r\n"
    else:
        if len(text) > 1:
            temp = temp + text

array_old = js.loads(temp, encoding="utf-8")

array_new["model"] = "Seer-2017"

array_new["chassis"] = {}
array_new["chassis"]["mode_param"] = {}
array_new["chassis"]["shape"]           = 1
array_new["chassis"]["radius"]          = array_old["chassis"]["width"]
array_new["chassis"]["width"]           = array_old["chassis"]["width"]
array_new["chassis"]["head"]            = array_old["chassis"]["head"]
array_new["chassis"]["tail"]            = array_old["chassis"]["tail"]
array_new["chassis"]["mode"]            = array_old["chassis"]["mode"]
array_new["chassis"]["mode_param"]["type"]              = array_old["chassis"]["mode"] * 10
if (1 == array_old["chassis"]["mode"]):
    array_new["chassis"]["mode_param"]["A"]             = array_old["chassis"]["wheelbase"]
    array_new["chassis"]["mode_param"]["B"]             = 0
    array_new["chassis"]["mode_param"]["theta"]         = 0
elif (2 == array_old["chassis"]["mode"]):
    array_new["chassis"]["mode_param"]["A"]             = array_old["chassis"]["wheelbase"]
    array_new["chassis"]["mode_param"]["B"]             = 0
    array_new["chassis"]["mode_param"]["theta"]         = 0
elif (3 == array_old["chassis"]["mode"]):
    array_new["chassis"]["mode_param"]["A"]             = array_old["chassis"]["wheelbase"]
    array_new["chassis"]["mode_param"]["B"]             = 0
    array_new["chassis"]["mode_param"]["theta"]         = 0
    print("should cleck A and B")
elif (4 == array_old["chassis"]["mode"]):
    array_new["chassis"]["mode_param"]["A"]             = 0
    array_new["chassis"]["mode_param"]["B"]             = array_old["chassis"]["wheelbase"]
    array_new["chassis"]["mode_param"]["theta"]         = 0
else:
    print("no define mode")

array_new["chassis"]["mode_param"]["wheelRadius"]       = array_old["chassis"]["wheelRadius"]
array_new["chassis"]["mode_param"]["reductionRatio"]    = array_old["chassis"]["reductionRatio"]
array_new["chassis"]["mode_param"]["encoderLine"]       = array_old["chassis"]["encoderLine"]
array_new["chassis"]["mode_param"]["maxMotorRpm"]       = array_old["chassis"]["maxMotorRpm"]
array_new["chassis"]["mode_param"]["inverse"]           = []
array_new["chassis"]["mode_param"]["inverse"].append(array_old["chassis"]["inverse"])
array_new["chassis"]["mode_param"]["inverse"].append(array_old["chassis"]["inverse"])
array_new["chassis"]["mode_param"]["inverse"].append(array_old["chassis"]["inverse"])
array_new["chassis"]["mode_param"]["inverse"].append(array_old["chassis"]["inverse"])

array_new["chassis"]["mode_param"]["absEncoder"]        = 1
array_new["chassis"]["mode_param"]["absEncoderRange"]   = []
array_new["chassis"]["mode_param"]["absEncoderRange"].append(0)
array_new["chassis"]["mode_param"]["absEncoderRange"].append(1000)
array_new["chassis"]["mode_param"]["steerAngle"]        = []
array_new["chassis"]["mode_param"]["steerAngle"].append(-90)
array_new["chassis"]["mode_param"]["steerAngle"].append(90)
array_new["chassis"]["mode_param"]["driver"]            = array_old["chassis"]["reductionRatio"]
array_new["chassis"]["mode_param"]["driverBrand"]       = "none"
array_new["chassis"]["mode_param"]["driverID"]          = []
array_new["chassis"]["mode_param"]["driverID"].append(1)
array_new["chassis"]["mode_param"]["driverID"].append(2)
array_new["chassis"]["mode_param"]["driverID"].append(3)
array_new["chassis"]["mode_param"]["driverID"].append(4)

array_new["chassis"]["gyro"]            = array_old["chassis"]["gyro"]
array_new["chassis"]["autoGyroCal"]     = array_old["chassis"]["autoGyroCal"]
if array_old["chassis"]["brake"] == True:
    array_new["chassis"]["brake"] = 1
else:
    array_new["chassis"]["brake"] = 0

array_new["chassis"]["autoBrake"]       = array_old["chassis"]["autoBrake"]
array_new["chassis"]["batteryInfo"]     = array_old["chassis"]["batteryInfo"]
if array_old["chassis"]["LED"] == True:
    array_new["chassis"]["LED"] = 1
else:
    array_new["chassis"]["LED"] = 0

array_new["laser"] = {}
array_new["laser"]["num"]       = array_old["laser"]["num"]
array_new["laser"]["index"] = []
for i in range(array_new["laser"]["num"]):
    index = {}
    index["model"]         = array_old["laser"]["index"][i]["model"]
    index["x"]             = array_old["laser"]["index"][i]["x"]
    index["y"]             = array_old["laser"]["index"][i]["y"]
    index["z"]             = array_old["laser"]["index"][i]["z"]
    index["r"]             = array_old["laser"]["index"][i]["r"]
    index["step"]          = array_old["laser"]["index"][i]["step"]
    index["minAngle"]      = array_old["laser"]["index"][i]["minAngle"]
    index["maxAngle"]      = array_old["laser"]["index"][i]["maxAngle"]
    index["upside"]        = array_old["laser"]["index"][i]["upside"]
    index["ip"]            = array_old["laser"]["index"][i]["ip"]
    index["port"]          = array_old["laser"]["index"][i]["port"]
    index["useForLocalization"] = array_old["laser"]["index"][i]["useForLocalization"]
    array_new["laser"]["index"].append(index)

array_new["DI"] = {}
array_new["DI"]["maxDINum"]     = 16
array_new["DI"]["num"]          = array_old["DI"]["num"]
array_new["DI"]["index"]        = []
for i in range(array_new["DI"]["num"]):
    index = {}
    index["id"]       = array_old["DI"]["index"][i]["id"]
    index["func"]     = array_old["DI"]["index"][i]["func"]
    index["type"]     = array_old["DI"]["index"][i]["type"]
    index["x"]        = array_old["DI"]["index"][i]["x"]
    index["y"]        = array_old["DI"]["index"][i]["y"]
    index["z"]        = array_old["DI"]["index"][i]["z"]
    index["r"]        = array_old["DI"]["index"][i]["r"]
    index["range"]    = 1
    index["minDist"]  = 0.1
    index["maxDist"]  = 0.2
    index["inverse"]  = array_old["DI"]["index"][i]["inverse"]
    array_new["DI"]["index"].append(index)

array_new["DO"] = {}
array_new["DO"]["maxDONum"]     = 16
array_new["DO"]["num"]          = array_old["DO"]["num"]
array_new["DO"]["defaultValue"] = array_old["DO"]["defaultValue"]
array_new["DO"]["index"]        = []
for i in range(array_new["DO"]["num"]):
    index = array_old["DO"]["index"][i]
    array_new["DO"]["index"].append(index)

array_new["ultrasonic"] = {}
array_new["ultrasonic"]["type"]                     = 0
array_new["ultrasonic"]["maxUltrasonicNum"]         = 24
array_new["ultrasonic"]["num"]                      = array_old["ultrasonic"]["num"]
array_new["ultrasonic"]["index"]                    = []
for i in range(array_new["DO"]["num"]):
    index = {}
    index["func"]     = array_old["ultrasonic"]["num"][i]["func"]
    index["id"]       = array_old["ultrasonic"]["num"][i]["id"]
    index["maxDist"]  = array_old["ultrasonic"]["num"][i]["maxDist"]
    index["minDist"]  = array_old["ultrasonic"]["num"][i]["minDist"]
    index["r"]        = array_old["ultrasonic"]["num"][i]["r"]
    index["range"]    = array_old["ultrasonic"]["num"][i]["range"]
    index["x"]        = array_old["ultrasonic"]["num"][i]["x"]
    index["y"]        = array_old["ultrasonic"]["num"][i]["y"]
    index["z"]        = array_old["ultrasonic"]["num"][i]["z"]
    array_new["ultrasonic"]["index"].append(index)

array_new["magneticSensor"] = {}
array_new["magneticSensor"]["type"] = 0
array_new["magneticSensor"]["num"]  = array_old["magneticSensor"]["num"]
array_new["magneticSensor"]["index"] = []
for i in range(array_new["magneticSensor"]["num"]):
    index = {}
    index["id"]           = 1
    index["r"]            = 0.00000
    index["resolution"]   = 8.00000
    index["step"]         = 0.01000
    index["x"]            = 0.16500
    index["y"]            = -0.44000
    index["z"]            = 0.00000
    array_new["magneticSensor"]["index"].append(index)

array_new["RFID"] = {}
array_new["RFID"]["type"]   = 0
array_new["RFID"]["num"]    = 0
array_new["RFID"]["index"]  = []

array_new["fork"] = {}
array_new["fork"]["type"]           = 0
array_new["fork"]["pump"]           = 0
array_new["fork"]["encoderSteps"]   = 16384
array_new["fork"]["lengthPerTurn"]  = 0.15
array_new["fork"]["driverPeriod"]   = 0.008
array_new["fork"]["pressureSensor"] = 0
array_new["fork"]["minHeight"]      = 0.0
array_new["fork"]["maxHeight"]      = 0.1
array_new["fork"]["blockLaserDist"] = 0.0
array_new["fork"]["lengthFactor"]   = 1.0

array_new["speaker"] = {}

temp = js.dumps(array_new, sort_keys = True, indent = 4, separators=(',', ': '))

print("temp = ", temp)

new_file_object.write(temp)
new_file_object.close()

os.system('pause')
sys.exit()
