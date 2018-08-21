-- This is a Robokit Connection file

gNetProtocol = rbk.getPlugin("NetProtocol")
gEKF = rbk.getPlugin("RobotPosEKF")
gMap = rbk.getPlugin("MapConstructor")
gLoc = rbk.getPlugin("MCLoc")
gJoystick = rbk.getPlugin("XBox360Joystick")
gDebug = rbk.getPlugin("QtDebugGui")
gMoveFactory = rbk.getPlugin("MoveFactory")
gMapLogger = rbk.getPlugin("OnlineMapLogger")
gPlayer = rbk.getPlugin("SoundPlayer")
gTask = rbk.getPlugin("TaskManager")
gSeg = rbk.getPlugin("LaserSegmentation")
gSensorFuser = rbk.getPlugin("SensorFuser")

if rbkConfig.simulation then
    -- simulation
	gChassis = rbk.getPlugin("PioneerSim")

	gSensorFuser.connect(gMoveFactory,"rbk.protocol.Message_SensorPointCloud")
	gLoc.connect(gSensorFuser,"rbk.protocol.Message_Localization")
	gMap.connect(gSensorFuser,"rbk.protocol.Message_Map")
	gMoveFactory.connect(gSensorFuser, "rbk.protocol.Message_VirtualLineList")
	gSensorFuser.connect(gDebug,"rbk.protocol.Message_Debug")
	gSensorFuser.connect(gNetProtocol, "rbk.protocol.Message_UserObject")
	gChassis.connect(gSensorFuser,"rbk.protocol.Message_AllLasers")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_AllLasers");

	gMap.connect(gMoveFactory,"rbk.protocol.Message_Map")
	gMap.connect(gNetProtocol,"rbk.protocol.Message_Map")
	gLoc.connect(gMoveFactory,"rbk.protocol.Message_Localization")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_NavSpeed")

	gLoc.connect(gSeg, "rbk.protocol.Message_Localization")
	gChassis.connect(gSeg, "rbk.protocol.Message_Laser")
	gSeg.connect(gDebug, "rbk.protocol.Message_Debug")
	gSeg.connect(gMoveFactory, "rbk.protocol.Message_LaserSegResult")

	gMap.connect(gLoc, "rbk.protocol.Message_Map")
	gEKF.connect(gLoc,"rbk.protocol.Message_Odometer")
	gMoveFactory.connect(gNetProtocol, "rbk.protocol.Message_NavSpeed")

	gChassis.connect(gEKF,"rbk.protocol.Message_Odometer")
	gChassis.connect(gEKF,"rbk.protocol.Message_IMU")

	gMap.connect(gDebug,"rbk.protocol.Message_Map")
	gLoc.connect(gDebug,"rbk.protocol.Message_Debug")
	gMoveFactory.connect(gDebug,"rbk.protocol.Message_Debug")
	gChassis.connect(gDebug,"rbk.protocol.Message_Debug")


	gMoveFactory.connect(gChassis, "rbk.protocol.Message_NavSpeed")
	gMoveFactory.connect(gNetProtocol, "rbk.protocol.Message_NavPath")
	gMoveFactory.connect(gNetProtocol, "rbk.protocol.Message_MoveStatus")

	gLoc.connect(gNetProtocol, "rbk.protocol.Message_Localization")
	-- gChassis.connect(gNetProtocol,"rbk.protocol.Message_Battery")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Laser")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Odometer")
	gChassis.connect(gJoystick, "rbk.protocol.Message_Odometer");

	gChassis.connect(gMapLogger, "rbk.protocol.Message_Laser")
	gEKF.connect(gMapLogger, "rbk.protocol.Message_Odometer")
	gEKF.connect(gMoveFactory, "rbk.protocol.Message_Odometer")

	gChassis.connect(gLoc,"rbk.protocol.Message_Laser")
	gChassis.connect(gMoveFactory, "rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gSensorFuser, "rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gSensorFuser, "rbk.protocol.Message_DI")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_DI")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_DO")

	gTask.connect(gNetProtocol, "rbk.protocol.Message_TaskStatus")
else
	-- real
	gChassis = rbk.getPlugin("DSPChassis")
	gLaser = rbk.getPlugin("MultiLaser")
	-- gIMU = rbk.getPlugin("MiniIMU")
	gSensorFuser.connect(gMoveFactory,"rbk.protocol.Message_SensorPointCloud")
	gLoc.connect(gSensorFuser,"rbk.protocol.Message_Localization")
	gMap.connect(gSensorFuser,"rbk.protocol.Message_Map")
	gMoveFactory.connect(gSensorFuser, "rbk.protocol.Message_VirtualLineList")
	gSensorFuser.connect(gDebug,"rbk.protocol.Message_Debug")
	gSensorFuser.connect(gNetProtocol, "rbk.protocol.Message_UserObject")
	gLaser.connect(gSensorFuser,"rbk.protocol.Message_AllLasers")
	gLaser.connect(gNetProtocol,"rbk.protocol.Message_AllLasers");

	gMap.connect(gMoveFactory,"rbk.protocol.Message_Map")
	gMap.connect(gNetProtocol,"rbk.protocol.Message_Map")
	gLoc.connect(gMoveFactory,"rbk.protocol.Message_Localization")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_NavSpeed")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_Battery")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_DI")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_DO")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_Magnetic")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_RFID")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Fork")
	gChassis.connect(gSensorFuser, "rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gSensorFuser, "rbk.protocol.Message_DI")

	gLoc.connect(gSeg, "rbk.protocol.Message_Localization")
	gLaser.connect(gSeg, "rbk.protocol.Message_Laser")
	gSeg.connect(gDebug, "rbk.protocol.Message_Debug")
	gSeg.connect(gMoveFactory, "rbk.protocol.Message_LaserSegResult")

	gMap.connect(gLoc, "rbk.protocol.Message_Map")
	gEKF.connect(gLoc,"rbk.protocol.Message_Odometer")
	gMoveFactory.connect(gNetProtocol, "rbk.protocol.Message_NavSpeed")

	gChassis.connect(gEKF,"rbk.protocol.Message_Odometer")
	gChassis.connect(gEKF,"rbk.protocol.Message_IMU")

	gMap.connect(gDebug,"rbk.protocol.Message_Map")
	gLoc.connect(gDebug,"rbk.protocol.Message_Debug")
	gMoveFactory.connect(gDebug,"rbk.protocol.Message_Debug")

	gMoveFactory.connect(gChassis, "rbk.protocol.Message_NavSpeed")
	gMoveFactory.connect(gNetProtocol, "rbk.protocol.Message_NavPath")
	gMoveFactory.connect(gNetProtocol, "rbk.protocol.Message_MoveStatus")

	gLoc.connect(gNetProtocol, "rbk.protocol.Message_Localization")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_Battery")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_DI")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_DO")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_Controller")
	gLaser.connect(gNetProtocol, "rbk.protocol.Message_Laser")

	gLaser.connect(gMapLogger, "rbk.protocol.Message_Laser")
	gEKF.connect(gMapLogger, "rbk.protocol.Message_Odometer")
	gEKF.connect(gMoveFactory, "rbk.protocol.Message_Odometer")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Odometer")
	gChassis.connect(gJoystick, "rbk.protocol.Message_Odometer")
	gChassis.connect(gTask, "rbk.protocol.Message_DI")

	gLaser.connect(gLoc,"rbk.protocol.Message_Laser")

	gTask.connect(gNetProtocol, "rbk.protocol.Message_TaskStatus")
end

-- logic
local logicName = "./lua_scripts/logic/"..gLogic..".lua"
dofile(logicName)

gChassis.start()
if not rbkConfig.simulation then
	gLaser.start()
end
rbk.sleep(5000)
gEKF.start()
gMap.start()
gLoc.start()
-- gDebug.start()
gJoystick.start()
gMoveFactory.start()
gMapLogger.start()
gPlayer.start()
gTask.start()
rbk.sleep(3000)
if rbkConfig.useNetProtocol then
	gNetProtocol.start()
end
gSensorFuser.start()
gSeg.start()
