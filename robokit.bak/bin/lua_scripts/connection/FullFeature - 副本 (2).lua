gNetProtocol = rbk.getPlugin("NetProtocol")
gEKF = rbk.getPlugin("RobotPosEKF")
gObj = rbk.getPlugin("LaserObstacleDetection")
gFus =rbk.getPlugin("FusedLaser")
gMap = rbk.getPlugin("MapConstructor")
gLoc = rbk.getPlugin("MCLoc")
gJoystick = rbk.getPlugin("XBox360Joystick")
gDebug = rbk.getPlugin("QtDebugGui")
gMoveFactory = rbk.getPlugin("MobileRobotFactory")
gMapLogger = rbk.getPlugin("OnlineMapLogger")
gPlayer = rbk.getPlugin("SoundPlayer")
gTask = rbk.getPlugin("TaskManager")
gSeg = rbk.getPlugin("LaserSegmentation")
gABB = rbk.getPlugin("ABB")
-- gFruitVisionTest = rbk.getPlugin("FruitVisionTest")
-- gPeripheral = rbk.getPlugin("PeripheralDevice")

if rbkConfig.simulation then
    -- simulation
	gChassis = rbk.getPlugin("PioneerSim")

	gChassis.connect(gMoveFactory,"rbk.protocol.Message_LaserPointCloud")
	gFus.connect(gMoveFactory,"rbk.protocol.Message_Laser")
	gMap.connect(gMoveFactory,"rbk.protocol.Message_Map")
	gMap.connect(gNetProtocol,"rbk.protocol.Message_Map")
	gLoc.connect(gMoveFactory,"rbk.protocol.Message_Localization")
	gObj.connect(gMoveFactory,"rbk.protocol.Message_Object")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_NavSpeed")

	gLoc.connect(gSeg, "rbk.protocol.Message_Localization")
	gChassis.connect(gSeg, "rbk.protocol.Message_Laser")
	gSeg.connect(gDebug, "rbk.protocol.Message_Debug")
	gSeg.connect(gMoveFactory, "rbk.protocol.Message_LaserSegResult")

	gMap.connect(gFus,"rbk.protocol.Message_Map")
	gLoc.connect(gFus,"rbk.protocol.Message_Localization")
	gMoveFactory.connect(gFus, "rbk.protocol.Message_VirtualLineList")

	gFus.connect(gObj,"rbk.protocol.Message_Laser")
	gLoc.connect(gObj,"rbk.protocol.Message_Localization")

	gMap.connect(gLoc, "rbk.protocol.Message_Map")
	gEKF.connect(gLoc,"rbk.protocol.Message_Odometer")
	gEKF.connect(gNetProtocol, "rbk.protocol.Message_NavSpeed")

	gChassis.connect(gEKF,"rbk.protocol.Message_Odometer")
	gChassis.connect(gEKF,"rbk.protocol.Message_IMU")

	gMap.connect(gDebug,"rbk.protocol.Message_Map")
	gLoc.connect(gDebug,"rbk.protocol.Message_Debug")
	gObj.connect(gDebug,"rbk.protocol.Message_Debug")
	gFus.connect(gDebug,"rbk.protocol.Message_Debug")
	gMoveFactory.connect(gDebug,"rbk.protocol.Message_Debug")

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

	gChassis.connect(gFus,"rbk.protocol.Message_Laser")
	gChassis.connect(gLoc,"rbk.protocol.Message_Laser")
	gChassis.connect(gMoveFactory, "rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_DI")
	gChassis.connect(gNetProtocol,"rbk.protocol.Message_DO")

	gTask.connect(gNetProtocol, "rbk.protocol.Message_TaskStatus")

	-- gPeripheral.connect(gNetProtocol, "rbk.protocol.Message_Environment")
else
	-- real
	gChassis = rbk.getPlugin("DSPChassis")
	gLaser = rbk.getPlugin("MultiLaser")
	-- gIMU = rbk.getPlugin("MiniIMU")
	gLaser.connect(gMoveFactory,"rbk.protocol.Message_LaserPointCloud")
	gFus.connect(gMoveFactory,"rbk.protocol.Message_Laser")
	gMap.connect(gMoveFactory,"rbk.protocol.Message_Map")
	gMap.connect(gNetProtocol,"rbk.protocol.Message_Map")
	gLoc.connect(gMoveFactory,"rbk.protocol.Message_Localization")
	gObj.connect(gMoveFactory,"rbk.protocol.Message_Object")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_NavSpeed")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_Battery")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_DI")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_DO")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_Magnetic")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_RFID")
	gChassis.connect(gMoveFactory,"rbk.protocol.Message_Ultrasonic")
	gChassis.connect(gNetProtocol, "rbk.protocol.Message_Ultrasonic")

	gLoc.connect(gSeg, "rbk.protocol.Message_Localization")
	gLaser.connect(gSeg, "rbk.protocol.Message_Laser")
	gSeg.connect(gDebug, "rbk.protocol.Message_Debug")
	gSeg.connect(gMoveFactory, "rbk.protocol.Message_LaserSegResult")

	gMap.connect(gFus,"rbk.protocol.Message_Map")
	gLoc.connect(gFus,"rbk.protocol.Message_Localization")
	gMoveFactory.connect(gFus, "rbk.protocol.Message_VirtualLineList")

	gFus.connect(gObj,"rbk.protocol.Message_Laser")
	gLoc.connect(gObj,"rbk.protocol.Message_Localization")

	gMap.connect(gLoc, "rbk.protocol.Message_Map")
	gEKF.connect(gLoc,"rbk.protocol.Message_Odometer")
	gEKF.connect(gNetProtocol, "rbk.protocol.Message_NavSpeed")

	gChassis.connect(gEKF,"rbk.protocol.Message_Odometer")
	gChassis.connect(gEKF,"rbk.protocol.Message_IMU")

	gMap.connect(gDebug,"rbk.protocol.Message_Map")
	gLoc.connect(gDebug,"rbk.protocol.Message_Debug")
	gObj.connect(gDebug,"rbk.protocol.Message_Debug")
	gFus.connect(gDebug,"rbk.protocol.Message_Debug")
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
	gLaser.connect(gFus,"rbk.protocol.Message_Laser")

	gTask.connect(gNetProtocol, "rbk.protocol.Message_TaskStatus")

	-- gPeripheral.connect(gNetProtocol, "rbk.protocol.Message_Environment")

	gChassis.bindEvent("emc_pressed", function ()
        gMoveFactory.suspendTask()
    end)

    gChassis.bindEvent("emc_released", function ()
        gMoveFactory.resumeTask()
    end)
end

gDebug.bindEvent("get_map", function()
	gMap.sendMapImmediately()
end)

gDebug.bindEvent("reloc", function(x, y)
	gLoc.relocService(x, y, 3, 0, 180)
end)

gDebug.bindEvent("go_target", function(id)
	task.gotoSpecifiedPose(gMoveFactory, id)
end)

gDebug.bindEvent("go_point", function(x, y)
	task.smartGotoPosition(gMoveFactory, nil, x, y)
end)

-- 地图加载完成
gLoc.bindEvent("relocFinished", function()
	gNetProtocol.setLoadmapStatus(1)
end)

-- 重定位完成
gLoc.bindEvent("relocSuccessed", function ()
	gNetProtocol.setRelocStatus(3) -- 这里只会标记为重定位完成, 需要等用户确认成功
end)

-- 重定位失败
gLoc.bindEvent("relocFailed", function ()
	gNetProtocol.setRelocStatus(3)
end)

gMoveFactory.bindEvent("needGyroCali", function ()
	if not rbkConfig.simulation then
	    gChassis.calGyro()
    end
end)

local is_manual_mode = false
local is_joystick_connected = false

gJoystick.bindEvent("connected", function ()
	is_joystick_connected = true
	gMoveFactory.setAutoMode(false)
	gNetProtocol.disconnect(gMoveFactory, "rbk.protocol.Message_ManualSpeed")
	gJoystick.connect(gMoveFactory, "rbk.protocol.Message_ManualSpeed")
end)

gJoystick.bindEvent("disconnected", function ()
	is_joystick_connected = false
	gJoystick.disconnect(gMoveFactory, "rbk.protocol.Message_ManualSpeed")
	if is_manual_mode then
		gNetProtocol.connect(gMoveFactory, "rbk.protocol.Message_ManualSpeed")
	else
		gMoveFactory.setAutoMode(true)
		gNetProtocol.disconnect(gMoveFactory, "rbk.protocol.Message_ManualSpeed")
	end
end)

gNetProtocol.bindDelegate("manual", function ()
	gMoveFactory.setAutoMode(false)
    is_manual_mode = true
	if not is_joystick_connected then
		gNetProtocol.connect(gMoveFactory, "rbk.protocol.Message_ManualSpeed")
	end
end)

gNetProtocol.bindDelegate("auto", function ()
    is_manual_mode = false
	if not is_joystick_connected then
		gMoveFactory.setAutoMode(true)
		gNetProtocol.disconnect(gMoveFactory, "rbk.protocol.Message_ManualSpeed")
	end
end)

gNetProtocol.bindDelegate("afd_lift", function(position)
	return gChassis.setAfdLift(position)
end)

gABB.bindDelegate("go_target", function (pos)
	print("hhhh",pos)
	task.gotoSpecifiedPose(gMoveFactory, pos)
end)
	-- gABB.targetReached()

gChassis.start()
rbk.sleep(2000)
gLoc.start()
gEKF.start()
gMap.start()
gDebug.start()
gJoystick.start()
gObj.start()
gFus.start()
gMoveFactory.start()
gMapLogger.start()
gPlayer.start()
gSeg.start()
-- gPeripheral.start()
-- gFruitVisionTest.start()
if not rbkConfig.simulation then
	gLaser.start()
end
if rbkConfig.useNetProtocol then
	gNetProtocol.start()
end
gTask.start()
-- gABB.start()