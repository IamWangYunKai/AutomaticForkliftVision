-- This is a Robokit Logic file

if rbkConfig.simulation then
    -- simulation
else
	-- real
	gChassis.bindEvent("emc_pressed", function ()
        gMoveFactory.suspendTask()
    end)

    gChassis.bindEvent("emc_released", function ()
        gMoveFactory.resumeTask(true)
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
