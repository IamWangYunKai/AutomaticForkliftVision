module(..., package.seeall)

local json = require('json')

function sendSpeed(dev, speed)
	local nav_info = {}
	nav_info.x = 0
	nav_info.rotate = 1
	dev.updateJsonMsg("rbk.protocol.Message_NavSpeed", json.encode(nav_info))
end

function speak(dev, str, id, speed, is_setting)
	if is_setting == nil then
		-- is_setting = false
		is_setting = false
	end
	if id == nil then
		id = 0
	end
	if speed == nil then
		speed = 0
	end
	local tts = {}

	tts.str = str
	tts.voiceId = id
	tts.speed = speed
	tts.is_setting = is_setting
	dev.updateJsonMsg("rbk.protocol.Message_TTS", json.encode(tts))
end

-- enum NavMode{
-- 		TaskTargetReachMode = 1;
-- 		SpeedControlMode = 2;
-- 	}
-- 	enum NavCmd{
-- 		TaskCancel = 1;
-- 		TaskSuspend = 2;
-- 		TaskResume = 3;
-- 		TaskBegin = 4;
-- 	}
--mode: Forward = 1 ; BackWard = 2
function moveToTarget(dev, x, y, w, mode)
	local nav_info = {}

	nav_info.nav_mode = 1
	nav_info.nav_cmd = 4
	nav_info.nav_target_x = x
	nav_info.nav_target_y = y
	nav_info.nav_target_theta = w
	nav_info.nav_target_mode = mode
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function cancelTask(dev)
	local nav_info = {}
	nav_info.nav_mode = 2
	nav_info.nav_cmd = 1
	nav_info.nav_speed_x = 0
	nav_info.nav_speed_y = 0
	nav_info.nav_speed_w = 0
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function moveLine(dev, speed)
	local nav_info = {}
	nav_info.nav_mode = 2
	nav_info.nav_cmd = 1
	nav_info.nav_speed_x = speed
	nav_info.nav_speed_y = 0
	nav_info.nav_speed_w = 0
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function suspendTask(dev)
	local nav_info = {}
	nav_info.nav_mode = 1
	nav_info.nav_cmd = 2
	nav_info.nav_speed_x = 0
	nav_info.nav_speed_y = 0
	nav_info.nav_speed_w = 0
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function resumeTask( dev )
	local nav_info = {}
	nav_info.nav_mode = 1
	nav_info.nav_cmd = 3
	nav_info.nav_speed_x = 0
	nav_info.nav_speed_y = 0
	nav_info.nav_speed_w = 0
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function suspendTaskTopo(dev)
	local nav_info = {}
	nav_info.nav_mode = 3
	nav_info.nav_cmd = 2
	nav_info.nav_speed_x = 0
	nav_info.nav_speed_y = 0
	nav_info.nav_speed_w = 0
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function resumeTaskTopo( dev )
	local nav_info = {}
	nav_info.nav_mode = 3
	nav_info.nav_cmd = 3
	nav_info.nav_speed_x = 0
	nav_info.nav_speed_y = 0
	nav_info.nav_speed_w = 0
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function moveCircle(dev, speed)
	local nav_info = {}
	nav_info.nav_mode = 2
	nav_info.nav_cmd = 1
	nav_info.nav_speed_x = 0
	nav_info.nav_speed_y = 0
	nav_info.nav_speed_w = speed
	dev.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
end

function sendVoice(dev, voice_str, voice_size)
	local voice = {}
	voice.data = voice_str
	voice.length = voice_size
	dev.updateJsonMsg("rbk.protocol.Message_Voice", json.encode(voice))
end

function goTargetId(dev, target_id, target_angle)
	local target = {}
	target.id = target_id
	target.angle = target_angle
	dev.updateJsonMsg("rbk.protocol.Message_NavTopoPose", json.encode(target))
end

function setAdvanceTask(task_name)
	local new_task = {}
	new_task.b = tt.create(task_name)
	new_task.bindEvent = function(dev, event, func)
		local fn_s = task_name.."__"..dev.plugin_name.."__"..event
	    _G[fn_s] = func
	    new_task.b:bindEvent(dev.plugin_name, event, fn_s)
	end
	new_task.run = function ()
		new_task.b:run()
	end
	new_task.finish = function ()
		new_task.b:finish()
	end
	new_task.getName = function ()
		return new_task.b:getName()
	end
	new_task.setPriority = function (pri)
		new_task.b:setPriority(pri)
	end
	return new_task
end

function changeSpeedAccordingToOdo(Loc, Nav)
	local lLoc = Loc
	local lNav = Nav

	local agent = task.setAdvanceTask("changeSpeedAccordingToOdo")
	agent.bindEvent(lLoc, "changeToOdoMode", function ()
		lNav.setMotionControlLimits(0.3,0.3,math.pi/4,math.pi/4)
	end)

	agent.bindEvent(lLoc, "changeToNormalMode", function ()
		lNav.setMotionControlLimits(0.5, 0.3, math.pi/3.0,math.pi/4.0)
	end)

	agent.setPriority(0)

	return agent
end

function moveToToPoTarget( NavigatonStack )
	local lNavigationStack = NavigatonStack
	local move_to_topo_target = task.setAdvanceTask("moveToToPoTarget")

	move_to_topo_target.bindEvent(NavigatonStack,"reachTarget",function (  )
		task.cancelTask(lNavigationStack)
		move_to_topo_target.finish()
	end)
	move_to_topo_target.gotoTarget = function ( id,theta,mode )
		move_to_topo_target.run()
		local nav_info = {}
		nav_info.nav_mode = 3
		nav_info.nav_cmd = 4
		nav_info.topo_target_id = id
		nav_info.nav_target_theta = theta
		nav_info.nav_target_mode = mode
		lNavigationStack.updateJsonMsg("rbk.protocol.Message_NavInfo", json.encode(nav_info))
	end
	move_to_topo_target.bindEvent(lNavigationStack, "changeToBlocked", function ()
		task.suspendTaskTopo(lNavigationStack)
		--task.tempStop(lNav)
	end)
	move_to_topo_target.bindEvent(lNavigationStack, "changeToNoBlocked", function ()
		task.resumeTaskTopo(lNavigationStack)
		--task.autoMove(lNav)
	end)
	move_to_topo_target.setPriority(1)
	return move_to_topo_target
end

function moveWithoutStop(NavigatonStack)
	local lNavigationStack = NavigatonStack
	local move_without_stop = task.setAdvanceTask("moveWithoutStop")

	move_without_stop.bindEvent(lNavigationStack, "reachTarget", function ()
		task.cancelTask(lNavigationStack)
		move_without_stop.finish()
	end)

	move_without_stop.gotoTarget = function (x, y, theta, mode)
		move_without_stop.run()
		task.moveToTarget(lNavigationStack, x, y, theta,mode)
	end

	move_without_stop.setPriority(1)
	return move_without_stop
end

function moveWithStop(NavigatonStack)
	local lNavigationStack = NavigatonStack

	local move_with_stop = task.setAdvanceTask("moveWithStop")
	move_with_stop.bindEvent(lNavigationStack, "reachTarget", function ()
		task.cancelTask(lNavigationStack)
		move_with_stop.finish()
	end)

	move_with_stop.bindEvent(lNavigationStack, "changeToBlocked", function ()
		task.suspendTask(lNavigationStack)
		--task.tempStop(lNav)
	end)

	move_with_stop.bindEvent(lNavigationStack, "changeToNoBlocked", function ()
		task.resumeTask(lNavigationStack)
		--task.autoMove(lNav)
	end)

	move_with_stop.gotoTarget = function (x, y, theta,mode)
		move_with_stop.run()
		task.moveToTarget(lNavigationStack, x, y, theta, mode)
		--task.autoMove(lNav)
	end

	move_with_stop.setPriority(1)
	return move_with_stop
end


function startFromHomeWithoutStop(NavigatonStack,BMS)
	local lNavigationStack = NavigatonStack
	local lBMS = BMS
	local start_from_home_without_stop = task.setAdvanceTask("startFromHomeWithoutStop")
	local m_x = 1
	local m_y = 1
	local m_theta = 1
	local m_mode = 1
	local m_vx = 0.05
	start_from_home_without_stop.bindEvent(lNavigationStack, "reachTarget", function ()
		task.cancelTask(lNavigationStack)
		start_from_home_without_stop.finish()
	end)

	start_from_home_without_stop.bindEvent(lBMS, "robotOutCharge", function ()
		task.moveToTarget(lNavigationStack,m_x, m_y, m_theta, m_mode)
	end)

	start_from_home_without_stop.gotoTarget = function (x, y, theta, mode)
		m_x = x
		m_y = y
		m_theta = theta
		m_mode = mode
		start_from_home_without_stop.run()
		if lBMS.robot_in_charge() == true then
			task.moveLine(lNavigationStack,m_vx)
		else
			task.moveToTarget(lNavigationStack, m_x, m_y, m_theta, m_mode)
		end
		--task.moveToTarget(lNavigationStack, x, y, theta,mode)
	end

	start_from_home_without_stop.setPriority(1)
	return start_from_home_without_stop
end

function startFromHomeWithStop(NavigatonStack,BMS)
	local lNavigationStack = NavigatonStack
	local lBMS = BMS
	local start_from_home_with_stop = task.setAdvanceTask("startFromHomeWithStop")
	local m_x = 1
	local m_y = 1
	local m_theta = 1
	local m_mode = 1
	local m_vx = 0.05
	local x = 1
	local go_to_target = false

	start_from_home_with_stop.bindEvent(lNavigationStack, "reachTarget", function ()
		task.cancelTask(lNavigationStack)
		start_from_home_with_stop.finish()
	end)

	start_from_home_with_stop.bindEvent(lBMS, "robotOutCharge", function ()
		go_to_target = true
		task.moveToTarget(lNavigationStack,m_x, m_y, m_theta, m_mode)
	end)

	start_from_home_with_stop.bindEvent(lNavigationStack, "changeToBlocked", function ()
		if go_to_target == true then
			task.suspendTask(lNavigationStack)
		end
		--task.tempStop(lNav)
	end)

	start_from_home_with_stop.bindEvent(lNavigationStack, "changeToNoBlocked", function ()
		if go_to_target == true then
			task.resumeTask(lNavigationStack)
		end
		--task.autoMove(lNav)
	end)

	start_from_home_with_stop.gotoTarget = function (x, y, theta,mode)
		m_x = x
		m_y = y
		m_theta = theta
		m_mode = mode
		start_from_home_with_stop.run()
		go_to_target = false

		if lBMS.robot_in_charge() == true then

			task.moveLine(lNavigationStack,m_vx)
		else
			go_to_target = true
			task.moveToTarget(lNavigationStack, m_x, m_y, m_theta, m_mode)
		end
		--task.moveToTarget(lNavigationStack, x, y, theta, mode)
		--task.autoMove(lNav)
	end

	start_from_home_with_stop.setPriority(1)
	return start_from_home_with_stop
end

function goToChargePoint(NavigationStack, Infrared, BMS)

	local lNavigationStack = NavigationStack
	local lInfrared = Infrared
	local lBMS = BMS
	local go_to_charge_point = task.setAdvanceTask("goToChargePoint")

	-- local has_run = false
	local m_x = 1
	local m_y = 1
	local m_theta = 1
	local m_mode = 1
	local m_vx = 0.01
	local m_w = -1 * math.pi / 180.0

	local state1 = "PREPARE"
	local state2 = "LEFTEXIST"
	local state3 = "RIGHTEXIST"
	local state4 = "BOTHEXIST"
	local state5 = "BOTHLOST"

	local action1 = "GOTOPOS"
	local action2 = "TURNLEFT"
	local action3 = "TURNRIGHT"
	local action4 = "TURNBACK"

	local last_state = "NONE"
	local last_action = "NONE"

	local reachPreparePos = false

	go_to_charge_point.stateMachine = function ( state )
		--print(state)
		--print(last_state)
		local  current_action = "NONE"
		if last_state ~= state then
			if state == "PREPARE" then
				current_action = "GOTOPOS"
			elseif state == "LEFTEXIST" then
				current_action = "TURNLEFT"
			elseif state == "RIGHTEXIST" then
				current_action = "TURNRIGHT"
			elseif state == "BOTHEXIST" then
				current_action = "TURNBACK"
			elseif state == "BOTHLOST" then
				current_action = "GOTOPOS"
			end
		else
			current_action = last_action
		end
		last_state = state

		if last_action ~= current_action then
			--print(current_action)
		   -- print(last_action)
			if current_action == "GOTOPOS" then
				reachPreparePos = false
				task.moveToTarget(lNavigationStack,m_x,m_y,m_theta,m_mode)
			elseif current_action == "TURNLEFT" then
				task.moveCircle(lNavigationStack,m_w)
			elseif current_action == "TURNRIGHT" then
				task.moveCircle(lNavigationStack,-1*m_w)
			elseif current_action == "TURNBACK" then
				task.moveLine(lNavigationStack,-1*m_vx)
			end
		end
		last_action = current_action
	end


	go_to_charge_point.bindEvent(lNavigationStack, "reachTarget", function ()
		reachPreparePos = true
		task.cancelTask(lNavigationStack)
		local left = lInfrared.left_infrared_detected()
		local right = lInfrared.right_infrared_detected()

		if left == true and right == true then
			print("reach target: both exist")
			go_to_charge_point.stateMachine("BOTHEXIST")
		elseif left == true then
			print("reach target: left exist")
			go_to_charge_point.stateMachine("LEFTEXIST")
		elseif right == true then
			print("reach target: right exist")
			go_to_charge_point.stateMachine("RIGHTEXIST")
		else
			print("reach target: both lost")
			go_to_charge_point.stateMachine("PREPARE")
		end
	end)

	go_to_charge_point.gotToPreparePos = function (x, y, theta, mode)
		print("go to prepare pos !!!!!!!!")
		reachPreparePos = false
		go_to_charge_point.run()
		m_x = x
		m_y = y
		m_theta = theta
		m_mode = mode
		last_state = "NONE"
	 	last_action = "NONE"
		go_to_charge_point.stateMachine("PREPARE")
	end

	go_to_charge_point.bindEvent(lInfrared,"infraredBothDetected",function (  )
		if reachPreparePos == true then
			go_to_charge_point.stateMachine("BOTHEXIST")
		else
			print("Both detected, but not reach prepare pos yet")
		end
	end)

	go_to_charge_point.bindEvent(lInfrared,"infraredBothLose",function (  )
		if reachPreparePos == true then
			print("Has reached prepare pos,but neither infrared detected")
			go_to_charge_point.stateMachine("PREPARE")
		else
			print("Neither detected, but not reach prepare pos yet")
		end
	end)

	go_to_charge_point.bindEvent(lInfrared,"infraredOnlyRightDetected",function (  )
		if reachPreparePos == true then
			--print("Has reached prepare pos,but neither infrared detected")
			go_to_charge_point.stateMachine("RIGHTEXIST")
		else
			print("Right detected, but not reach prepare pos yet")
		end
	end)

	go_to_charge_point.bindEvent(lInfrared,"infraredOnlyLeftDetected",function (  )
		if reachPreparePos == true then
			go_to_charge_point.stateMachine("LEFTEXIST")
		else
			print("Left detected, but not reach prepare pos yet")
		end
	end)

	go_to_charge_point.bindEvent(lBMS,"robotInCharge", function (  )
		print("Robot in charge")
		task.cancelTask(lNavigationStack)
		go_to_charge_point.finish()
	end)

	go_to_charge_point.setPriority(1)
	return go_to_charge_point
end

function smartGotoPosition(dev, name, x, y, theta, mode, max_speed, max_rot, max_acc, max_rot_acc)
 	local move_task = {
 		skill_name = "SmartGotoPose",
 		target_name = {
 			value = name
 		},
 		target_x = {
 			value = x
 		},
 		target_y = {
 			value = y
 		},
 		target_angle = {
 			value = theta
 		},
 		reach_method = {
 			value = mode
 		},
		max_speed = {
 			value = max_speed
 		},
 		max_acc = {
 			value = max_acc
 		},
 		max_rot = {
 			value = max_rot
 		},
 		max_rot_acc = {
 			value = max_rot_acc
 		},
 	}
	dev.updateJsonMsg("rbk.protocol.Message_MoveTask", json.encode(move_task))
end

function simpleGoto(dev, name, x, y, theta,mode,max_speed, max_acc, max_rot, max_rot_acc)
 	local move_task = {
 		skill_name = "SimpleGotoPose",
 		target_name = {
 			value = name
 		},
 		target_x = {
 			value = x
 		},
 		target_y = {
 			value = y
 		},
 		target_angle = {
 			value = theta
 		},
 		max_speed = {
 			value = max_speed
 		},
 		max_acc = {
 			value = max_acc
 		},
 		max_rot = {
 			value = max_rot
 		},
 		max_rot_acc = {
 			value = max_rot_acc
 		},
 	}
	dev.updateJsonMsg("rbk.protocol.Message_MoveTask", json.encode(move_task))
end

function gotoSpecifiedPose(dev, id, theta, max_speed, max_acc, max_rot, max_rot_acc)
 	local move_task = {
 		skill_name = "GotoSpecifiedPose",
 		target_name = {
 			value = ""..id
 		},
 		target_angle = {
 			value = theta
 		},
 		max_speed = {
 			value = max_speed
 		},
 		max_acc = {
 			value = max_acc
 		},
 		max_rot = {
 			value = max_rot
 		},
 		max_rot_acc = {
 			value = max_rot_acc
 		},
 	}
	dev.updateJsonMsg("rbk.protocol.Message_MoveTask", json.encode(move_task))
end

function autoCharge( dev,name,x,y,theta,vx,vw)
	local move_task = {
		skill_name = "AutoCharge",
		target_name = {
			value = ""..name
		},
		target_x = {
			value = x
		},
		target_y = {
			value = y
		},
		target_angle = {
			value = theta
		},
		reach_method = {
			value = "forward"
		},
		speed_x = {
			value = vx
		},
		speed_w = {
			value = vw
		}
	}
	dev.updateJsonMsg("rbk.protocol.Message_MoveTask", json.encode(move_task))
end

function goByOdometer(dev, move_dist, move_angle, speed_x, speed_y, speed_w)
 	local move_task = {
 		skill_name = "GoByOdometer",
 		move_dist = {
 			value = move_dist
 		},
 		move_angle = {
 			value = move_angle
 		},
 		speed_x = {
 			value = speed_x
 		},
 		speed_y = {
 			value = speed_y
 		},
 		speed_w = {
 			value = speed_w
 		}
 	}
	dev.updateJsonMsg("rbk.protocol.Message_MoveTask", json.encode(move_task))
end


function simpleGotoDWA(dev, name, x, y, theta,mode,max_speed, max_acc, max_rot, max_rot_acc)
 	local move_task = {
 		skill_name = "SimpleGotoPointDWA",
 		target_name = {
 			value = name
 		},
 		target_x = {
 			value = x
 		},
 		target_y = {
 			value = y
 		},
 		target_angle = {
 			value = theta
 		},
 		max_speed = {
 			value = max_speed
 		},
 		max_acc = {
 			value = max_acc
 		},
 		max_rot = {
 			value = max_rot
 		},
 		max_rot_acc = {
 			value = max_rot_acc
 		},
 	}
	dev.updateJsonMsg("rbk.protocol.Message_MoveTask", json.encode(move_task))
end

function goAlongMagstripe(dev, max_speed, max_rot, mode)
	local move_task = {
 		skill_name = "GoAlongMagstripe",
 		max_speed = {
 			value = max_speed
 		},
 		max_rot = {
 			value = max_rot
 		},
 		mode = {
 			value = mode
 		}
 	}
	dev.updateJsonMsg("rbk.protocol.Message_MoveTask", json.encode(move_task))
end
