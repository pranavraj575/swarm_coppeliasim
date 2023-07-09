require 'ankiHeader'
require 'rosMsg'
require 'common'

function ctrlsFromInput(vec)
    -- must return in order lVel, rVel, headPos, liftHeight
    -- found in child script
    use_lift=vec[3] -- bit that decides whether to use lift control
    use_head=vec[5] -- bit that decides whether to tilt head

    lVel, rVel = getVelos(vec[1],vec[6])
    headPos=0
    liftHeight=0

    if (use_head~=0) then
        headPos=vec[4]
    end
    if (use_lift~=0) then
        liftHeight=vec[2]
    end

    return lVel, rVel, headPos, liftHeight
end

-- Inverse Kinematics to determine the left/right velocities for a differential drive model 
-- based on forward velocity and rotational velocity
function getVelos(xDot, thetaDot)
    local v = xDot
    local w = thetaDot
    if (v >= ANKI_MAX_SPEED) then
        v = ANKI_MAX_SPEED
    elseif (v <= -ANKI_MAX_SPEED) then
        v = -ANKI_MAX_SPEED
    end
    if (w < W_HYSTERESIS) and (w > -W_HYSTERESIS) then
        return v, v
    end
    local l = ANKI_BASELINE_L
    local R = v/w
    local vr = w*(R+l/2.0)
    local vl = w*(R-l/2.0)


    vl = clamp(vl, ANKI_MAX_SPEED, -ANKI_MAX_SPEED)
    vr = clamp(vr, ANKI_MAX_SPEED, -ANKI_MAX_SPEED)
    return vl, vr
end

-- Function to return the id number based on the model name that adheres to 
-- Coppeliasim's automatic naming convention. 
-- Models without a '#' are typically the first/original model and will have id 0
-- Models with a '#' and then a num, start incrementing from 0, 
-- so we add 1 to its number to offset it from the first/original model
function getIdFromName(nameStr)
    local result = string.find(nameStr, '#')
    if result then
        local hashSubstr = string.sub(nameStr, result + 1) -- +1 to ignore the hash
        return (tonumber(hashSubstr) + 1) -- +1 to offset from base model name not having a hash 
    else
        return 0
    end
end

-- This function will iterate over all objects in the scene and identify all the anki bots and their id
-- and return a formatted string for publishing to the /swarm/list topic
function searchSceneForAnkis()
    local swarmListStr = ''
    local objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 1)
    
    -- Iterate over every object and compare its name to the anki model name
    for _, v in pairs(objects) do    
        local objectName = sim.getObjectName(v)
        local result = string.find(objectName, ANKI_MODEL_NAME)
        if result then
            local id = getIdFromName(objectName)
            
            -- The first item in the list should not have any prefixed spaces, 
            -- while the rest of the items are space separated
            if string.len(swarmListStr) == 0  then
                swarmListStr = swarmListStr..'s'..id
            else                
                swarmListStr = swarmListStr..' '..'s'..id
            end
        end
    end
    -- Clear the swarm list signal so we do not re enter this function next simulation step
    sim.clearIntegerSignal(SWARM_LIST_SIGNAL)
    return swarmListStr
end

function createStateTable()
    ankiState = {leftSensor=1.0,
                centerSensor=1.0,
                rightSensor=1.0,
                emitter=false,
                position={x=0,y=0,z=0},
                orientation={x=0,y=0,z=0},
                randWalkCount = 0,
                randWalkLinX = 0,
                randWalkAngZ = 0,
                lastCtrlTime = 0.0,
                isEmitterOn=false,
                lightCycleStartTime= -randomFloat(0, LIGHT_PERIOD),
                lightDutyCycle=0.5,
                agentType="simple",
                wpIndex=math.random(1,2)
            }


    return ankiState
end

--function to determine wether or not contorl occurs in this time step
function determineDoCtrl(ankiState)
    local doCtrl = false
    local currTime = sim.getSimulationTime()
    local lastTime = ankiState["lastCtrlTime"]
    if (currTime - lastTime) >= CTRL_T then
        doCtrl = true
        ankiState["lastCtrlTime"] = currTime
    end

    return doCtrl
end

function determineSimpleControl(ankiState)
    local randWalkCount = ankiState["randWalkCount"]
    local randWalkLinX = ankiState["randWalkLinX"]
    local randWalkAngZ = ankiState["randWalkAngZ"]

    local v, w = 0,0

    leftDist = ankiState['leftSensor']
    centerDist = ankiState['centerSensor']
    rightDist = ankiState['rightSensor']
    seeSomething = leftDist <= DETECTION_THRESHOLD or rightDist <= DETECTION_THRESHOLD or centerDist <= DETECTION_THRESHOLD
    withinStopDist = leftDist <= STOP_THRESHOLD or rightDist <= STOP_THRESHOLD or centerDist <= STOP_THRESHOLD

    -- If we see light, move towards it
    -- Else, perform random walk
    if seeSomething then
        -- if we are witin stop dist, we wont move, otherwise we turn towards the closer/brighter object
        if withinStopDist then
            v = 0.0
            w = 0.0
        else
            v = FORWARD_RATE
            if (leftDist < rightDist and leftDist < centerDist) then
                --Left is closer, turn towards left                        
                w = TURNING_RATE
            elseif (rightDist < leftDist  and rightDist < centerDist) then
                --Right is closer, turn right                 
                w = -TURNING_RATE
            else
                -- Center is best, stay straight
                w = 0.0
            end
        end

        -- Since we saw something, we need to reset our random walk plan since it is no longer valid
        randWalkCount = 0   
    else
        -- Follow last random walk plan if not done 
        -- Else, Create new random walk   
        if (randWalkCount > 0) then
            randWalkCount = randWalkCount - 1
        else
            randWalkCount = math.floor(math.random(5, 20)) -- With a ctrlT of 0.1 s, this gives us a random walk for ~0.5 to 2.0s
            randWalkLinX = randomFloat(-MAX_FORWARD_RATE, MAX_FORWARD_RATE)
            randWalkAngZ = randomFloat(-MAX_TURNING_RATE, MAX_TURNING_RATE)
        end
        v = randWalkLinX
        w = randWalkAngZ
    end

    ankiState["randWalkCount"] = randWalkCount
    ankiState["randWalkLinX"] = randWalkLinX
    ankiState["randWalkAngZ"] = randWalkAngZ

    return v, w
end

function determineLeaderControl(ankiState)
    local v, w = 0,0
    currWaypoint = WAYPOINTS[ankiState["wpIndex"]]

    -- ankiPosition is a table. Access x,y,z by ankiPosition["x"]  etc
    ankiPosition = ankiState["position"]

    -- ankiOrientation is a table. Access x,y,z by ankiOrientation["x"]  etc
    ankiOrientation = ankiState["orientation"]


    omega = ankiOrientation["z"] --'z' omega angular heading
    dx = currWaypoint[1] - ankiPosition["x"] 
    dy = currWaypoint[2] - ankiPosition["y"] 
    theta = math.atan2(dy, dx)
    omegaError = angleDiff(theta,omega)
    if math.abs(omegaError) > OMEGA_DEADBAND then
        kp_omega = 1.2

        w = kp_omega * omegaError 
    end

    distError = math.sqrt(dx*dx + dy*dy) 
    steeringAllowed = math.abs(omegaError) < STEERING_BAND
    if distError > DIST_DEADBAND and steeringAllowed then
        kp_dist = 0.8 

        v = kp_dist * distError 
    end
    v = clamp(v, FORWARD_RATE,-FORWARD_RATE)

    if distError <= ACHIEVEMENT_THRESHOLD then
        ankiState["wpIndex"] = ankiState["wpIndex"] + 1
        if ankiState["wpIndex"] > NUM_WAYPOINTS then
            ankiState["wpIndex"] = 1
        end
    end


    return v, w
end


-- Function responsiple for taking in the anki state and determing output controls, v,w
function determineMotionControl(ankiState)
    if ankiState["agentType"] == "simple" then
        return determineSimpleControl(ankiState)
    elseif ankiState["agentType"] == "leader" then
        return determineLeaderControl(ankiState)
    else
        return 0,0
    end
end


function determineLightControl(ankiState)
    local lightCycleStartTime = ankiState["lightCycleStartTime"]
    local lightDutyCycle = ankiState["lightDutyCycle"]
    local simTime = sim.getSimulationTime()

    local lightElapsedTime = simTime - lightCycleStartTime

    if (lightElapsedTime >= LIGHT_PERIOD) then
        ankiState["lightCycleStartTime"] = simTime
    end
    if (lightElapsedTime <= LIGHT_PERIOD*lightDutyCycle) then
        -- light should be on
        if (ankiState['emitter'] == false) then
            --Turn light on             
            return true
        end
    else
        -- light should be off
        if (ankiState['emitter'] == true) then
            -- Turn light off
            return false
        end
    end
end


-- Retrieve the global position/orientation and format it as a twistStamped msg
function getStateGlobal(robotId, objHandle, relTo)
    t = simROS2.getSimulationTime() -- simROS2.getSystemTime()
    p = sim.getObjectPosition(objHandle, relTo)
    o = sim.getObjectOrientation(objHandle, relTo)

    return {
        header = {
            stamp = t,
            frame_id = tostring(robotId),
        },
        twist = {
            -- For now we are only returning the x and y positions and the z orientation
            --linear = { x = (p[1]+0.6096)*1000, y = (p[2]+0.4826)*1000, z = 0}, -- rough correction to align with camera data
            linear = { x = p[1], y = p[2], z = 0},
            angular = { x = 0, y = 0, z = o[3]}
        }
    }
end

-- Retrieve the proximity sensors reading and format as a float msg
function getStateProximity(proximityHandle)
    local result, distance = sim.readProximitySensor(proximityHandle)
    if result then
        if result == 0 then
            -- no object detected, means open space 
            return { data = MAX_PROXIMITY_DIST } 
        elseif result == 1 then
            -- object detected, so we have a distance to report
            if distance < MIN_PROXIMITY_DIST then
                distance = MIN_PROXIMITY_DIST
            end
            return { data = distance }
        end
    end
    -- There was either no result (maybe bad handle) or an error from the sensor
    return { data = -1 }
end
