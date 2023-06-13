require 'blimpHeader'
require 'rosMsg'
require 'common'

-- force for 4 motors
function calculateInherentLiftForce4M()
    return calculateInherentLiftForce(4)
end
-- force for 6 motors (new blimps)
function calculateInherentLiftForce6M()
    return calculateInherentLiftForce(6)
end

-- Function to calculate and return the lift force provided by the envelope to help offset gravity
function calculateInherentLiftForce(motors)
    MASS_EST = (MASS_MOTOR*motors) + MASS_BODY + MASS_GONDOLA + MASS_BATTERY + MASS_BALLAST + MASS_ADJUSTMENT
    return {0,0,MASS_EST*-GRAVITY}
end

-- Function to calculate and return an estimated drag force
function calculateEstimatedDragForce(blimpHandle)
    local linVel, angVel = sim.getVelocity(blimpHandle)

    local abs2DVel = math.sqrt(linVel[1]*linVel[1] + linVel[2]*linVel[2])
    local theta = math.atan2(linVel[2], linVel[1])

    -- local airDensity = 1.22 --kg/(m^3)
    -- local crossArea = 0.19949
    -- local coefficient = 0.4
    -- local forceDrag = 0.5 * airDensity * abs2DVel * abs2DVel * coefficient * crossArea
    -- no longer need these coefficients as we can calculate the coefficient based on real world data

    local forceDrag = DRAG_FORCE_COEFFICIENT * abs2DVel * abs2DVel
    local xForce = -1 * forceDrag * math.cos(theta)
    local yForce = -1 * forceDrag * math.sin(theta)
    -- print(string.format("vel: %.4f | fxdrag:%.3f | y drag:%.3f", abs2DVel, xForce, yForce))
    return {xForce,yForce,0}
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

function createStateTable()
    blimpState = {ultrasonic=0,
                position={x=0,y=0,z=0},
                prev_position={x=0,y=0,z=0},
                orientation={x=0,y=0,z=0},
                prevHeightError = 0,
                prevOmegaError = 0,
                prevDistError = 0}
    return blimpState
end
function ctrlsFromInput(vec)
    return ctrlsFromVel(vec)
end
function ctrlsFromVelxy(target_vel)
    MEM=.9
    HOVER_FORCE=.0315
    blimpPosition = blimpState["position"]
    old_position=blimpState["prev_position"]
    xp,yp=blimpPosition["x"],blimpPosition["y"]
    blimpUltrasonic = blimpState['ultrasonic']
    zp=blimpUltrasonic + ULTRASONIC_Z_OFFSET
    vel_x=(xp-old_position["x"])/SIM_DT
    vel_y=(yp-old_position["y"])/SIM_DT
    vel_z=(zp-old_position["z"])/SIM_DT
    old_position["x"]=(1-MEM)*xp+MEM*old_position["x"]
    old_position["y"]=(1-MEM)*yp+MEM*old_position["y"]
    old_position["z"]=(1-MEM)*zp+MEM*old_position["z"]
    return ctrlsFromForce2({target_vel[1]-vel_x,target_vel[2]-vel_y,target_vel[3]+HOVER_FORCE})
end

function ctrlsFromVel(target_vel)
    MEM=0.
    blimpPosition = blimpState["position"]
    old_position=blimpState["prev_position"]
    xp,yp=blimpPosition["x"],blimpPosition["y"]
    blimpUltrasonic = blimpState['ultrasonic']
    zp=blimpUltrasonic + ULTRASONIC_Z_OFFSET
    zp=blimpPosition["z"]
    vel_x=(xp-old_position["x"])/SIM_DT
    vel_y=(yp-old_position["y"])/SIM_DT
    vel_z=(zp-old_position["z"])/SIM_DT
    old_position["x"]=(1-MEM)*xp+MEM*old_position["x"]
    old_position["y"]=(1-MEM)*yp+MEM*old_position["y"]
    old_position["z"]=(1-MEM)*zp+MEM*old_position["z"]
    return ctrlsFromForce3({target_vel[1]-vel_x,target_vel[2]-vel_y,target_vel[3]-vel_z})
end
function ctrlsFromForce3(target_force)
    EFFECT={18.854802558,18.869096405}
    HOVER_FORCE=.00063043
    local lMotorForce, rMotorForce, bMotorForce = 0,0,0

    -- blimpOrientation is a table. Access x,y,z by blimpOrientation["x"]  etc
    blimpOrientation = blimpState["orientation"]

    bMotorForce = HOVER_FORCE + target_force[3]/EFFECT[2]

    omega = blimpOrientation["z"] --'z' omega angular heading

    fx = target_force[1]
    fy = target_force[2]
    theta = math.atan2(fy, fx) -- head of blimp
    omegaError = angleDiff(theta,omega)

    forward=math.cos(omegaError)
    if forward<0 then
        omegaError = angleDiff(theta,omega+PI)
        -- reverse controls
    end

    left_skew=math.sin(omegaError)

    prevOmegaError = blimpState["prevOmegaError"]
    -- Note: Should take about 3-5 seconds to spin 90 degrees
    if math.abs(omegaError) > OMEGA_DEADBAND then
        derivative = (omegaError - prevOmegaError) / SIM_DT

        kp_omega = 0.15
        kd_omega = 0.17
        ang_adj = kp_omega * omegaError + kd_omega * derivative
        rMotorForce = ang_adj
        lMotorForce = -rMotorForce
    end
    blimpState["prevOmegaError"] = omegaError


    steeringAllowed = math.abs(omegaError) < STEERING_BAND
    -- steeringAllowed = true
    if steeringAllowed then

        adj = math.sqrt(fx*fx + fy*fy)/EFFECT[1]
        if forward<0 then -- reverse if necessary
            adj=-adj
        end
        rMotorForce = rMotorForce + adj
        lMotorForce = lMotorForce + adj
    end
    condInt = steeringAllowed and 1 or 0

    if math.abs(rMotorForce)>RMOTOR_MAX_FORCE then
        -- shrink both right and left by a factor so we can maintain same xy control direction
        fact=RMOTOR_MAX_FORCE/math.abs(rMotorForce)
        lMotorForce=lMotorForce*fact
        rMotorForce=rMotorForce*fact
    end
    if math.abs(lMotorForce)>LMOTOR_MAX_FORCE then
        -- shrink both right and left by a factor so we can maintain same xy control direction
        fact=LMOTOR_MAX_FORCE/math.abs(lMotorForce)
        rMotorForce=rMotorForce*fact
        lMotorForce=lMotorForce*fact
    end
    -- we can handle z separately, just clamp it
    bMotorForce = clamp(bMotorForce, BMOTOR_MAX_FORCE, -BMOTOR_MAX_FORCE)
    return lMotorForce, rMotorForce, bMotorForce
end
function ctrlsFromForce2(target_force)
    return ctrlsFromCam(target_force,true)
end
function ctrlsFromForce(target_force)
    return ctrlsFromCam(target_force,false)
end
function ctrlsFromCam(target_force,reverse_allowed)

    local lMotorForce, rMotorForce, bMotorForce = 0,0,0

    -- blimpOrientation is a table. Access x,y,z by blimpOrientation["x"]  etc
    blimpOrientation = blimpState["orientation"]

    blimpUltrasonic = blimpState['ultrasonic']
    prevHeightError = blimpState["prevHeightError"]
    heightError = target_force[3]
    -- Note: should take about 5-10 seconds to rise 1m delta
    if math.abs(heightError) > ALTITUDE_DEADBAND then
        derivative = (heightError - prevHeightError) / SIM_DT

        kp_height = 0.02
        kd_height = 0.07

        bMotorForce = kp_height * heightError + kd_height * derivative
    end
    -- update state with relevant controller terms
    blimpState["prevHeightError"] = heightError

    omega = blimpOrientation["z"] --'z' omega angular heading
    dx = target_force[1]
    dy = target_force[2]
    theta = math.atan2(dy, dx)
    omegaError = angleDiff(theta,omega)
    forward=math.cos(omegaError)
    left_skew=math.sin(omegaError)
    if forward<0 and reverse_allowed then
        left_skew = -left_skew
        omegaError = angleDiff(theta,omega+PI)
        -- reverse controls
    end
    prevOmegaError = blimpState["prevOmegaError"]
    -- Note: Should take about 3-5 seconds to spin 90 degrees
    if math.abs(omegaError) > OMEGA_DEADBAND then
        derivative = (omegaError - prevOmegaError) / SIM_DT

        kp_omega = 0.15
        kd_omega = 0.17
        ang_adj = kp_omega * omegaError + kd_omega * derivative
        rMotorForce = ang_adj
        lMotorForce = -rMotorForce
    end
    blimpState["prevOmegaError"] = omegaError

    distError = math.sqrt(dx*dx + dy*dy) -- DIST_DEADBAND
    prevDistError = blimpState["prevDistError"]
    steeringAllowed = math.abs(omegaError) < STEERING_BAND
    if distError > DIST_DEADBAND and steeringAllowed then
        derivative = (distError - prevDistError) / SIM_DT

        kp_dist = 0.01 --0.03
        kd_dist = 0.09 --0.09

        adj = kp_dist * distError + kd_dist * derivative
        if forward<0 and reverse_allowed then -- reverse if necessary
            adj=-adj
        end
        rMotorForce = rMotorForce + adj
        lMotorForce = lMotorForce + adj
    end
    blimpState["prevDistError"] = distError
    condInt = steeringAllowed and 1 or 0

    -- TODO: why would you change the vector here, do a smarter clamp thingy
    rMotorForce = clamp(rMotorForce, RMOTOR_MAX_FORCE, -RMOTOR_MAX_FORCE)
    lMotorForce = clamp(lMotorForce, LMOTOR_MAX_FORCE, -LMOTOR_MAX_FORCE)
    bMotorForce = clamp(bMotorForce, BMOTOR_MAX_FORCE, -BMOTOR_MAX_FORCE)

    return lMotorForce, rMotorForce, bMotorForce
end

-- Function responsiple for taking in the blimp state and determing output controls
-- The output controls are units of force for each of the left, right, and bottom motors
function determineControls(blimpState)
    local lMotorForce, rMotorForce, bMotorForce = 0,0,0

    -- blimpPosition is a table. Access x,y,z by blimpPosition["x"]  etc
    blimpPosition = blimpState["position"]

    -- blimpOrientation is a table. Access x,y,z by blimpOrientation["x"]  etc
    blimpOrientation = blimpState["orientation"]

    -- Try and see if there is a goal waypoint
    goalHandle = sim.getObject("/Goal",{noError=true})
    if goalHandle == -1 then
        -- Goal does not exist, do other behavior
        -- for now, do nothing, return 0s
        return lMotorForce, rMotorForce, bMotorForce
    end

    goalPosition = sim.getObjectPosition(goalHandle, -1)
    altitudeHoldingHeight = goalPosition[3]
    -- print(altitudeHoldingHeight)

    blimpUltrasonic = blimpState['ultrasonic']
    prevHeightError = blimpState["prevHeightError"]
    heightError = altitudeHoldingHeight - (blimpUltrasonic + ULTRASONIC_Z_OFFSET)
    -- Note: should take about 5-10 seconds to rise 1m delta
    if math.abs(heightError) > ALTITUDE_DEADBAND then
        kp_height = 0.0
        kd_height = 0.0
        derivative = (heightError - prevHeightError) / SIM_DT

        kp_height = 0.02
        kd_height = 0.07

        bMotorForce = kp_height * heightError + kd_height * derivative
    end
    -- update state with relevant controller terms
    blimpState["prevHeightError"] = heightError
    -- print(string.format("bZ:%.2f | U:%.2f | dH: %.2f prevdH: %.2f |der:%.4f | {%.4f,%.4f,%.4f}",
    --    blimpPosition["z"],blimpUltrasonic, heightError, prevHeightError, derivative, lMotorForce,rMotorForce,bMotorForce))


    omega = blimpOrientation["z"] --'z' omega angular heading
    dx = goalPosition[1] - blimpPosition["x"]
    dy = goalPosition[2] - blimpPosition["y"]
    theta = math.atan2(dy, dx)
    omegaError = angleDiff(theta,omega)
    prevOmegaError = blimpState["prevOmegaError"]
    -- Note: Should take about 3-5 seconds to spin 90 degrees
    if math.abs(omegaError) > OMEGA_DEADBAND then
        kp_omega = 0.0
        kd_omega = 0.0
        derivative = (omegaError - prevOmegaError) / SIM_DT

        kp_omega = 0.15
        kd_omega = 0.17

        rMotorForce = kp_omega * omegaError + kd_omega * derivative
        lMotorForce = -rMotorForce
    end
    blimpState["prevOmegaError"] = omegaError
    -- print(string.format("blimpOmega:%.2f | thetaToGoal:%.2f | dw: %.2f prevdw: %.2f |der:%.4f | {%.4f,%.4f,%.4f}",
    --     omega,theta, omegaError, prevOmegaError, derivative, lMotorForce,rMotorForce,bMotorForce))

    distError = math.sqrt(dx*dx + dy*dy) -- DIST_DEADBAND
    prevDistError = blimpState["prevDistError"]
    steeringAllowed = math.abs(omegaError) < STEERING_BAND
    if distError > DIST_DEADBAND and steeringAllowed then
        kp_dist = 0.0
        kd_dist = 0.0
        derivative = (distError - prevDistError) / SIM_DT

        kp_dist = 0.01 --0.03
        kd_dist = 0.09 --0.09

        adj = kp_dist * distError + kd_dist * derivative
        rMotorForce = rMotorForce + adj
        lMotorForce = lMotorForce + adj

    end
    blimpState["prevDistError"] = distError
    condInt = steeringAllowed and 1 or 0
    rMotorForce = clamp(rMotorForce, RMOTOR_MAX_FORCE, -RMOTOR_MAX_FORCE)
    lMotorForce = clamp(lMotorForce, LMOTOR_MAX_FORCE, -LMOTOR_MAX_FORCE)
    bMotorForce = clamp(bMotorForce, BMOTOR_MAX_FORCE, -BMOTOR_MAX_FORCE)

    -- lMotorForce = MOTOR_MAX_FORCE
    -- rMotorForce = lMotorForce

    -- print(string.format("blimpOmega:%.2f | thetaToGoal:%.2f | dw: %.2f | distErr: %.2f | goodOrient:%d | {%.4f,%.4f,%.4f}",
    --     omega,theta, omegaError, distError, condInt, lMotorForce,rMotorForce,bMotorForce))

    return lMotorForce, rMotorForce, bMotorForce
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
            linear = { x = p[1], y = p[2], z = p[3]},
            angular = { x = o[1], y = o[2], z = o[3]}
        }
    }
end

-- Retrieve the proximity sensors reading and format as a float msg
-- Done for special handling of how proximity sensor returns are returned by Coppeliasim
function getStateProximity(proximityHandle)
    local result, distance = sim.readProximitySensor(proximityHandle)
    if result then
        if result == 0 then
            -- no object detected, means open space, return the maximum detectable distance
            return { data = MAX_PROXIMITY_DIST }
        elseif result == 1 then
            -- object detected, so we have a distance to report
            if distance < MIN_PROXIMITY_DIST then
                distance = MIN_PROXIMITY_DIST
            end
            local gauss = gaussian(0,ULTRASONIC_NOISE_VARIANCE)
            -- local gauss = 0
            -- print(string.format("%.4f",gauss))
            return { data = distance + gauss }
        end
    end
    -- There was either no result (maybe bad handle) or an error from the sensor
    return { data = -1 }
end
