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
                prev_orientation={x=0,y=0,z=0},
                }
    return blimpState
end
function ctrlsFromInput(vec)
    -- must return in order frontLeftMF,backRightMF,frontRightMF,backLeftMF,bMotorForce
    -- found in child script
    use_head=vec[4] -- bit that decides whether to use heading control
    --TODO: change the child script to take twist[x] input or something
    if (use_head==0) then
        --return ctrlsFromForceLocal({vec[1],vec[2],vec[3]},.5)
        return ctrlsFromVel({vec[1],vec[2],vec[3]})
    else
        --return ctrlsFromForceHeadLocal(vec,.5)
        return ctrlsFromVelHead(vec,.5)
    end


    --use this one for just position controls
    --return ctrlsFromVel({vec[1],vec[2],vec[3]})

end

function ctrlsFromVelHead(vec,CARE)
    linFLMF,linBRMF, linFRMF,linBLMF, bMotorForce = ctrlsFromVel({vec[1],vec[2],vec[3]})
    angFLMF,angBRMF,angFRMF,angBLMF=ctrlsFromHead(vec[6],CARE)
    FLMF=linFLMF+angFLMF
    BRMF=linBRMF+angBRMF
    FRMF=linFRMF+angFRMF
    BLMF=linBLMF+angBLMF
    mm=math.max(math.abs(FLMF),math.abs(BRMF),math.abs(FRMF),math.abs(BLMF))
    if mm>MOTOR_MAX_FORCE then
        fact=MOTOR_MAX_FORCE/mm
        FLMF=FLMF*fact
        BRMF=BRMF*fact
        FRMF=FRMF*fact
        BLMF=BLMF*fact
    end
    return FLMF,BRMF,FRMF,BLMF,bMotorForce
end
function ctrlsFromHead(head,CARE)
    --CARE: at most this proportion of the motor energy goes to rotating blimp
    omega = blimpState["orientation"]["z"] -- radians
    --vel_omega = (omega-blimpState["prev_orientation"]["z"])/SIM_DT

    -- angle to rotate
    theta = angleDiff(head,omega)
    actuation=(theta/PI)*MOTOR_MAX_FORCE*CARE
    --blimpState["prev_orientation"]["x"]=blimpState["orientation"]["x"]
    --blimpState["prev_orientation"]["y"]=blimpState["orientation"]["y"]
    blimpState["prev_orientation"]["z"]=blimpState["orientation"]["z"]
    return -actuation,-actuation,actuation,actuation
    --frontLeftMF,backRightMF,frontRightMF,backLeftMF
end
function ctrlsFromForceHeadLocal(vec,CARE)
    linFLMF,linBRMF, linFRMF,linBLMF, bMotorForce = ctrlsFromForceLocal({vec[1],vec[2],vec[3]})
    angFLMF,angBRMF,angFRMF,angBLMF=ctrlsFromHeadLocal(vec[6],CARE)
    FLMF=linFLMF+angFLMF
    BRMF=linBRMF+angBRMF
    FRMF=linFRMF+angFRMF
    BLMF=linBLMF+angBLMF
    mm=math.max(math.abs(FLMF),math.abs(BRMF),math.abs(FRMF),math.abs(BLMF))
    if mm>MOTOR_MAX_FORCE then
        fact=MOTOR_MAX_FORCE/mm
        FLMF=FLMF*fact
        BRMF=BRMF*fact
        FRMF=FRMF*fact
        BLMF=BLMF*fact
    end
    return FLMF,BRMF,FRMF,BLMF,bMotorForce
end
function ctrlsFromHeadLocal(theta,CARE)
    --theta: angle to rotate
    --CARE: at most this proportion of the motor energy goes to rotating blimp

    actuation=(theta/PI)*MOTOR_MAX_FORCE*CARE
    return -actuation,-actuation,actuation,actuation
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
    blimpState["prev_position"]["x"]=(1-MEM)*xp+MEM*old_position["x"]
    blimpState["prev_position"]["y"]=(1-MEM)*yp+MEM*old_position["y"]
    blimpState["prev_position"]["z"]=(1-MEM)*zp+MEM*old_position["z"]
    return ctrlsFromForce({target_vel[1]-vel_x,target_vel[2]-vel_y,target_vel[3]-vel_z})
end

function ctrlsFromForceLocal(target_force)
    EFFECT={18.634321312,18.578009728}
    -- effect of xy motors and bottom motors (i.e. if effect of bottom motor is 2, an input of x will result in acceleration 2x)
    HOVER_FORCE=.00058644
    local frontLeftForce, frontRightForce, bMotorForce = 0,0,0

    bMotorForce = HOVER_FORCE + target_force[3]/EFFECT[2]

    fx = target_force[1]
    fy = target_force[2]
    theta = math.atan2(fy, fx) -- head of blimp
    norm=math.sqrt(fx*fx+fy*fy)
    FL_component=math.cos(theta-PI/4)
    FR_component=math.cos(theta+PI/4)
    frontLeftForce=FL_component*norm/EFFECT[1]
    frontRightForce=FR_component*norm/EFFECT[1]



    if math.abs(frontLeftForce)>MOTOR_MAX_FORCE then
        -- shrink both right and left by a factor so we can maintain same xy control direction
        fact=MOTOR_MAX_FORCE/math.abs(frontLeftForce)
        frontLeftForce=frontLeftForce*fact
        frontRightForce=frontRightForce*fact
    end
    if math.abs(frontRightForce)>MOTOR_MAX_FORCE then
        -- shrink both right and left by a factor so we can maintain same xy control direction
        fact=MOTOR_MAX_FORCE/math.abs(frontRightForce)
        frontRightForce=frontRightForce*fact
        frontLeftForce=frontLeftForce*fact
    end
    -- we can handle z separately, just clamp it
    bMotorForce = clamp(bMotorForce, BMOTOR_MAX_FORCE, -BMOTOR_MAX_FORCE)
    -- must return in order frontLeftMF,backRightMF,frontRightMF,backLeftMF,bMotorForce

    return frontLeftForce,-frontLeftForce, frontRightForce,-frontRightForce, bMotorForce
end

function ctrlsFromForce(target_force)
    EFFECT={18.634321312,18.578009728}
    -- effect of xy motors and bottom motors (i.e. if effect of bottom motor is 2, an input of x will result in acceleration 2x)
    HOVER_FORCE=.00058644
    local frontLeftForce, frontRightForce, bMotorForce = 0,0,0

    -- blimpOrientation is a table. Access x,y,z by blimpOrientation["x"]  etc
    blimpOrientation = blimpState["orientation"]

    bMotorForce = HOVER_FORCE + target_force[3]/EFFECT[2]

    omega = blimpOrientation["z"] --'z' omega angular heading

    fx = target_force[1]
    fy = target_force[2]
    theta = math.atan2(fy, fx) -- head of blimp
    norm=math.sqrt(fx*fx+fy*fy)
    omegaError = angleDiff(theta,omega)
    FL_component=math.cos(omegaError-PI/4)
    FR_component=math.cos(omegaError+PI/4)
    frontLeftForce=FL_component*norm/EFFECT[1]
    frontRightForce=FR_component*norm/EFFECT[1]



    if math.abs(frontLeftForce)>MOTOR_MAX_FORCE then
        -- shrink both right and left by a factor so we can maintain same xy control direction
        fact=MOTOR_MAX_FORCE/math.abs(frontLeftForce)
        frontLeftForce=frontLeftForce*fact
        frontRightForce=frontRightForce*fact
    end
    if math.abs(frontRightForce)>MOTOR_MAX_FORCE then
        -- shrink both right and left by a factor so we can maintain same xy control direction
        fact=MOTOR_MAX_FORCE/math.abs(frontRightForce)
        frontRightForce=frontRightForce*fact
        frontLeftForce=frontLeftForce*fact
    end
    -- we can handle z separately, just clamp it
    bMotorForce = clamp(bMotorForce, BMOTOR_MAX_FORCE, -BMOTOR_MAX_FORCE)
    -- must return in order frontLeftMF,backRightMF,frontRightMF,backLeftMF,bMotorForce

    return frontLeftForce,-frontLeftForce, frontRightForce,-frontRightForce, bMotorForce
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
