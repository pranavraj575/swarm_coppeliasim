require 'quadHeader'
require 'rosMsg'
require 'common'

function createRecords()
    prevEuler=0
    lastE=0
    psp2=0
    psp1=0
end

function createStateTable()
    state = {
                position={x=0,y=0,z=0},
                prev_position={x=0,y=0,z=0},
                orientation={x=0,y=0,z=0},
                prev_orientation={x=0,y=0,z=0},
                }
    return state
end

function thrustFromInputVec(vec)

    s=sim.getObjectSizeFactor(quad_base)

    pos=sim.getObjectPosition(quad_base,-1)

    -- Vertical control:
    -- targetPos=sim.getObjectPosition(targetObj,-1)
    -- pos=sim.getObjectPosition(quad_base,-1)
    l=sim.getVelocity(heli)
    e=vec[3] --(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e

    -- Horizontal control:
    --sp=__getObjectPosition__(targetObj,quad_base)
    sp={vec[1],vec[2],vec[3]}
    m=sim.getObjectMatrix(quad_base,-1)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]

    -- Rotational control:
    --euler=__getObjectOrientation__(quad_base,targetObj)
    euler={vec[4],vec[5],vec[6]}
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]

    -- Decide of the motor velocities:
    out={0,0,0,0}
    out[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    out[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    out[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    out[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
    return out
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