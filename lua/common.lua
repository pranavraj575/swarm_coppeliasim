-- This file contains some common math definitions and functions
PI = 3.14159
--gets port, defaults to 23000 if nil
function getZMQport()
    port=sim.getNamedInt32Param('zmqRemoteApi.rpcPort')
    if port==nil then
        return 23000
    else
        return port
    end
end

-- Clamping function to saturate the input
function clamp(val, maxVal, minVal)
    if val >= maxVal   then
        return maxVal
    elseif val <= minVal then
        return minVal
    else
        return val
    end
end

function radsToDegs(rads)
    return rads*180.0/PI
end

function degsToRads(degs)
    return degs*PI/180.0
end

function linToAng(vel, wheelDiam)
    return vel*2.0/wheelDiam
end

function angToLin(rads, wheelDiam)
    return rads*wheelDiam/2.0
end

-- Returns the difference between two angles and keeps result in [-pi,pi] range
function angleDiff(leftAngle, rightAngle)
    local diff = leftAngle - rightAngle
    if(math.abs(diff) > PI) then
        if diff > 0 then
            diff = diff - 2*PI
        else
            diff = diff - -2*PI
        end
    end
    return diff
end

function shallowCopy(orig)
    local orig_type = type(orig)
    local copy
    if orig_type == 'table' then
        copy = {}
        for orig_key, orig_value in pairs(orig) do
            copy[orig_key] = orig_value
        end
    else
        copy = orig
    end
    return copy
end

function gaussian(mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end

function randomFloat(lower, upper)
    return lower + math.random()*(upper-lower)
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
