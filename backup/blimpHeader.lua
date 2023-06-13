-- This file contains constants that will be used by the blimps

GRAVITY = -9.81

-- Estimated masses of items attached to blimp, kg
MASS_MOTOR = 0.0007 --.005
MASS_BODY = 0.035 --.127 
MASS_BALLAST = MASS_MOTOR * 2.0 -- old ballast0.0035
MASS_GONDOLA= 0.0458 -- 71 g total mass: (4*MASS_MOTOR + MASS_BATTERY + MASS_BALLAST + MASS_GONDOLA)| MASS_GONDOLA = 0.071 - (6*0.0007 + 0.021) = 45.8 g
MASS_BATTERY = 0.021 --.013
MASS_WING = .001
MASS_ADJUSTMENT = -0.00015 -- 0.00079 -- rough adjustment to simulation masses to make blimp nuetrally boyuant/slightly negative
MASS_BLIMP_EST = (MASS_MOTOR*4) + MASS_BODY + MASS_GONDOLA + MASS_BATTERY + MASS_BALLAST + MASS_ADJUSTMENT
--simulation model mass: 0.10597853222862 kg
-- MASS_BLIMP_EST (before mass adjustment): 0.106 kg

MAX_PROXIMITY_DIST = 5.0 -- m, max distance the proximity sensor can detect 
MIN_PROXIMITY_DIST = 0.03 -- m, min distance the proximity sensor can detect 

--ALTITUDE_HOLDING_HEIGHT = 1.0 --m, height above ground to float at
-- Holding Height is now dependent on the z height of the goal position
ALTITUDE_DEADBAND = 0.001 -- m, delta from holding height in which no motor force is applie

MOTOR_MAX_FORCE = 0.02942 --0.127 -- could maximum measure 3 grams on scale, converted to Newtons
BMOTOR_MAX_FORCE = MOTOR_MAX_FORCE
LMOTOR_MAX_FORCE = MOTOR_MAX_FORCE
RMOTOR_MAX_FORCE = MOTOR_MAX_FORCE
SIM_DT = sim.getSimulationTimeStep()

ULTRASONIC_Z_OFFSET = 0.24
ULTRASONIC_NOISE_VARIANCE = 0.0003

OMEGA_DEADBAND = 0.01 -- rads, delta from orientation facing goal

DIST_DEADBAND = 0.01 -- m, delta from goalpoint, only 2d dist
STEERING_BAND = 5* 3.14/180 -- rads, band that allows forward motion if blimp is facing correct orientation


-- For air drag, applied x,y, we can calculate based on maximum velocity and apply 
-- an air drag force in a similar fashion as the bouyant force
-- equations derived from physics, Fnet = 0, Fdrag = Fpropellers, 
-- 1/2*p*C*A*VelMax^2 = 2*MOTOR_MAX_FORCE, just call  "1/2*p*C*A" the coefficient 'c'
-- c*VelMax^2 = 2*MOTOR_MAX_FORCE, c = (2*MOTOR_MAX_FORCE)/(VelMax^2)
-- there are 2 propellers facing forward
TERMINAL_DRAG_VELOCITY = 1.75 --m/s
DRAG_FORCE_COEFFICIENT = (2*MOTOR_MAX_FORCE)/(TERMINAL_DRAG_VELOCITY*TERMINAL_DRAG_VELOCITY)


 -- material property, angular damping of 3.6000e-01 works well 
 -- for psuedo air drag in terms of rocking motion, is no  
--[[
    Some alright tuning terms for height keeping
    PID Terms for raising from 1 to 2m
    kp_height = 0.011
    ki_height = 0.00025
    kd_height = 0.06

PD terms 
    kp_height = 0.02
    kd_height = 0.07

--]]