import numpy as np
import scipy as sp
import scipy.interpolate

def mix_inputs(u):
    tau = actuation_vector_saturation(u)
    f = mixer_positive(tau)
    duty_cycles = thrust2dutyCycle(f)
    duty_cycles = duty_cycle_saturation(duty_cycles)
    duty_cycles = direction_and_order_mapping(duty_cycles)
    
    return duty_cycles

def actuation_vector_saturation(user_tau):
    # diamond force constraint
    # function for constraining user/controller input force by force end loop
    # define single motor maximum force
    fmax_xy = 0.0594
    fmax_z_up = 0.0454
    fmax_z_down = -0.0183

    thrust_ratio = 0.8 # change if you want to adjust maximum usable thrust
    torque_ratio = 0.2 # change if you want to adjust maximum usable thrust
    f_max_xy = fmax_xy * thrust_ratio * np.sqrt(2) # maximum force along x or y direction
    f_z_max_up = fmax_z_up*2 # maximum force along -z direction, up
    f_z_max_down = fmax_z_down*2 # maximum force along +z direction, down
    tau_max =  0.0594 * torque_ratio * 2 * 0.053
    fx = user_tau[0]
    fy = user_tau[1]
    fz = user_tau[2]
    tauz = user_tau[5]

    if fy != 0:
       theta = np.arctan(fx/fy)

    else:
       theta = 0

    # saturation for fx and fy by diamond shape
    ratio_xy = np.sqrt(fx**2 + fy**2) / f_max_xy*(np.sqrt((np.cos(np.radians(theta)))**2 + (1 - np.cos(np.radians(theta)))**2));
    if ratio_xy > 1:
        fx = fx / ratio_xy
        fy = fy / ratio_xy

    # saturation for fz
    if fz > 0:
        ratio_z = abs(fz / f_z_max_down)
    else:
        # if fz < 0
        ratio_z = abs(fz / f_z_max_up)

    if ratio_z > 1:
        fz = fz / ratio_z

    # saturation for tau_z
    ratio_tau = abs(tauz / tau_max)

    if ratio_tau > 1:
        tauz = tauz / ratio_tau

    tau = user_tau
    tau[0] = fx
    tau[1] = fy
    tau[2] = fz
    tau[5] = tauz

    return tau

def mixer_positive(tau):
    # extract desired control output
    fx = tau[0]
    fy = tau[1]
    fz = tau[2]
    tz = tau[5]
    dc = 0.053

    #mixer matrix multiplication
    f1 = -np.sqrt(2)*fx*np.heaviside(-fx, 0.5)/2 + np.sqrt(2)*fy*np.heaviside(fy, 0.5)/2 + 2*tz*np.heaviside(tz, 0.5)/dc
    f2 = np.sqrt(2)*fx*np.heaviside(fx, 0.5)/2 + np.sqrt(2)*fy*np.heaviside(fy, 0.5)/2 - 2*tz*np.heaviside(-tz, 0.5)/dc
    f3 = np.sqrt(2)*fx*np.heaviside(fx, 0.5)/2 - np.sqrt(2)*fy*np.heaviside(-fy, 0.5)/2 + 2*tz*np.heaviside(tz, 0.5)/dc
    f4 = -np.sqrt(2)*fx*np.heaviside(-fx, 0.5)/2 - np.sqrt(2)*fy*np.heaviside(-fy, 0.5)/2 - 2*tz*np.heaviside(-tz, 0.5)/dc
    f5 = -fz/2
    f6 = -fz/2
    f = [f1, f2, f3, f4, f5, f6]

    return f

def thrust2dutyCycle(tau):
    ## Function for convert thrust to corresponding duty cycles according to our measurements
    # INPUT: force --6 by 1 vector containing the thrust required for each motor: [F1,F2,F3,F4,F5,F6]
    #       --F1: thrust required for motor 1, in Newton (N), upper-right motor in x-y plane
    #       --F2: thrust required for motor 2, in Newton (N), lower-right motor in x-y plane
    #       --F3: thrust required for motor 3, in Newton (N), lower-left motor in x-y plane
    #       --F4: thrust required for motor 4, in Newton (N), upper-left motor in x-y plane
    #       --F5: thrust required for motor 5, in Newton (N), right vertical
    #       motor
    #       --F6: thrust required for motor 6, in Newton (N), left vertical
    #       motor
    #
    # OUTPUT: dutyCycle --6 by 1 vector containing the duty cycle for each
    # motor: [d1, d2, d3, d4, d5, d6]
    #       --d1: duty cycle count required for motor 1, 0<d1<250
    #       --d2: duty cycle count required for motor 2, 0<d2<250
    #       --d3: duty cycle count required for motor 3, 0<d3<250
    #       --d4: duty cycle count required for motor 4, 0<d4<250
    #       --d5: duty cycle count required for motor 5, 0<d5<250
    #       --d6: duty cycle count required for motor 6, 0<d6<250
    #
    # Use example: dutyCycle = force2dutyCycle(tau)

    ## Define measurements for vertical (z axis) and horizontal (x-y plane) thrusters
    
    # duty cycle for vertical thrusters
    d_z = np.array([-1, -.8, -.6, -.4, -.2, 0, .2, .4, .6, .8, 1])
    
    # measured thrust for vertical thrusters, in Newton (N)
    f_z = 9.8 * 0.001 * np.array([-1.87, -1.43, -0.991, -0.551, -0.11, 0, 0.22, 1.1, 2.2, 3.3, 4.63])

    # duty cycle for horizontal thrusters
    d_xy = np.array([0, .2, .3, .4, .5, .6, .7, .8, .9, 1])
    
    # measured thrust for horizontal thrusters, in Newton (N)
    f_xy = 9.8 * 0.001 * np.array([0, 0.33, 0.771, 1.43, 2.09, 2.75, 3.52, 4.52, 5.18, 6.06])

    ## Extract all thrust values for horizontal (motor 1-4) and vertical (motor 5-6) thrusters
    # force_xy is a 4 by 1 array containing all required thrust for horizontal motors
    force_xy = tau[:4]

    # force_z is a 2 by 1 array containing all required thrust for vertical motors 
    force_z = tau[4:]
    
    ## clip force into the boundary boundary defined by our measurements
    force_z_limited = np.maximum(np.minimum(force_z, np.ones(2)*np.max(f_z)), np.ones(2)*np.min(f_z))
    force_xy_limited = np.maximum(np.minimum(force_xy, np.ones(4)*np.max(f_xy)), np.ones(4)*np.min(f_xy))

    ## convert duty cycle to thrust by linear interpolation of our measurements
    dutyCycle_xy = sp.interpolate.interp1d(f_xy, d_xy)(force_xy_limited)
    dutyCycle_z = sp.interpolate.interp1d(f_z, d_z)(force_z_limited)
    
    ## Return the result after the conversion
    return np.concatenate([dutyCycle_xy, dutyCycle_z])

def duty_cycle_saturation(duty_cycles):
    ## Function to make sure duty cycles fall in the range of [-1,1]
    # INPUT: duty_cycles --6 by 1 vector representing the required
    # duty cycle for each motor: [d1, d2, d3, d4, d5, d6]
    #       --d1: duty cycle count required for motor 1, 0<d1<250
    #       --d2: duty cycle count required for motor 2, 0<d2<250
    #       --d3: duty cycle count required for motor 3, 0<d3<250
    #       --d4: duty cycle count required for motor 4, 0<d4<250
    #       --d5: duty cycle count required for motor 5, 0<d5<250
    #       --d6: duty cycle count required for motor 6, 0<d6<250
    #
    #
    # OUTPUT: duty_cycles --6 by 1 vector representing the saturated duty cycle
    # vector: [d1, d2, d3, d4, d5, d6]
    #       --d1: duty cycle count required for motor 1, 0<d1<250
    #       --d2: duty cycle count required for motor 2, 0<d2<250
    #       --d3: duty cycle count required for motor 3, 0<d3<250
    #       --d4: duty cycle count required for motor 4, 0<d4<250
    #       --d5: duty cycle count required for motor 5, 0<d5<250
    #       --d6: duty cycle count required for motor 6, 0<d6<250

    ## Make sure the duty cycles stay within the range
    return np.clip(duty_cycles, -1, 1)

def direction_and_order_mapping(duty_cycles):
    ## Function for mapping the duty cycles generated to match up with actual directions and orders of motors
    # INPUT: duty_cycles --6 by 1 vector representing the duty cycle for each
    # motor: [d1, d2, d3, d4, d5, d6]
    #       --d1: duty cycle count required for motor 1, -1<d1<1
    #       --d2: duty cycle count required for motor 2, -1<d2<1
    #       --d3: duty cycle count required for motor 3, -1<d3<1
    #       --d4: duty cycle count required for motor 4, -1<d4<1
    #       --d5: duty cycle count required for motor 5, -1<d5<1
    #       --d6: duty cycle count required for motor 6, -1<d6<1
    #
    # OUTPUT: duty_cycles --6 by 1 vector representing the duty cycles for each
    # motor after direction and order mapping: [d1, d2, d3, d4, d5, d6]
    #       --d1: duty cycle count required for motor 1, -1<d1<1
    #       --d2: duty cycle count required for motor 2, -1<d2<1
    #       --d3: duty cycle count required for motor 3, -1<d3<1
    #       --d4: duty cycle count required for motor 4, -1<d4<1
    #       --d5: duty cycle count required for motor 5, -1<d5<1
    #       --d6: duty cycle count required for motor 6, -1<d6<1
    #
    #
    ## Map to real directions (for all 6 motors, -1 means propelling forward (optimized direction))
    # Define the direction mapping matrix
    direction_matrix = np.diag([-1,-1,-1,-1,-1,-1]);


    ## Map to real orders
    # Motor 1: upper-right horizontal motor in our convention, upper-left horizontal motor in actual
    # configuration, corresponds with motor 4 in actual configuration
    # Motor 2: lower-right horizontal motor in our convention, upper-right horizontal motor in actual
    # configuration, corresponds with motor 1 in actual configuration
    # Motor 3: lower-left horizontal motor in our convention, right vertical
    # motor in actual configuration, corresponds with motor 5 in actual
    # configuration
    # Motor 4: upper-left horizontal motor in our convention, left vertical
    # motor in actual configuration, corresponds with motor 6 in actual
    # configuration
    # Motor 5: right vertical motor in our convention, lower-right horizontal
    # motor in actual configuration, corresponds with motor 2 in actual
    # configuration
    # Motor 6: left vertical motor in our convention, lower-left horizontal
    # motor in actual configuration, corresponds with motor 3 in actual
    # configuration

    # Define the order mapping matrix
    order_matrix = np.array([[0, 0, 0, 1, 0, 0],
                             [1, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 0, 1],
                             [0, 1, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 0]])

    ## Apply the direction and order mapping accordingly
    return order_matrix @ direction_matrix @ duty_cycles

def convertCMD(motor_duty_cycles, id_num):
    throttle = np.round(motor_duty_cycles * 250)
    di = throttle > 0
    direction = di[0]*32 + di[1]*16 + di[2]*8 + di[3]*4 + di[4]*2 + di[5]*1
    throttle = abs(throttle)
    
    if id_num is None:
        checksum = ((sum(throttle) + direction) % 4) + 251
        output = np.concatenate([[255], throttle, [direction], [checksum]])

    else:
        checksum = ((sum(throttle) + direction + id_num) % 4) + 251
        output = np.concatenate([[255], throttle, [direction], [id_num], [checksum]])

    return output


