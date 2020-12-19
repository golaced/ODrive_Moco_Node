#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
from fibre import Logger, Event
from odrive.utils import OperationAbortedException
from fibre.protocol import ChannelBrokenException
import sys

def set_gains(odrv,axis):
    """
    Sets the nested PID gains to a good default
    """
    axis.controller.config.pos_gain = 100.0 #f [(counts/s) / counts]
    axis.controller.config.pos_gain = 0.01 #f [(counts/s) / counts]
    axis.controller.config.vel_gain = 5.0 / 10000.0 #[A/(counts/s)]
    axis.controller.config.vel_limit = 50000.0
    axis.controller.config.vel_integrator_gain = 0 #[A/((counts/s) * s)]

    axis.controller.config.kp_theta = 0.04 *  6000 / (2 * math.pi)
    axis.controller.config.kd_theta = 5.0 / 10000.0 *  6000 / (2 * math.pi)
    axis.controller.config.kp_gamma = 0.0 *  6000 / (2 * math.pi)
    axis.controller.config.kd_gamma = 5.0 / 10000.0 *  6000 / (2 * math.pi)

def calibrate_motor(odrv, axis):
    # time.sleep(0.5)
    print('Calibrating motor...',end='',flush=True)
    run_state(axis, AXIS_STATE_MOTOR_CALIBRATION, True)
    axis.motor.config.pre_calibrated = True# then set motor pre calibrated to true
    print('Done');

def calibrate_motor_and_zencoder(odrv,axis):
    """
    Runs motor and encoder calibration (with z index) and saves
    Enables closed loop control on startup and encoder search on startup
    """

    calibrate_motor(odrv, axis)

    print('Calibrating encoder...',end='',flush=True)
    axis.encoder.config.use_index = True
    run_state(axis, AXIS_STATE_ENCODER_INDEX_SEARCH, True)
    run_state(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, True)
    axis.encoder.config.pre_calibrated = True
    print('Done');

    print('Setting startup flags...',end='',flush=True)
    # axis.config.startup_encoder_index_search = True
    axis.config.startup_encoder_index_search = True
    axis.config.startup_closed_loop_control = True
    print('Done with axis.')

def run_state(axis, requested_state, wait):
    """
    Sets the requested state on the given axis. If wait is True, the method
    will block until the state goes back to idle.

    Returns whether the odrive went back to idle in the given timeout period,
    which is 10s by default.
    """
    axis.requested_state = requested_state
    timeout_ctr = 100; # 20s timeout for encoder calibration to finish
    if wait:
        while(timeout_ctr > 0): # waits until state goes back to idle to continue
            time.sleep(0.2)
            timeout_ctr -= 1
            if axis.current_state == AXIS_STATE_IDLE:
                break
    return timeout_ctr > 0

def reboot_odrive(odrv):
    """
    Reboot odrive
    """
    try:
        odrv.reboot()
    except ChannelBrokenException:
        print('Lost connection because of reboot...')

def reset_odrive(odrv):
    """
    Erase config
    """
    print('Erasing config and rebooting...')
    odrv.erase_configuration()
    reboot_odrive(odrv)

def init_odrive(odrv):
    """
    NOTE: does not save
    """
    print('Initializing ODrive...')
    odrv.config.brake_resistance = 0

    odrv.axis0.motor.config.pole_pairs = 11
    odrv.axis1.motor.config.pole_pairs = 11

    odrv.axis0.motor.config.resistance_calib_max_voltage = 4.0
    odrv.axis1.motor.config.resistance_calib_max_voltage = 4.0

    odrv.axis0.encoder.config.cpr = 4000
    odrv.axis1.encoder.config.cpr = 4000

    odrv.axis0.motor.config.current_lim = 25 #40
    odrv.axis1.motor.config.current_lim = 25 #40

    odrv.axis0.motor.config.calibration_current = 10
    odrv.axis1.motor.config.calibration_current = 10

    odrv.axis0.motor.config.pre_calibrated = False
    odrv.axis1.motor.config.pre_calibrated = False

    odrv.axis0.encoder.config.pre_calibrated = False
    odrv.axis1.encoder.config.pre_calibrated = False
    print('Done.')

def set_odrive_limits(odrv,axis,cur_lim):
    """
    Set motor current limits
    """
    axis.motor.config.current_lim = current_lim

def calibrate_odrive(odrv):
    """
    Calibrate motors and encoders (including index) on both axes
    """
    print('Calibrating axes...')

    calibrate_motor_and_zencoder(odrv,odrv.axis0)
    calibrate_motor_and_zencoder(odrv,odrv.axis1)
    print('Done.')

def set_odrive_gains(odrv):
    """
    Set position control gains for both motors
    """
    print('Setting gains...')
    set_gains(odrv,odrv.axis0)
    set_gains(odrv,odrv.axis1)
    print('Done.')

def test_odrive_motors(odrv):
    """
    Give two position commands to both axis
    """
    print('Testing motors...')
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    time.sleep(0.5)
    odrv.axis0.controller.set_pos_setpoint(1000.0, 0.0, 0.0)
    odrv.axis1.controller.set_pos_setpoint(-1000.0, 0.0, 0.0)
    time.sleep(1)
    odrv.axis0.controller.set_pos_setpoint(0.0, 0.0, 0.0)
    odrv.axis1.controller.set_pos_setpoint(0.0, 0.0, 0.0)
    print('Done.')

def get_odrive(shutdown_token):
    """
    Look for and return an odrive connected via usb
    """

    print('Looking for ODrive...')
    odrv = odrive.find_any(search_cancellation_token=app_shutdown_token, channel_termination_token=app_shutdown_token)
    print('Found.')
    return odrv

def print_configs(odrv0):
    """
    Print configuration info
    """

    print('\n\nMOTOR CONFIG')
    print(odrv0.axis0.motor.config)
    print('\n')
    print(odrv0.axis1.motor.config)
    print('\n\n ENC CONFIG')
    print(odrv0.axis0.encoder.config)
    print('\n')
    print(odrv0.axis1.encoder.config)
    print('\n\n AXIS CONFIG')
    print(odrv0.axis0.config)
    print('\n')
    print(odrv0.axis1.config)

def full_calibration(app_shutdown_token):
    """
    Reset the odrive and calibrate everything:
    1) Gains and limits
    2) Motor
    3) Encoder index
    4) Encoder offset
    5) Set startup
    """

    # Find a connected ODrive (this will block until you connect one)
    odrv0 = get_odrive(app_shutdown_token)
    reset_odrive(odrv0)
    odrv0 = get_odrive(app_shutdown_token)

    init_odrive(odrv0)
    set_odrive_gains(odrv0)
    calibrate_odrive(odrv0)

    odrv0.save_configuration()

    #run_state(axis=odrv0.axis0, requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL, wait=False);
    #run_state(axis=odrv0.axis1, requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL, wait=False);
    #odrv0.axis0.controller.pos_setpoint = 0;
    #odrv0.axis1.controller.pos_setpoint = 0;

    print_configs(odrv0)

def bare_bones_calibration(app_shutdown_token, reset=True):
    """
    Just calibrate motors and basic gains
    """
    print("Run doghome\n ")# odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH   odrv0.axis0.encoder.index_found
    odrv0 = get_odrive(app_shutdown_token)
    if reset:
        reset_odrive(odrv0)
        odrv0 = get_odrive(app_shutdown_token)

    init_odrive(odrv0)
    set_odrive_gains(odrv0)

    calibrate_motor(odrv0, odrv0.axis0)
    calibrate_motor(odrv0, odrv0.axis1)

    odrv0.axis0.config.startup_encoder_offset_calibration = True
    odrv0.axis1.config.startup_encoder_offset_calibration = True
    odrv0.axis0.config.startup_closed_loop_control = True
    odrv0.axis1.config.startup_closed_loop_control = True
    
    odrv0.axis0.encoder.config.use_index = False
    odrv0.axis1.encoder.config.use_index = False
    odrv0.axis0.config.startup_encoder_index_search =False
    odrv0.axis1.config.startup_encoder_index_search =False
    odrv0.save_configuration()

    print_configs(odrv0)

def main(app_shutdown_token):
    """
    !! Main program !!
    Looks for odrive, then calibrates, then sets gains, then tests motors


    WARNING: Saving more than twice per boot will cause a reversion of all changes
    """
    bare_bones_calibration(app_shutdown_token, reset=True)
    #bare_bones_calibration(app_shutdown_token, reset=False)

import odrive
from fibre import Logger, Event
from odrive.utils import OperationAbortedException
from fibre.protocol import ChannelBrokenException

app_shutdown_token = Event()
try:
    main(app_shutdown_token)
    # init_odrive(odrv0)
except OperationAbortedException:
    logger.info("Operation aborted.")
finally:
    app_shutdown_token.set()

# encoder calibration
# set use_index ot true
# request encoder_index_search
# pre_calibrated to True
# then change startup to encoder_index_search true
