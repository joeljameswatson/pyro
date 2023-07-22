# -*- coding: utf-8 -*-
"""
Estimate Relaxation from Band Powers

This example shows how to buffer, epoch, and transform EEG data from a single
electrode into values for each of the classic frequencies (e.g. alpha, beta, theta)
Furthermore, it shows how ratios of the band powers can be used to estimate
mental state for neurofeedback.

The neurofeedback protocols described here are inspired by
*Neurofeedback: A Comprehensive Review on System Design, Methodology and Clinical Applications* by Marzbani et. al

Adapted from https://github.com/NeuroTechX/bci-workshop
"""

import numpy as np  # Module that simplifies computations on matrices
import matplotlib.pyplot as plt  # Module used for plotting
from pylsl import StreamInlet, resolve_byprop  # Module to receive EEG data
import utils  # Our own utility functions

from colorama import init, Fore, Back, Style
import sys

# Initializes Colorama
init(autoreset=True)

import time
# import board
try:
    from adafruit_motor import stepper
    from adafruit_motorkit import MotorKit
    kit = MotorKit()
except ImportError:
    print('Did not load motor kit')
# Below initialises the variable kit to be our I2C Connected Adafruit Motor HAT
# kit = MotorKit(i2c=board.I2C()) # i2c=board.I2C() doesn't appear to be needed


STANDARD_DEVIATION_MAX=250
STANDARD_DEVIATION_MIN=5
MOTOR_STEPS = 130
UPDATE_FIRE_SECONDS = 30
FIRE_BURN_TIME = 20

# bump up val (good for beta when too low)
BETA_BUMP = .5

# fire step levels
FIRE_STEP_LEVEL_0 = 0
FIRE_STEP_LEVEL_1 = 30
FIRE_STEP_LEVEL_2 = 60
FIRE_STEP_LEVEL_3 = 90
FIRE_STEP_LEVEL_4 = 130

activate_fire_threshold = 10
good_samples = [False] * 10
eeg_metric = "alpha"
update_mode = "1"
current_step = 0

brain_connected = False
last_update_time = time.time()

if (len(sys.argv) > 1):
    eeg_metric = sys.argv[1]

if (len(sys.argv) > 2):
    update_mode = sys.argv[2]

metric_dict = {
    "alpha": "RELAXATION",
    "beta": "FOCUS"
}

metric_desc = metric_dict[eeg_metric]

# Handy little enum to make code more readable
class Band:
    Delta = 0
    Theta = 1
    Alpha = 2
    Beta = 3


""" EXPERIMENTAL PARAMETERS """
# Modify these to change aspects of the signal processing

# Length of the EEG data buffer (in seconds)
# This buffer will hold last n seconds of data and be used for calculations
BUFFER_LENGTH = 5

# Length of the epochs used to compute the FFT (in seconds)
EPOCH_LENGTH = 1

# Amount of overlap between two consecutive epochs (in seconds)
OVERLAP_LENGTH = 0.8

# Amount to 'shift' the start of each next consecutive epoch
SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH

# Index of the channel(s) (electrodes) to be used
# 0 = left ear, 1 = left forehead, 2 = right forehead, 3 = right ear
INDEX_CHANNEL = [0]

def print_level(metric):
    multiplier = 60
    max_width = 60
    rounded_metric = int(metric * multiplier)
    if rounded_metric < 1:
        rounded_metric = 1

    rounded_activate_fire_threshold = int(activate_fire_threshold * multiplier)
    if rounded_activate_fire_threshold < 1:
        rounded_activate_fire_threshold = 1
    level = ""
    for i in range(max_width):
        if i == rounded_activate_fire_threshold:
            level = f"{level + Style.BRIGHT + Fore.WHITE}|{Style.BRIGHT + Fore.GREEN}"
        elif i - 1 < rounded_metric:
            level = f"{Style.BRIGHT + Fore.GREEN + level}|"
        elif i > rounded_metric:
            level = f"{level} "
    print(level)

def fire_control(metric):
    global last_update_time
    global activate_fire_threshold
    global brain_connected
    
    if brain_connected == False:
        return
        
    print_level(metric)

    current_time = time.time()
    if (current_time - last_update_time) > UPDATE_FIRE_SECONDS:
        last_update_time = current_time
        
        print(' ')
        print(f"COMPARING {metric_desc} LEVELS")
        time.sleep(2)
        print(' ')

        if metric > activate_fire_threshold:
            print(Style.BRIGHT + Fore.WHITE + 'ACTIVATING FIRE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            print(' ')
            time.sleep(1)
            
            for i in range(MOTOR_STEPS):
                # print('step anticlockwise', i)
                if 'kit' in globals():
                    kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.SINGLE)
                time.sleep(.01)
                if 'kit' in globals():
                    kit.stepper2.release()

            time.sleep(FIRE_BURN_TIME)

            for i in range(MOTOR_STEPS):
                # print('step clockwise', i)
                if 'kit' in globals():
                    kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.SINGLE)
                time.sleep(.01)
                if 'kit' in globals():
                    kit.stepper2.release()

        print('SETTING ACTIVATION LEVEL')
        activate_fire_threshold  = metric
        time.sleep(1)

def fire_control2(metric):
    global last_update_time
    global activate_fire_threshold
    global brain_connected
    global current_step
    
    if(brain_connected == True):
        print_level(metric)
        

    current_time = time.time()
    if (current_time - last_update_time) > UPDATE_FIRE_SECONDS:
        last_update_time = current_time


        def move_to_step(target_step):
            global current_step

            if (current_step > target_step):
                print('turning down')
                time.sleep(2)
                step_down(current_step - target_step)
            elif (current_step < target_step):
                print('turning up')
                time.sleep(2)
                step_up(target_step - current_step)
            else:
                # print('no change')
                time.sleep(2)
            
            current_step = target_step

            #update level

        def step_up(up_steps):    
            for i in range(up_steps):
                # print('step anticlockwise', i)
                if 'kit' in globals():
                    kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.SINGLE)
                time.sleep(.01)
                if 'kit' in globals():
                    kit.stepper2.release()

        def step_down(down_steps): 
            for i in range(down_steps):
                # print('step clockwise', i)
                if 'kit' in globals():
                    kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.SINGLE)
                time.sleep(.01)
                if 'kit' in globals():
                    kit.stepper2.release()

                print(' ')

        if (brain_connected == True):
            print(f"COMPARING {metric_desc} LEVELS")
            time.sleep(2)
            print(' ')
            
            print('UPDATING LEVEL...')
            activate_fire_threshold  = metric

            # print_level(metric)

        if (brain_connected == False):
            print('brain disconnect stop fire')
            move_to_step(FIRE_STEP_LEVEL_0)
        elif (activate_fire_threshold < .25):
            #1/4
            # step 30
            move_to_step(FIRE_STEP_LEVEL_1)
        elif (activate_fire_threshold >= .25 and activate_fire_threshold < .5):
            # 1/2
            # step 60
            move_to_step(FIRE_STEP_LEVEL_2)
        elif (activate_fire_threshold >= .5 and activate_fire_threshold < .75):
            # step 90
            move_to_step(FIRE_STEP_LEVEL_3)
        elif (activate_fire_threshold >= .75):
            # step 130
            move_to_step(FIRE_STEP_LEVEL_3)
        else:
            move_to_step(FIRE_STEP_LEVEL_0)

# set to baseline when connected  
# move activate_fire_threshold to predefined level
# print metric to show new level moving to
# 4 levels
# 0 no brain
# 1 < 25%
# 2 < 50%
# 3 <  




if __name__ == "__main__":

    """ 1. CONNECT TO EEG STREAM """

    # Search for active LSL streams
    print('Looking for an EEG stream...')
    streams = resolve_byprop('type', 'EEG', timeout=2)
    if len(streams) == 0:
        raise RuntimeError('Can\'t find EEG stream.')

    # Set active EEG stream to inlet and apply time correction
    print("Start acquiring data")
    inlet = StreamInlet(streams[0], max_chunklen=12)
    eeg_time_correction = inlet.time_correction()

    # Get the stream info and description
    info = inlet.info()
    description = info.desc()

    # Get the sampling frequency
    # This is an important value that represents how many EEG data points are
    # collected in a second. This influences our frequency band calculation.
    # for the Muse 2016, this should always be 256
    fs = int(info.nominal_srate())

    """ 2. INITIALIZE BUFFERS """

    # Initialize raw EEG data buffer
    eeg_buffer = np.zeros((int(fs * BUFFER_LENGTH), 1))
    filter_state = None  # for use with the notch filter

    # Compute the number of epochs in "buffer_length"
    n_win_test = int(np.floor((BUFFER_LENGTH - EPOCH_LENGTH) /
                              SHIFT_LENGTH + 1))

    # Initialize the band power buffer (for plotting)
    # bands will be ordered: [delta, theta, alpha, beta]
    band_buffer = np.zeros((n_win_test, 4))

    """ 3. GET DATA """

    # The try/except structure allows to quit the while loop by aborting the
    # script with <Ctrl-C>
    print('Press Ctrl-C in the console to break the while loop.')

    try:
        # The following loop acquires data, computes band powers, and calculates neurofeedback metrics based on those band powers
        while True:
            try:
                """ 3.1 ACQUIRE DATA """
                # Obtain EEG data from the LSL stream
                eeg_data, timestamp = inlet.pull_chunk(
                    timeout=1, max_samples=int(SHIFT_LENGTH * fs))
                
                # print(eeg_data)

                standard_deviations = np.std(eeg_data, axis=0)
                st_float = standard_deviations[0]
                st = int(st_float)

                if (st < STANDARD_DEVIATION_MAX and st > STANDARD_DEVIATION_MIN):
                    good_samples.append(True)
                    # print('brain attached')
                else:
                    good_samples.append(False)

                good_samples.pop(0)

                if (all(good_samples)):
                    brain_connected = True
                else:
                    brain_connected = False
                    print(f"SEARCHING FOR BRAIN")
                    print(f"PLEASE KEEP STILL..  {STANDARD_DEVIATION_MAX} | {st}")
                    print("")
                    

                # Only keep the channel we're interested in
                ch_data = np.array(eeg_data)[:, INDEX_CHANNEL]

                # Update EEG buffer with the new data
                eeg_buffer, filter_state = utils.update_buffer(
                    eeg_buffer, ch_data, notch=True,
                    filter_state=filter_state)

                """ 3.2 COMPUTE BAND POWERS """
                # Get newest samples from the buffer
                data_epoch = utils.get_last_data(eeg_buffer,
                                                EPOCH_LENGTH * fs)
                # print(data_epoch)

                # Compute band powers
                band_powers = utils.compute_band_powers(data_epoch, fs)
                band_buffer, _ = utils.update_buffer(band_buffer,
                                                    np.asarray([band_powers]))
                # Compute the average band powers for all epochs in buffer
                # This helps to smooth out noise
                smooth_band_powers = np.mean(band_buffer, axis=0)

                # print('Delta: ', band_powers[Band.Delta], ' Theta: ', band_powers[Band.Theta],
                #       ' Alpha: ', band_powers[Band.Alpha], ' Beta: ', band_powers[Band.Beta])


                """ 3.3 COMPUTE NEUROFEEDBACK METRICS """
                # These metrics could also be used to drive brain-computer interfaces

                # Alpha Protocol:
                # Simple redout of alpha power, divided by delta waves in order to rule out noise
                # alpha_metric = smooth_band_powers[Band.Alpha] / \
                #     smooth_band_powers[Band.Delta]
                # metric = alpha_metric
                # print('Alpha Relaxation: ', alpha_metric)

                # Beta Protocol:
                # Beta waves have been used as a measure of mental activity and concentration
                # This beta over theta ratio is commonly used as neurofeedback for ADHD
                # beta_metric = smooth_band_powers[Band.Beta] / \
                #     smooth_band_powers[Band.Theta]
                # # print('Beta Concentration: ', beta_metric)
                # metric = beta_metric

                # Alpha/Theta Protocol:
                # This is another popular neurofeedback metric for stress reduction
                # Higher theta over alpha is supposedly associated with reduced anxiety
                # theta_metric = smooth_band_powers[Band.Theta] / \
                #     smooth_band_powers[Band.Alpha]
                # print('Theta Relaxation: ', theta_metric)
                # metric = theta_metric

                metric_val = 0
   
                if (eeg_metric == "alpha"):
                    metric_val = smooth_band_powers[Band.Alpha] / \
                        smooth_band_powers[Band.Delta]
                    # print('alpha')
                elif (eeg_metric == "beta"):
                    metric_val = smooth_band_powers[Band.Beta] / \
                        smooth_band_powers[Band.Theta]
                    metric_val = metric_val + BETA_BUMP

                # print(f"{eeg_metric} {metric_val}")
                
                if (update_mode == "1"):
                   fire_control(metric_val)
                elif (update_mode == "2"):
                    fire_control2(metric_val) 

                
            except Exception as e: print(e)

    except KeyboardInterrupt:
        print('Closing!')
