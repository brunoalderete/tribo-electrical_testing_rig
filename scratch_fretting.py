"""
Date 01.2021
@author: Chair of Functional Materials - Saarland University - Saarbrücken, Bruno Alderete
@version 1.0
"""
__author__ = 'Bruno Alderete'

#######################################################################################################################
#
# The MIT License (MIT)
#
# Copyright (c) 2021 Bruno Alderete
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#######################################################################################################################

#########################################################################

########################################################
########################################################
############# SCRATCH AND FRETTING TEST  ###############
########################################################
########################################################

#########################################################################

from gsv8 import gsv8                               # import for force sensor
from GSV_BasicMeasurement import BasicMeasurement   # import for force Sensor
from pipython import GCSDevice, pitools             # import for stages
import time                                         # import for time
from datetime import datetime, timedelta
import threading
import pandas as pd
import pyvisa


########################
#####  INSTRUMENTS #####
########################

### STAGES ###
STAGES = ('L-511.20SD00', 'L-511.20SD00')
REFMODE = ('FNL')                                                               # Fast reference move to negative limit
motorZ = GCSDevice('C-663.12')
motorX = GCSDevice('C-663.12')

### FORCE SENSOR ###
force_sensor = gsv8("COM16", 115200)

### PIEZO ###
piezo = GCSDevice(devname='E-709.CHG')

### ELECTRICAL EQUIPMENT ###
rm = pyvisa.ResourceManager('@py')

k2400 = rm.open_resource('GPIB::24::INSTR')
k2182 = rm.open_resource('GPIB::7::INSTR')


###############################
##### VARIABLES AND LISTS #####
###############################
is_done = False


### ELECTRIC PARAMETERS ###
i_meas_list = []
v_meas_list = []
resistance = []

current = 100E-3            # 100 mA
sleep_time = 1

### FORCE ACQUISITION ###
force_x = []
force_z = []
cof = []

### FRETTING ###
amp = []
freq = []
dur = []
dur_time = []

### RESULTS ###
df_results = pd.DataFrame(columns=['Force X-Axis', 'Force Z-Axis', 'CoF'])
df_fretting = pd.DataFrame(columns=['Amplitude', 'Frequency', 'Cycles', 'Fretting duration'])
df_electric = pd.DataFrame(columns=['Current', 'Voltage', 'Resistance'])
df_all_values = pd.DataFrame(columns=['Force X-Axis', 'Force Z-Axis', 'CoF', 'Amplitude', 'Frequency', 'Cycles', 'Fretting duration', 'Current', 'Voltage', 'Resistance'])


###############################
##### FUNCTION DEFINITION #####
###############################

### TAKE FORCE ALL AXIS ###
def take_force_all():
    measurement0 = force_sensor.ReadValue()

    x_load0 = "%.4f" % float('{}'.format(measurement0.getChannel1()))
    y_load0 = "%.4f" % float('{}'.format(measurement0.getChannel2()))
    z_load0 = "%.4f" % float('{}'.format(measurement0.getChannel3()))

    print('X load is: ', x_load0, ' N')
    print('Y load is: ', y_load0, ' N')
    print('Z load is: ', z_load0, ' N')


### TAKE NORMAL LOAD ONLY ###
def take_force_normal():
    measurementZ = force_sensor.ReadValue()

    z_load = "%.4f" % float('{}'.format(measurementZ.getChannel3()))

    print('Z load is ', z_load, ' N')
    return z_load


### START, REFERENCE AND MOVE STAGES TO CENTER POSITION ###
def start_stages(targetZ=10,targetX=26,vel_Z=15,vel_X=15,wait=5):
    motorZ.OpenUSBDaisyChain(description='019550102')
    daisychainid = motorZ.dcid
    motorZ.ConnectDaisyChainDevice(1, daisychainid)
    motorX.ConnectDaisyChainDevice(2, daisychainid)


    pitools.startup(motorZ, stages=STAGES, refmode=REFMODE)
    print('Z-axis current position: ', float(str(motorZ.qPOS(motorZ.axes))[18:-3]))

    pitools.startup(motorX, stages=STAGES, refmode=REFMODE)
    print('X-axis current position: ', float(str(motorX.qPOS(motorX.axes))[18:-3]))

    time.sleep(wait)

    motorX.VEL(motorX.axes, vel_X)
    motorX.MOV(motorX.axes, targetX)
    pitools.waitontarget(motorX)
    print('Position X: ',float(str(motorX.qPOS(motorX.axes))[18:-3]))

    motorZ.VEL(motorZ.axes, vel_Z)
    motorZ.MOV(motorZ.axes, targetZ)
    pitools.waitontarget(motorZ)
    print('Position Z: ',float(str(motorZ.qPOS(motorZ.axes))[18:-3]))

    time.sleep(wait)

### START ELECTRIC INSTRUMENTS ###
def start_instr(curr_app='100E-3',curr_prot='150E-3'):
    # Start and reset Keithley 2400
    k2400.write('*RST')
    k2400.timeout = 60000                                                       # 60 seconds timeout
    k2400.write(':ROUT:TERM REAR')
    
    k2400.write(':SENS:FUNC:CONC OFF')
    k2400.write(':SOUR:FUNC CURR')
    k2400.write(f':SOUR:CURR {curr_app}')                                       # Applied current in A
    k2400.write(":SENS:FUNC 'CURR:DC'")
    k2400.write(f':SENS:CURR:PROT {curr_prot}')

    # Start and resete Keithley 2182
    k2182.write('*RST')


### TAKE i MEASUREMENT ###
def take_i_meas(sour_del=1):                                             
    k2400.write('TRIG:COUN 1')                                                  # Amount of measurements
    k2400.write(f'SOUR:DEL {sour_del}')                                         # Delay in seconds (between 0, 60). Time i is applied
    k2400.write(':FORM:ELEM CURR')
    k2400.write(':OUTP ON')
    i_meas_list.append(k2400.query_ascii_values(':READ?'))


### TAKE V MEASUREMENT ###
def take_v_meas():
    k2182.write('*RST')
    k2182.write(":SENS:FUNC 'VOLT'")
    k2182.write(':SENS:CHAN 1')
    k2182.write(':SENS:VOLT:CHAN1:RANG 1')                                # k2182.write(':SENS:VOLT:CHAN1:RANG:AUTO ON')
    v_meas_list.append(k2182.query_ascii_values(':READ?'))



### TAKE FULL MEASUREMENT ###
def take_measurement(meas=5, trigs=0, curr_app='100E-3', curr_prot='150E-3', sour_del=1, sleep_time=1):


    start_instr(curr_app, curr_prot)

    while trigs < meas:

        take_i_meas(sour_del)                                   
        take_v_meas()
        trigs += 1
        k2400.write(':OUTP OFF')        # turns off smu between measurements
        

### ECR ###
def ecr(meas=5):
    trigs = 0               # set counter to zero
    while trigs < meas:

        if trigs % 2 == 0:
            take_measurement(meas=1, trigs=0, curr_app=str(current), curr_prot='150E-3', sour_del=1)                  
            time.sleep(sleep_time)
        else:
            take_measurement(meas=1, trigs=0, curr_app='-' + str(current), curr_prot='150E-3', sour_del=1)     
            time.sleep(sleep_time)
        trigs += 1


### COARSE APPROACH X-AXIS ###
def approach_X_stage(approach_X=1):
    print('Position X: ',float(str(motorX.qPOS(motorX.axes))[18:-3]))
    while approach_X == 1:
        print('\n##############################################################')
        moveX = float(input('Enter how many mm the X-axis should move: '))
        print('You chose to move: ', moveX, ' mm\n')

        motorX.MVR(motorX.axes, moveX)                                                          # MVR moves relative, MOV moves absolute
        pitools.waitontarget(motorX)
        print('Position X: ',float(str(motorX.qPOS(motorX.axes))[18:-3]))
        approach_X = int(input('Do you want to keep moving the X stage? (enter 1 to continue approaching, enter 0 if you DO NOT want to continue approaching: '))
        if approach_X == 1:
            print('You chose to continue approaching the X stage.\n')                 # MAKE SURE YOU INPUT CORRECTLY
        else:
            print('You are done approaching the X stage.\n')


### COARSE APPROACH Z-AXIS ###
def approach_Z_stage(approach_Z=1):
    print('Position Z: ',float(str(motorZ.qPOS(motorZ.axes))[18:-3])) 
    while approach_Z == 1:
        print('\n##############################################################')
        moveZ = float(input('Enter how many mm the Z-axis should move: '))
        print('You chose to move: ', moveZ, ' mm\n')

        motorZ.MVR(motorZ.axes, moveZ)
        pitools.waitontarget(motorZ)
        print('Position Z: ',float(str(motorZ.qPOS(motorZ.axes))[18:-3]))
        approach_Z = int(input('Do you want to keep moving the Z stage? (enter 1 to continue approaching, enter 0 if you DO NOT want to continue approaching: '))
        if approach_Z == 1 :
            print('You chose to continue approaching the Z stage.\n')               # MAKE SURE YOU INPUT CORRECTLY
        else:
            print('You are done approaching the Z stage.\n')


start_stages()
approach_X_stage()
approach_Z_stage()
approach_X_stage()


#############################
###### PARAMETER INPUT ######
#############################

### SCRATCH ###
print('\n##############################################################')
print('In the following commands please input the parameters for the scratch test.')
track_length = float(input('Enter the length of the desired scratch (in mm): '))
print('You chose {} mm track length\n'.format(track_length))

print('Current X stage velocity: {} mm/s'.format(float(str(motorX.qVEL(motorX.axes))[18:-3])))
scratch_vel = float(input('Enter the motion velocity: '))
motorX.VEL(motorX.axes, scratch_vel)
print('You chose the {} mm/s.\n'.format(float(str(motorX.qVEL(motorX.axes))[18:-3])))


### FRETTING ###
print('\n##############################################################')
print('Now enter the parameters for the fretting test.')
amplitude = float(input('Enter the amplitude (max amplitude is 75 µm): '))
print('You chose {} µm amplitude.\n'.format(amplitude))

frequency_input = float(input('Enter the frequency (max frequency is 100 Hz): '))
fretting_vel = (frequency_input * amplitude)
print('You chose {} Hz frequency.\n'.format(frequency_input))

cycles = float(input('Enter the desired duration of the fretting test (in cycles): '))
print('{} cycles will be done.'.format(cycles))
total_time = (cycles / frequency_input)                                                     # approximate time it takes for the fretting test

### START ###
start = int(input('Do you want to start? Enter 1 to start, enter 0 to exit. '))
time.sleep(1)


#########################################
##### FINE APPROACH TO FORCE SENSOR #####
#########################################

def fine_approach(target_load=5):
    z_load_i = take_force_normal()
    print('Initial normal load measurement: ', z_load_i, ' N')
    
    if start == 1:
        print ('\nStarting fine approach')

        print('Current force: ', float(z_load_i), ' N')                             # first measurement from force sensor (NULLER)

        current_z_load = take_force_normal()                                        # current sensor value
        print('Current load: ',current_z_load, ' N')

        while (float(current_z_load) < float(target_load)):
            position = float(str(motorZ.qPOS(motorZ.axes))[18:-3])                  # get position
            print('Current motor position: ', position)

            
            print('\nApproaching...')                             # starts moving
            if float(current_z_load) < (target_load * 0.4):
                ztarget = position + 0.01
            elif float(current_z_load) >= (target_load * 0.4) and float(current_z_load) <= (target_load * 0.90):
                ztarget = position + 0.001
            else:
                ztarget = position + 0.0001
        
            print('Target: ', ztarget)
            motorZ.MOV(motorZ.axes, ztarget)
            pitools.waitontarget(motorZ)
            position = float(str(motorZ.qPOS(motorZ.axes))[18:-3]) 
            print('New motor position: ', position)

            time.sleep(0.5)                                                         # wait for 0.5 s before taking new force measurement
            current_z_load = take_force_normal()                   
            print('current load: ',current_z_load, ' N')

    
        print('Target force reached: {}'.format(current_z_load))
        time.sleep(1)


### SCRATCH TEST ###
def scratch_test(track_length, scratch_vel):

    print('\nStarting scratch test...')
    time.sleep(0.5)
    motorX.VEL(motorX.axes, scratch_vel)
    motorX.MVR(motorX.axes, track_length)
    time.sleep(2)


### RETURN AFTER SCRATCH ###
def scratch_return(track_length, scratch_vel):

    motorX.VEL(motorX.axes, scratch_vel)
    motorX.MVR(motorX.axes, -track_length)


### FRETTING TEST ###
def fretting_test(amplitude, fretting_vel, cycles):
    print('\nStarting fretting test...')
    global cycle_counter
    cycle_counter = 0

    #piezo.InterfaceSetupDlg()
    piezo.ConnectUSB('0119028920')

    start_fretting = datetime.now()
    
    piezo.SVO(1, 1)                     # Set servo mode
    piezo.MOV(1, 0)                     # Move to absolute position 0
    print(piezo.qPOS(1))                # Query starting position
    piezo.VEL(1, fretting_vel)          # Set velocity to the value
    print(piezo.qVEL(1))                # Query velocity

    while cycle_counter < cycles:
        piezo.MVR(1, amplitude)
        position1 = piezo.qPOS(1)
        print(position1)

        piezo.MOV(1, 0)
        position2 = piezo.qPOS(1)
        print(position2)
        cycle_counter += 1

    end_fretting = datetime.now()
    global fretting_runtime
    fretting_runtime = (end_fretting - start_fretting).total_seconds()
    print('Fretting test ran for {} seconds.'.format(fretting_runtime))

    time.sleep(1)

    piezo.CloseConnection()

    return fretting_runtime


### ACQUIRE FORCE DATA ###
def acquire_cof():
    timestamp = time.time()
    start_time = timestamp
    
    Mesfrq = 0.01
    next_measurement = timestamp
    duration1 = (track_length / scratch_vel) + 4

    while (start_time + duration1) > time.time():
        
        if (time.time() >= next_measurement):
                    
            next_measurement += Mesfrq
            force_meas = force_sensor.ReadValue()
            value_z = float('{}'.format(force_meas.getChannel3()))              # normal load
            value_x = float('{}'.format(force_meas.getChannel1()))              # friction

            force_x.append(value_x)
            force_z.append(value_z)

            print('\nZ value: ',value_z)
            print('X value: ',value_x)

            timestamp = time.time()

### CONTROL NORMAL LOAD ###
def control_normal_load(target_load):
    
    while is_done == False:
        #current_time =time.time()
        normal_load = take_force_normal()

        if float(normal_load) < target_load:                                                        
               
            position = float(str(motorZ.qPOS(motorZ.axes))[18:-3])
            ztarget = position + 0.0001
            motorZ.MOV(motorZ.axes, ztarget)
            normal_load = take_force_normal()

        elif float(normal_load) > target_load:

            position = float(str(motorZ.qPOS(motorZ.axes))[18:-3])
            ztarget = position - 0.0001)
            motorZ.MOV(motorZ.axes, ztarget)
            normal_load = take_force_normal()
    else:
        return
        

def start_threading(target_load):
    global is_done
    is_done = False
    
    ### Start force control and first ECR ###
    t9 = threading.Thread(target=control_normal_load, args=((5),))
    t9.start()
    t5 = threading.Thread(target=ecr, args=((5),))
    t5.start()
    t5.join()
    time.sleep(0.2)
    if t5.is_alive() == False:
        is_done = True

    ### Scratch test with CoF ### No compensation
    t2 = threading.Thread(target=acquire_cof)
    t3 = threading.Thread(target=scratch_test, args=((track_length), (scratch_vel), ))
    t2.start()
    t3.start()
    t3.join()
    t2.join()

    ### Second ECR with compensation pause ###
    is_done = False
    t10 = threading.Thread(target=control_normal_load, args=((5),))
    t10.start()
    time.sleep(5)
    normal_load = float(take_force_normal())
    while normal_load < float((target_load * 0.99)) or normal_load > float((target_load *1.01)):
        time.sleep(0.5)
        normal_load = float(take_force_normal())
    t6 = threading.Thread(target=ecr, args=((5),))
    t6.start()
    t6.join()
    time.sleep(0.2)

    ### Fretting test (compensation cont.) ###
    t1 = threading.Thread(target=control_normal_load, args=((5),)) 
    t1.start()
    time.sleep(1)
    t4 = threading.Thread(target=fretting_test, args=((amplitude), (fretting_vel), (cycles)))
    t4.start()
    t4.join()
    time.sleep(0.2)

    ### Third ECR (compensation cont.) ###
    t11 = threading.Thread(target=control_normal_load, args=((5),))
    t11.start()
    time.sleep(5)
    t7 = threading.Thread(target=ecr, args=((5),))
    t7.start()
    t7.join()
    time.sleep(0.2)
    if t7.is_alive() == False:
        is_done = True
        
    ### Scratch return to starting position with CoF ### No compensation
    t8 = threading.Thread(target=acquire_cof)
    t12 = threading.Thread(target=scratch_return, args=((track_length), (scratch_vel)))
    t8.start()
    t12.start()
    t12.join()
    t8.join()

    print('Tests finished...')



### DATA TO CSV ###
def save_data():
    
    # Electric parameters #
    for x in range(len(i_meas_list[:])):
        df_electric.loc[x, 'Current'] = (i_meas_list[x-1][0])
        df_electric.loc[x, 'Voltage'] = (v_meas_list[x-1][0])
        resist = v_meas_list[x-1][0] / i_meas_list[x-1][0]
        resistance.append(resist)
    
    df_electric['Resistance'] = resistance
        

    # Force parameters #
    for x in range(len(force_x[:])):
        df_results.loc[x, 'Force X-Axis'] = (force_x[x-1])
        df_results.loc[x, 'Force Z-Axis'] = (force_z[x-1])
        coef_of_f = force_x[x-1] / force_z[x-1]
        cof.append(coef_of_f)
    
    df_results['CoF'] = cof
    amp.append(amplitude)
    df_fretting['Amplitude'] = amp
    freq.append(frequency_input)
    df_fretting['Frequency'] = freq
    dur.append(cycle_counter)
    df_fretting['Cycles'] = dur
    dur_time.append(fretting_runtime)
    df_fretting['Fretting duration'] = fretting_runtime
    print(df_results)
    print(df_fretting)
    print(df_electric)

    df_all_values = pd.concat([df_results,df_fretting,df_electric], ignore_index=True, axis=1)
    print(df_all_values)

    df_results.to_csv(r'C:\Users\Labor\Desktop\fretting_scratch_erc24.02_2.1.csv', index=False)
    df_fretting.to_csv(r'C:\Users\Labor\Desktop\fretting_scratch_erc24.02_2.2.csv', index=False)
    df_all_values.to_csv(r'C:\Users\Labor\Desktop\fretting_scratch_erc24.02_2.3.csv', index=False)


### Z Stage retreats ###
def retreat_stages(targetZ=10,targetX=26,vel_Z=15,vel_X=15,wait=2):
    time.sleep(2)
    retreat = int(input('Return stages to starting position? Enter 1 for yes, enter 0 for no: '))
    
    if retreat == 1:
        targetZ0 = -5
        motorZ.MVR(motorZ.axes, targetZ0)
        pitools.waitontarget(motorZ)
        time.sleep(1)
    
        motorX.VEL(motorX.axes, vel_X)
        motorX.MOV(motorX.axes, targetX)
        pitools.waitontarget(motorX)
        print('Position X: ',float(str(motorX.qPOS(motorX.axes))[18:-3]))

        motorZ.VEL(motorZ.axes, vel_Z)
        motorZ.MOV(motorZ.axes, targetZ)
        pitools.waitontarget(motorZ)
        print('Position Z: ',float(str(motorZ.qPOS(motorZ.axes))[18:-3]))

        time.sleep(wait)

fine_approach(target_load=5)
start_threading(target_load=5)
save_data()

### END PROGRAM ###
retreat_stages(targetZ=10,targetX=26,vel_Z=15,vel_X=15,wait=5)
print('Closing program...')

elapse_time = (time.perf_counter() / 60)
print('Elapse time: {} min'.format(elapse_time))

#########################################
##### CLOSES CONNECTION WITH MOTORS #####
#########################################

motorZ.CloseDaisyChain()

