"""
Date 04.2021
@author: Chair of Functional Materials - Saarland University - Saarbrücken, Bruno Alderete
@version 1.2
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

#########################################################################
#########################################################################
######### THIS PROGRAM INPUTS A LIST OF APPLIED NORMAL LAODS  ###########
##### CARRIES OUT THE ECR MEASUREMENTS AND MOVES TO THE NEXT LOAD #######
############ THE CYCLE CONCLUDES AFTER MEASURING ECR UNDER ##############
############################# ALL LOADS #################################
#########################################################################

#########################################################################



from gsv8 import gsv8                               # import for force sensor
from GSV_BasicMeasurement import BasicMeasurement   # import for force Sensor
from pipython import GCSDevice, pitools             # import for stages
import time                                         # import for time
import datetime                                     # import for time
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

current = 100E-3                        # 100 mA (user input)
sleep_time = 2                          # delay between measurements (fixed value)
meas = 5                                # amount of measurements per force (user input)
delay = 1                               # fixed value

j = 0

f_list = []

### RESULTS ###
df_results = pd.DataFrame(columns=['Current', 'Voltage', 'Resistance', 'Force'])


###############################
##### FUNCTION DEFINITION #####
###############################

### TAKE FORCE ALL AXIS ###
def take_force_all():
    #force_sensor = gsv8("COM16", 115200)
    measurement0 = force_sensor.ReadValue()

    x_load0 = "%.4f" % float('{}'.format(measurement0.getChannel1()))
    y_load0 = "%.4f" % float('{}'.format(measurement0.getChannel2()))
    z_load0 = "%.4f" % float('{}'.format(measurement0.getChannel3()))


    print('X load is: ', x_load0, ' N')
    print('Y load is: ', y_load0, ' N')
    print('Z load is: ', z_load0, ' N')

take_force_all()                                                                                                # TAKES INITIAL FORCE

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

start_stages(targetZ=10,targetX=26,vel_Z=15,vel_X=15,wait=5)                                                    # STARTS STAGES


### START ELECTRIC INSTRUMENTS ###
def start_instr(curr_app='100E-3',curr_prot='150E-3'):
    # Start and reset Keithley 2400
    k2400.write('*RST')
    k2400.timeout = 60000                                                       # 60 seconds timeout
    k2400.write(':ROUT:TERM REAR')                                              # rear output             
    
    k2400.write(':SENS:FUNC:CONC OFF')
    k2400.write(':SOUR:FUNC CURR')                                              # source current
    k2400.write(f':SOUR:CURR {curr_app}')                                       # Applied current in A
    k2400.write(":SENS:FUNC 'CURR:DC'")
    k2400.write(f':SENS:CURR:PROT {curr_prot}')                                 # set protection current

    # Start and resete Keithley 2182
    k2182.write('*RST')


### TAKE i MEASUREMENT ###
def take_i_meas(sour_del=delay):
    k2400.write('TRIG:COUN 1')                                                  # Amount of measurements (set to one, will loop)
    k2400.write(f'SOUR:DEL {sour_del}')                                         # Delay in seconds (between 0, 60).
    k2400.write(':FORM:ELEM CURR')
    k2400.write(':OUTP ON')                                                     # turns off source
    i_meas_list.append(k2400.query_ascii_values(':READ?'))                      # saves value to list


### TAKE V MEASUREMENT ###
def take_v_meas(v_range='0.100'):
    k2182.write('*RST')                                                         # resets all stored values
    k2182.write(":SENS:FUNC 'VOLT'")                                            # measuremes voltage
    k2182.write(':SENS:CHAN 1')                                                 # channel out measurement  
    k2182.write(f':SENS:VOLT:CHAN1:RANG {v_range}')                             # sets voltmeter range # k2182.write(':SENS:VOLT:CHAN1:RANG:AUTO ON')
    v_meas_list.append(k2182.query_ascii_values(':READ?'))                      # saves value to list


### TAKE FULL MEASUREMENT ###
def take_measurement(meas=5, trigs=0, curr_app='100E-3', curr_prot='150E-3', sour_del=delay, v_range='0.100', sleep_time=1):

    start_instr(curr_app, curr_prot)                                    # initiates instruments

    # loops the measurements
    while trigs < meas:

        take_i_meas(sour_del)                                           # calls smu function
        take_v_meas(v_range)                                            # calls nanovoltmeter function
        trigs += 1                                                      # counter
        k2400.write(':OUTP OFF')                                        # turns off smu between measurements
    

### ECR ###
def ecr(meas=meas):
    trigs = 0                                                           # set counter to zero
    
    while trigs < meas:

        if trigs % 2 == 0:
            take_measurement(meas=1, trigs=0, curr_app=str(current), curr_prot='150E-3', sour_del=delay, v_range=str(v_range))
            time.sleep(sleep_time)
        else:
            take_measurement(meas=1, trigs=0, curr_app='-' + str(current), curr_prot='150E-3', sour_del=delay, v_range=str(v_range))
            time.sleep(sleep_time)
        trigs += 1
    
    trigs = 0


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
            print('You chose to continue approaching the X stage.\n')                 # MAKE SURE YOU INPUT CORRECTLY (agregar error handling)
        else:
            print('You are done approaching the X stage.\n')

approach_X_stage(approach_X=1)                                                                                                          # STARTS X APPORACH

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
            print('You chose to continue approaching the Z stage.\n')               # MAKE SURE YOU INPUT CORRECTLY   (agregar error handling)
        else:
            print('You are done approaching the Z stage.\n')

approach_Z_stage(approach_Z=1)                                                                                      # STARTS Z APPORACH
approach_X_stage(approach_X=1)                                                                                      # REPEATS X APPROACH (in case you want to modifiy X position after Z appraoch)


#############################
###### PARAMETER INPUT ######
#############################

forces = input('Enter the force sequence seperated by a space (for decimals use "."):' )                # force input
forces_list1 = list(forces.split(' '))                                                                   # string to list
forces_list = [float(i) for i in forces_list1]


meas = int(input('Input measurements per force: '))
current = str(input('Enter current (in A): '))
v_range = str(input('Enter nanovoltmeter range (0.010, 0.100, 1, 10, 100 V): '))

start = int(input('Do you want to start the measurements? Enter 1 to start, enter 0 to exit. '))
time.sleep(1)


#################################################
##### INITIAL FINE APPROACH TO FORCE SENSOR #####
#################################################

def fine_approach(target_load=forces_list[j]):
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

            # float(str(positionZ)[18:-3])      converts orderdict to float

            
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

            time.sleep(0.5)                                                         # wait for 0.2 s before taking new force measurement
            current_z_load = take_force_normal()                                    # tomo otra medida del sensor de fuerza
            print('current load: ',current_z_load, ' N')

    
        print('Target force reached: {}'.format(current_z_load))
        time.sleep(1)

fine_approach(target_load=forces_list[0])                                            # start fine appraoch to first load


### NEXT LOAD ###
def next_load(next_l=forces_list[j]): #j+1
    z_load_i = take_force_normal()
    print('Current load is: {} N'.format(z_load_i))
    print('Moving to next load {} N'.format(forces_list[j])) #j+1

    current_z_load = float(take_force_normal())

    while (float(current_z_load)) < float(next_l):
        position = float(str(motorZ.qPOS(motorZ.axes))[18:-3])                  # get position
        print('Current motor position: ', position)

        if float(current_z_load) < (next_l * 0.40):
            ztarget = position + 0.01
        elif float(current_z_load) >= (next_l * 0.40) and float(current_z_load) <= (next_l * 0.90):
            ztarget = position + 0.001
        else:
            ztarget = position + 0.0001

        print('Target: ', ztarget)
        motorZ.MOV(motorZ.axes, ztarget)
        pitools.waitontarget(motorZ)
        position = float(str(motorZ.qPOS(motorZ.axes))[18:-3]) 
        print('New motor position: ', position)

        time.sleep(0.5)                                                         # wait for 0.2 s before taking new force measurement
        current_z_load = float(take_force_normal())                                    # tomo otra medida del sensor de fuerza
        print('current load: ',current_z_load, ' N')

    while (float(current_z_load)) > float(next_l):
        position = float(str(motorZ.qPOS(motorZ.axes))[18:-3])                  # get position
        print('Current motor position: ', position)

        if float(current_z_load) > float((next_l * 0.90) + next_l):
            ztarget = position - 0.01
        elif float(current_z_load) <= float((next_l * 0.90) + next_l) and float(current_z_load) >= float((next_l * 0.4) + next_l):
            ztarget = position -0.001
        else:
            ztarget = position - 0.0001

        print('Target: ', ztarget)
        motorZ.MOV(motorZ.axes, ztarget)
        pitools.waitontarget(motorZ)
        position = float(str(motorZ.qPOS(motorZ.axes))[18:-3]) 
        print('New motor position: ', position)

        time.sleep(0.5)                                                         # wait for 0.2 s before taking new force measurement
        current_z_load = float(take_force_normal())                                    # tomo otra medida del sensor de fuerza
        print('current load: ',current_z_load, ' N')


    print('\n Target force reached: {}'.format(current_z_load))
    


### CONTROL NORMAL LOAD ###
def control_normal_load(target_load=forces_list[j]):

    while is_done == False:

        normal_load = take_force_normal()

        if float(normal_load) < target_load:                                                        # ver el martes, se paso la fuerza
               
            position = float(str(motorZ.qPOS(motorZ.axes))[18:-3])
            ztarget = position + 0.0001
            motorZ.MOV(motorZ.axes, ztarget)
            normal_load = take_force_normal()

        elif float(normal_load) > target_load:

            position = float(str(motorZ.qPOS(motorZ.axes))[18:-3])
            ztarget = position - 0.0001
            motorZ.MOV(motorZ.axes, ztarget)
            normal_load = take_force_normal()
        
        elif float(normal_load) == target_load:
            time.sleep(0.1)


### DATA TO CSV ###
def save_data():
    
    # Electric parameters #
    for x in range(len(i_meas_list[:])):
    #for x in range(len(i_meas_list[0:])):
        df_results.loc[x, 'Current'] = (i_meas_list[x-1][0])
        df_results.loc[x, 'Voltage'] = (v_meas_list[x-1][0])
        resist = v_meas_list[x-1][0] / i_meas_list[x-1][0]
        resistance.append(abs(resist))
    
    for x in range(len(forces_list[:])):
        for y in range(meas):
            f_list.append(forces_list[x])
    

    df_results['Resistance'] = resistance
    df_results['Force'] = f_list

    print(df_results)

    current_date = datetime.datetime.now()
    filename = (str(current_date.year)+'.'+str(current_date.month)+'.'+str(current_date.day)+'-'+str(current_date.hour)+'-'+str(current_date.minute)+'-'+str(current_date.second)+'.csv')

    df_results.to_csv(r'C:\Users\Labor\Desktop\{}'.format(filename), index=False)


##########################
### THREADED FUNCTIONS ###
##########################
def start_threading():
    global is_done
    is_done = False

    global j
    j = 0

    #for j in range(len(forces_list)):
    while j < (len(forces_list)):

        time.sleep(0.5)

        t1 = threading.Thread(target=ecr, args=(meas,))
        t2 = threading.Thread(target=control_normal_load, args=(forces_list[j],))
        
        if j > 0:
            t3 = threading.Thread(target=next_load, args=(forces_list[j],))
            t3.start()
            t3.join()
            time.sleep(0.5)

        #is_done = False

        t2.start()                                                                                              # start controlling normal load
        time.sleep(0.5)

        normal_load = float(take_force_normal())                                                                # take normal load measurement
        while normal_load < float((forces_list[j] * 0.99)) or normal_load > float((forces_list[j] * 1.01)):           # check that the load is above 99% or below 101% of target
            time.sleep(0.5)                                                                                     # if it isnt, wait 0.5
            normal_load = float(take_force_normal())                                                                    # take measurement again
        
        # once normal load is between 99% and 101%, start ECR measurements
        #if normal_load > float((target_load * 0.99)) and normal_load < float((target_load * 1.01)):
        time.sleep(0.5)
        t1.start()
        t1.join()
        time.sleep(0.5)
        #time.sleep(1)

        if t1.is_alive() == False:                                                                              # when ECR is done, stop controlling normal load
            is_done = True
            #t2.join()
            print(t1.is_alive())
        #t2.join()

        j += 1                                                                                                  # next force
        time.sleep(1)
        is_done = False



start_threading()

save_data()


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



retreat_stages(targetZ=10,targetX=26,vel_Z=15,vel_X=15,wait=5)
print('Closing program...')

elapse_time = (time.perf_counter() / 60)
print('Elapse time: {} min'.format(elapse_time))


#########################################
##### CLOSES CONNECTION WITH MOTORS #####
#########################################

motorZ.CloseDaisyChain()