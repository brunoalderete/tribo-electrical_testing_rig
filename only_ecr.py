"""
Date 11.2020
@author: Chair of Functional Materials - Saarland University - Saarbrücken, Bruno Alderete
@version 1.0
"""
__author__ = 'Bruno Alderete'

#######################################################################################################################
#
# The MIT License (MIT)
#
# Copyright (c) 2020 Bruno Alderete
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

import pyvisa
import time
import pandas as pd

#############################
##### FUNCTIONS DEFINED #####
#############################
##### ERC NOW AUTOMATED #####
#############################

# Start and reset instruments
def start_instr(curr_app='100E-3',curr_prot='150E-3'):
    # Start and reset Keithley 2400
    k2400.write('*RST')
    k2400.timeout = 60000                                                       # 60 seconds timeout
    k2400.write(':ROUT:TERM REAR')
    # print(k2400.query(':ROUT:TERM?'))
    
    k2400.write(':SENS:FUNC:CONC OFF')
    k2400.write(':SOUR:FUNC CURR')
    k2400.write(f':SOUR:CURR {curr_app}')                                       # Applied current in A
    k2400.write(":SENS:FUNC 'CURR:DC'")
    k2400.write(f':SENS:CURR:PROT {curr_prot}')

    # Start and resete Keithley 2182
    k2182.write('*RST')

# Take i measurement
def take_i_meas(sour_del=1):                                                    # , sleep_time=1 commented
    k2400.write('TRIG:COUN 1')                                                  # Amount of measurements
    k2400.write(f'SOUR:DEL {sour_del}')                                         # Delay in seconds (between 0, 60). Time i is applied
    k2400.write(':FORM:ELEM CURR')
    k2400.write(':OUTP ON')
    i_meas_list.append(k2400.query_ascii_values(':READ?'))
    #time.sleep(sleep_time)                                                     # x seconds between iteration          ### commented
    # print(i_meas_list)

# Take v measurement
def take_v_meas():
    k2182.write('*RST')
    #k2182.write('*CLS')
    k2182.write(":SENS:FUNC 'VOLT'")
    k2182.write(':SENS:CHAN 1')
    k2182.write(':SENS:VOLT:CHAN1:RANG 1')                                # k2182.write(':SENS:VOLT:CHAN1:RANG:AUTO ON')
    v_meas_list.append(k2182.query_ascii_values(':READ?'))
    # print(k2182.query(':READ?'))

# Take full measurements
def take_measurement(meas=5, trigs=0, curr_app='100E-3', curr_prot='150E-3', sour_del=1, sleep_time=1):


    start_instr(curr_app, curr_prot)

    while trigs < meas:

        take_i_meas(sour_del)                                           # , sleep_time commented
        take_v_meas()
        #print(i_meas_list)
        #print(v_meas_list)
        trigs += 1
        k2400.write(':OUTP OFF')        # turns off smu between measurements
    
    #k2400.write(':OUTP OFF')           # commented

rm = pyvisa.ResourceManager('@py')

k2400 = rm.open_resource('GPIB::24::INSTR')
k2182 = rm.open_resource('GPIB::7::INSTR')

trigs = 0                   # counter
meas = 4                   # 10 measurements per force
current = 100E-3            # 100 mA
sleep_time = 1              # time between measurements

i_meas_list = []
v_meas_list = []
resistance = []

df_values = pd.DataFrame(columns=['Current', 'Voltage', 'Resistance'])

while trigs < meas:

    if trigs % 2 == 0:
        take_measurement(meas=1, trigs=0, curr_app=str(current), curr_prot='150E-3', sour_del=1)                 # , sleep_time=1 commented
        time.sleep(sleep_time)
    else:
        take_measurement(meas=1, trigs=0, curr_app='-' + str(current), curr_prot='150E-3', sour_del=1)                 # , sleep_time=1 commented
        time.sleep(sleep_time)
    trigs += 1

trigs = 0       # returns counter to zero


y = len(i_meas_list[0:])

# adds values to df and divides voltage by current to get resistance in list
for x in range(y):
    df_values.loc[x, 'Current'] = (i_meas_list[x-1][0])
    df_values.loc[x, 'Voltage'] = (v_meas_list[x-1][0])
    resist = v_meas_list[x-1][0] / i_meas_list[x-1][0]
    resistance.append(resist)

#adds resistance list to df
df_values['Resistance'] = resistance

print(df_values)

# Writes data to csv file
#df_values.to_csv(r'E:\User data\Bruno Alderete\ECR new Bruno\IV_measurements_0.csv', index=False)