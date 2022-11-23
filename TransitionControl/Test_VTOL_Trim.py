#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 18:29:29 2021

@author: dsalarc
"""

import gym
import numpy as np
import matplotlib.pyplot as plt

# %% INPUTS
TrimVec = {}
TrimVec['VX_mps'] = np.array([0 , 5, 10, 20, 30, 40, 50, 60])
# TrimVec['VX_mps'] = np.array([40, 50, 60])

TrimRes = {}
TrimRes['Trimmed']       = np.zeros(len(TrimVec['VX_mps']))
TrimRes['W1_Tilt_deg']   = np.zeros(len(TrimVec['VX_mps']))
TrimRes['W2_Tilt_deg']   = np.zeros(len(TrimVec['VX_mps']))
TrimRes['PitchThrottle'] = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Throttle']      = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Thrust_1']      = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Thrust_5']      = np.zeros(len(TrimVec['VX_mps']))
TrimRes['RPM_1']      = np.zeros(len(TrimVec['VX_mps']))
TrimRes['RPM_5']      = np.zeros(len(TrimVec['VX_mps']))

# %% START ENV
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')
        

# %% RUN SIM
# obs,TrimAction = TestEnv.reset(Z=0,W=0,THETA=np.deg2rad(0), PHI=np.deg2rad(0), PSI=np.deg2rad(0), PaxIn = np.array([1,1]))

for n_sp in range(len(TrimVec['VX_mps'])):
    VX_mps = TrimVec['VX_mps'][n_sp]
    print("Trimming " + str(VX_mps) + " m/s")
    obs = TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, THETA = 0.0)

    TrimRes['Trimmed'][n_sp]       = TestEnv.TrimData['Trimmed']
    TrimRes['W1_Tilt_deg'][n_sp]   = TestEnv.TrimData['info']['AERO']['Wing1']['Incidence_deg']
    TrimRes['W2_Tilt_deg'][n_sp]   = TestEnv.TrimData['info']['AERO']['Wing2']['Incidence_deg']
    TrimRes['PitchThrottle'][n_sp] = TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')]
    TrimRes['Throttle'][n_sp]      = TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')]
    TrimRes['Thrust_1'][n_sp]      = TestEnv.TrimData['info']['MOT']['Thrust_N'][0]
    TrimRes['Thrust_5'][n_sp]      = TestEnv.TrimData['info']['MOT']['Thrust_N'][4]
    TrimRes['RPM_1'][n_sp]         = TestEnv.TrimData['info']['MOT']['RPM'][0]
    TrimRes['RPM_5'][n_sp]         = TestEnv.TrimData['info']['MOT']['RPM'][4]
    print("Iter n: " + str(TestEnv.TrimData['iter_n']))



print(str(TrimRes['Trimmed']))

# %% PLOT
plt_l = 3
plt_c = 2
plt_n = 1

fig = plt.figure()

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['W1_Tilt_deg'],label='W1 Tilt')
plt.plot(TrimVec['VX_mps'] , TrimRes['W2_Tilt_deg'],label='W2 Tilt')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Wing Tilt [deg]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle'],label='W1 Tilt')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Throttle [p]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['PitchThrottle'],label='W1 Tilt')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('PitchThrottle [p]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_1'],label='Motor 1')
plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_5'],label='Motor 5')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Thrust [N]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_1'],label='Motor 1')
plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_5'],label='Motor 5')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('RPM]')

fig.set_size_inches(10, 10)
fig.tight_layout() 
plt.show()


