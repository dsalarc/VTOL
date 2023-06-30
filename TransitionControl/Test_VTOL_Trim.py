#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 18:29:29 2021

@author: dsalarc
"""

import gym
import numpy as np
import matplotlib.pyplot as plt
import time

# %% INPUTS
TrimVec = {}
TrimVec['VX_mps'] = np.array([0 , 5, 10, 20, 30, 35, 40, 42, 45, 50, 55, 58, 60, 62, 65])
# TrimVec['VX_mps'] = np.array([0, 10])

TrimRes = {}
TrimRes['Trimmed']       = np.zeros(len(TrimVec['VX_mps']))
TrimRes['W1_Tilt_deg']   = np.zeros(len(TrimVec['VX_mps']))
TrimRes['W2_Tilt_deg']   = np.zeros(len(TrimVec['VX_mps']))
TrimRes['PitchThrottle'] = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Elev2_u']       = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Throttle']      = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Thrust_1']      = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Thrust_5']      = np.zeros(len(TrimVec['VX_mps']))
TrimRes['RPM_1']         = np.zeros(len(TrimVec['VX_mps']))
TrimRes['RPM_5']         = np.zeros(len(TrimVec['VX_mps']))
TrimRes['RPM_5']         = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Elevon3']       = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Elevon4']       = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Actions']       = np.zeros((5,len(TrimVec['VX_mps'])))
TrimRes['i1_A']          = np.zeros(len(TrimVec['VX_mps']))
TrimRes['i2_A']          = np.zeros(len(TrimVec['VX_mps']))
TrimRes['V1_V']          = np.zeros(len(TrimVec['VX_mps']))
TrimRes['V2_V']          = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Throttle1_p']   = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Throttle5_p']   = np.zeros(len(TrimVec['VX_mps']))
TrimRes['Reward']        = np.zeros(len(TrimVec['VX_mps']))
TrimRes['W1_CLS']        = np.zeros(len(TrimVec['VX_mps']))
TrimRes['W2_CLS']        = np.zeros(len(TrimVec['VX_mps']))

# %% START ENV
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')
        

# %% RUN SIM
t_ini = time.time()
for n_sp in range(len(TrimVec['VX_mps'])):
    VX_mps = TrimVec['VX_mps'][n_sp]
    print("\nTrimming " + str(VX_mps) + " m/s")
    obs = TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, THETA = 0.0, DispMessages = True)

    TrimRes['Trimmed'][n_sp]       = TestEnv.TrimData['Trimmed']
    TrimRes['W1_Tilt_deg'][n_sp]   = TestEnv.TrimData['info']['AERO']['Wing1']['Incidence_deg']
    TrimRes['W2_Tilt_deg'][n_sp]   = TestEnv.TrimData['info']['AERO']['Wing2']['Incidence_deg']
    TrimRes['PitchThrottle'][n_sp] = TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')]
    TrimRes['Elev2_u'][n_sp]       = TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Elevator')]
    TrimRes['Throttle'][n_sp]      = TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')]
    TrimRes['Thrust_1'][n_sp]      = TestEnv.TrimData['info']['MOT']['Thrust_N'][0]
    TrimRes['Thrust_5'][n_sp]      = TestEnv.TrimData['info']['MOT']['Thrust_N'][4]
    TrimRes['RPM_1'][n_sp]         = TestEnv.TrimData['info']['MOT']['RPM'][0]
    TrimRes['RPM_5'][n_sp]         = TestEnv.TrimData['info']['MOT']['RPM'][4]
    TrimRes['Elevon3'][n_sp]       = TestEnv.TrimData['info']['CONT']['Elevon_deg'][2]
    TrimRes['Elevon4'][n_sp]       = TestEnv.TrimData['info']['CONT']['Elevon_deg'][3]
    TrimRes['Actions'][:,n_sp]     = TestEnv.TrimData['Action']
    TrimRes['i1_A'][n_sp]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][0].MOTOR.i_A
    TrimRes['i2_A'][n_sp]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][4].MOTOR.i_A
    TrimRes['V1_V'][n_sp]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][0].V_V
    TrimRes['V2_V'][n_sp]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][4].V_V
    TrimRes['Throttle1_p'][n_sp]   = TestEnv.TrimData['info']['CONT']['Throttle_p'][0]
    TrimRes['Throttle5_p'][n_sp]   = TestEnv.TrimData['info']['CONT']['Throttle_p'][4]
    TrimRes['Reward'][n_sp]        = TestEnv.LastReward
    TrimRes['W1_CLS'][n_sp]        = TestEnv.TrimData['info']['AERO']['Wing1']['CLS_25Local']
    TrimRes['W2_CLS'][n_sp]        = TestEnv.TrimData['info']['AERO']['Wing2']['CLS_25Local']
    
    print("Iter n: " + str(TestEnv.TrimData['iter_n']))



t_end = time.time()
print("Execution Time: " + str(t_end - t_ini))
print(str(TrimRes['Trimmed']))

# %% PLOT
plt_l = 3
plt_c = 3
plt_n = 1

plt.rcParams.update({'font.size': 10})

fig = plt.figure()

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['W1_Tilt_deg'],'k-', linewidth = 2, label='W1 Tilt')
plt.plot(TrimVec['VX_mps'] , TrimRes['W2_Tilt_deg'],'k--', linewidth = 2,label='W2 Tilt')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.legend(loc='best')
plt.ylim([0, 90])
plt.yticks(np.arange(0,120,30))
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Wing Tilt [deg]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle'],'k-', linewidth = 2,label='W1 Tilt')
plt.plot(TrimVec['VX_mps'][TrimRes['Trimmed']==1] , TrimRes['Throttle'][TrimRes['Trimmed']==1],'ob', markerfacecolor = 'b', linewidth = 2,label='W1 Tilt')
plt.plot(TrimVec['VX_mps'][TrimRes['Trimmed']==0] , TrimRes['Throttle'][TrimRes['Trimmed']==0],'or', markerfacecolor = 'r', linewidth = 2,label='W1 Tilt')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.ylim([-1, +1])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Throttle Action')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['PitchThrottle'],'k-', linewidth = 2,label='W1 Tilt')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.ylim([-0.10, 0.05])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('PitchThrottle [p]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_1'],'k-', linewidth = 2,label='Motor 1')
plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_5'],'k--', linewidth = 2,label='Motor 5')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.legend(loc='best')
plt.ylim([0, 1250])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Thrust [N]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_1'],'k-', linewidth = 2,label='Motor 1')
plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_5'],'k--', linewidth = 2,label='Motor 5')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.legend(loc='best')
plt.ylim([1000, 3000])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('RPM')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['Elevon3'],'k-', linewidth = 2,label='Elevon 3')
plt.plot(TrimVec['VX_mps'] , TrimRes['Elevon4'],'k--', linewidth = 2,label='Elevon 4')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.legend(loc='best')
plt.ylim([-15, 15])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Elevon [deg]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['i1_A'],'k-', linewidth = 2,label='Motor 1')
plt.plot(TrimVec['VX_mps'] , TrimRes['i2_A'],'k--', linewidth = 2,label='Motor 5')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.legend(loc='best')
plt.ylim([0, 180])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Current [A]')

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
# plt.plot(TrimVec['VX_mps'] , TrimRes['V1_V'],'k-', linewidth = 2,label='Motor 1')
# plt.plot(TrimVec['VX_mps'] , TrimRes['V2_V'],'k--', linewidth = 2,label='Motor 5')
# # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.legend(loc='best')
# plt.ylim([0, 450])
# plt.xlabel('Inertial X Speed [m/s]')
# plt.ylabel('Voltage [V]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['W1_CLS'],'k-', linewidth = 2,label='Motor 1')
plt.plot(TrimVec['VX_mps'] , TrimRes['W2_CLS'],'k--', linewidth = 2,label='Motor 5')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.legend(loc='best')
plt.ylim([0, 2])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Wing CLS')

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
# plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle1_p'],'k-', linewidth = 2,label='Motor 1')
# plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle5_p'],'k--', linewidth = 2,label='Motor 5')
# plt.legend(loc='best')
# # plt.ylim([100, 150])
# plt.xlabel('Inertial X Speed [m/s]')
# plt.ylabel('Throttle [p]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
plt.plot(TrimVec['VX_mps'] , TrimRes['Reward'],'k-', linewidth = 2,label='Motor 1')
plt.legend(loc='best')
# plt.ylim([100, 150])
plt.xlabel('Inertial X Speed [m/s]')
plt.ylabel('Reward')

# fig.set_size_inches(8, 5)
fig.set_size_inches(14, 8)
fig.tight_layout() 

np.set_printoptions(precision=1)
print("VX: " + repr(TrimVec['VX_mps']))
print("Tilt[deg]: " + repr(TrimRes['W1_Tilt_deg']))
np.set_printoptions(precision=3)
print("Throttle[u]: " + repr(TrimRes['Throttle']))
print("Elev[deg]: " + repr(TrimRes['Elevon3']))
print("PitchThrottle[u]: " + repr(TrimRes['PitchThrottle']))
print("Elev[u]: " + repr(TrimRes['Elev2_u']))

fig.savefig('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/trim_corridor.pdf', bbox_inches='tight')
print('Fig Saved')

plt.show()






