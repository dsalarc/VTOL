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
TrimVec['VX_mps'] = np.arange(0.0, 65+0.1, 5.0)  #np.array([0 , 5, 10, 20, 28, 30, 32, 35, 40, 42, 45, 50, 55, 58, 60, 62, 65])
# TrimVec['VX_mps'] = np.array([ 10.0, 20.0, 25.0, 27.0, 28.0, 29.0, 29.5 ,   30.0, 32.0, 34.0, 36.0, 38.0, 40.0, 50.0])
# TrimVec['VX_mps'] = np.array([ 10.0, 20.0, 29.0, 30.0, 40.0, 50.0])
# TrimVec['VX_mps'] = np.array([ 10.0, 25.0])
# TrimVec['VX_mps'] = np.array([55, 60])
TrimVec['VX_mps'] = np.arange(60, 60.1, 1)  #np.array([0 , 5, 10, 20, 28, 30, 32, 35, 40, 42, 45, 50, 55, 58, 60, 62, 65])
TrimVec['Throttle'] =  np.arange(-1.0, 1.05, 0.1)

TrimRes = {}
TrimRes['Trimmed']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W1_Tilt_deg']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W2_Tilt_deg']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['PitchThrottle'] = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Elev2_u']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Throttle']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Thrust_1']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Thrust_5']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['RPM_1']         = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['RPM_5']         = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['CT_1']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['CT_5']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['J_1']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['J_5']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Elevon3']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Elevon4']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['i1_A']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['i2_A']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['V1_V']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['V2_V']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Throttle1_p']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Throttle5_p']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Reward']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W1_CLS']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W2_CLS']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W1_CDS']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W2_CDS']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['AX_mps']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['AZ_mps']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['dq_dElev']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['dq_dPThr']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['dq_dq']         = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['dq_dW']         = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['test']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W1_Alpha_deg']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['W2_Alpha_deg']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))
TrimRes['Actions']       = np.zeros((5,len(TrimVec['VX_mps']) , len(TrimVec['Throttle'])))

# %% START ENV
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')
        

# %% RUN SIM
t_ini = time.time()
for n_t in range(len(TrimVec['Throttle'])):
    Throttle_u = TrimVec['Throttle'][n_t]
    for n_sp in range(len(TrimVec['VX_mps'])):
        VX_mps = TrimVec['VX_mps'][n_sp]
        print("\nTrimming " + str(Throttle_u) + "  / " + str(VX_mps) + " m/s")
        # obs = TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, THETA = 0.0, Linearize = True, DispMessages = True, Elevator_deg = None)
        # obs = TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, THETA = 0.0, Linearize = True, DispMessages = True)
        # obs = TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, THETA = 0.0, Tilt_deg = None    , Linearize = True, DispMessages = True, Elevator_deg = 0)
        # obs = TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, THETA = 0.0, Tilt_deg = None    , Linearize = True, DispMessages = True, Elevator_deg = None)
        obs = TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, THETA = 0.0, Throttle_u = Throttle_u, Linearize = True, DispMessages = True, Elevator_deg = None)
    
        TrimRes['Trimmed'][n_sp,n_t]       = TestEnv.TrimData['Trimmed']
        TrimRes['W1_Tilt_deg'][n_sp,n_t]   = TestEnv.TrimData['info']['AERO']['Wing1']['Incidence_deg']
        TrimRes['W2_Tilt_deg'][n_sp,n_t]   = TestEnv.TrimData['info']['AERO']['Wing2']['Incidence_deg']
        TrimRes['PitchThrottle'][n_sp,n_t] = TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')]
        TrimRes['Elev2_u'][n_sp,n_t]       = TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Elevator')]
        TrimRes['Throttle'][n_sp,n_t]      = TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')]
        TrimRes['Thrust_1'][n_sp,n_t]      = TestEnv.TrimData['info']['MOT']['Thrust_N'][0]
        TrimRes['Thrust_5'][n_sp,n_t]      = TestEnv.TrimData['info']['MOT']['Thrust_N'][4]
        TrimRes['RPM_1'][n_sp,n_t]         = TestEnv.TrimData['info']['MOT']['RPM'][0]
        TrimRes['RPM_5'][n_sp,n_t]         = TestEnv.TrimData['info']['MOT']['RPM'][4]
        TrimRes['CT_1'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['PROPELLER']['obj'][0].CT
        TrimRes['CT_5'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['PROPELLER']['obj'][4].CT
        TrimRes['J_1'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['PROPELLER']['obj'][0].J
        TrimRes['J_5'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['PROPELLER']['obj'][4].J
        TrimRes['Elevon3'][n_sp,n_t]       = TestEnv.TrimData['info']['CONT']['Elevon_deg'][2]
        TrimRes['Elevon4'][n_sp,n_t]       = TestEnv.TrimData['info']['CONT']['Elevon_deg'][3]
        TrimRes['Actions'][:,n_sp,n_t]     = TestEnv.TrimData['Action']
        TrimRes['i1_A'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][0].MOTOR.i_A
        TrimRes['i2_A'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][4].MOTOR.i_A
        TrimRes['V1_V'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][0].V_V
        TrimRes['V2_V'][n_sp,n_t]          = TestEnv.TrimData['info']['MOT']['ASSEMBLY']['obj'][4].V_V
        TrimRes['Throttle1_p'][n_sp,n_t]   = TestEnv.TrimData['info']['CONT']['Throttle_p'][0]
        TrimRes['Throttle5_p'][n_sp,n_t]   = TestEnv.TrimData['info']['CONT']['Throttle_p'][4]
        TrimRes['Reward'][n_sp,n_t]        = TestEnv.LastReward
        TrimRes['W1_CLS'][n_sp,n_t]        = TestEnv.TrimData['info']['AERO']['Wing1']['CLS_25Local']
        TrimRes['W2_CLS'][n_sp,n_t]        = TestEnv.TrimData['info']['AERO']['Wing2']['CLS_25Local']
        TrimRes['W1_CDS'][n_sp,n_t]        = TestEnv.TrimData['info']['AERO']['Wing1']['CDS_25Local']
        TrimRes['W2_CDS'][n_sp,n_t]        = TestEnv.TrimData['info']['AERO']['Wing2']['CDS_25Local']
        TrimRes['AX_mps'][n_sp,n_t]        = TestEnv.TrimData['info']['EQM']['AccLin_EarthAx_mps2'][0]
        TrimRes['AZ_mps'][n_sp,n_t]        = TestEnv.TrimData['info']['EQM']['AccLin_EarthAx_mps2'][2]
        # TrimRes['test'][n_sp,n_t]        = TestEnv.TrimData['info']['AERO']['Elevon']['MYB_Nm'][-1]
        TrimRes['test'][n_sp,n_t]        = np.max(np.abs(TestEnv.TrimData['info']['AERO']['Elevon']['CMS_MRC']))
        TrimRes['test'][n_sp,n_t]        = TestEnv.TrimData['info']['AERO']['Elevon']['dCMSde_MRC'][-1]
        # TrimRes['test'][n_sp,n_t]        = np.max(np.abs(TestEnv.TrimData['info']['AERO']['Wing1']['MYB_Nm']))
        # TrimRes['test'][n_sp,n_t]        = TestEnv.TrimData['info']['ATM']['DynPres_Pa']
        TrimRes['dq_dq'][n_sp,n_t]         = TestEnv.TrimData['Linear']['A'][TestEnv.TrimData['Linear']['StaNames'].index('Q_radps'),
                                                                         TestEnv.TrimData['Linear']['StaNames'].index('Q_radps')]
        TrimRes['dq_dW'][n_sp,n_t]         = TestEnv.TrimData['Linear']['A'][TestEnv.TrimData['Linear']['StaNames'].index('Q_radps'),
                                                                         TestEnv.TrimData['Linear']['StaNames'].index('W_mps')]
        TrimRes['dq_dElev'][n_sp,n_t]      = TestEnv.TrimData['Linear']['B'][TestEnv.TrimData['Linear']['StaNames'].index('Q_radps'),
                                                                         TestEnv.TrimData['Linear']['InpNames'].index('W2_Elevator')]
        TrimRes['dq_dPThr'][n_sp,n_t]      = TestEnv.TrimData['Linear']['B'][TestEnv.TrimData['Linear']['StaNames'].index('Q_radps'),
                                                                         TestEnv.TrimData['Linear']['InpNames'].index('PitchThrottle')]
        
        print("Iter n: " + str(TestEnv.TrimData['iter_n']))



t_end = time.time()
print("Execution Time: " + str(t_end - t_ini))
print(str(TrimRes['Trimmed']))

# %% PLOT
plt_l = 3
plt_c = 4

plt.rcParams.update({'font.size': 10})

fig = plt.figure()

colors = ['r', 'm', 'k', 'c', 'b']
colors = ['m', 'k', 'c', 'b']

for n_t in range(len(TrimVec['Throttle'])):
    color = colors[np.mod(n_t,len(colors))]
    plt_n = 1
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['W1_Tilt_deg'][:,n_t],color +'-', linewidth = 2, label='W1 Tilt')
    # plt.plot(TrimVec['VX_mps'] , TrimRes['W2_Tilt_deg'],color +'--', linewidth = 2,label='W2 Tilt')
    plt.plot(TrimVec['VX_mps'][TrimRes['Trimmed'][:,n_t]==1] , TrimRes['W1_Tilt_deg'][TrimRes['Trimmed'][:,n_t]==1,n_t],'ob', markerfacecolor = 'b', linewidth = 2,label='W1 Tilt')
    plt.plot(TrimVec['VX_mps'][TrimRes['Trimmed'][:,n_t]==0] , TrimRes['W1_Tilt_deg'][TrimRes['Trimmed'][:,n_t]==0,n_t],'or', markerfacecolor = 'r', linewidth = 2,label='W1 Tilt')
    # plt.legend(loc='best')
    # plt.ylim([0, 90])
    plt.yticks(np.arange(0,120,30))
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Wing Tilt [deg]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle'][:,n_t],color +'-', linewidth = 2,label='W1 Throtte')
    plt.plot(TrimVec['VX_mps'][TrimRes['Trimmed'][:,n_t]==1] , TrimRes['Throttle'][TrimRes['Trimmed'][:,n_t]==1,n_t],'ob', markerfacecolor = 'b', linewidth = 2,label='W1 Throtte')
    plt.plot(TrimVec['VX_mps'][TrimRes['Trimmed'][:,n_t]==0] , TrimRes['Throttle'][TrimRes['Trimmed'][:,n_t]==0,n_t],'or', markerfacecolor = 'r', linewidth = 2,label='W1 Throtte')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.ylim([-1, +1])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Throttle Action')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['PitchThrottle'][:,n_t],color +'-', linewidth = 2,label='W1 Tilt')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.ylim([-0.10, 0.05])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('PitchThrottle [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_1'][:,n_t],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_5'][:,n_t],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    plt.ylim([0, 1250])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Thrust [N]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_1'][:,n_t],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_5'][:,n_t],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    plt.ylim([1000, 3000])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('RPM')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Elevon3'][:,n_t],color +'-', linewidth = 2,label='Elevon 3')
    plt.plot(TrimVec['VX_mps'] , TrimRes['Elevon4'][:,n_t],color +'--', linewidth = 2,label='Elevon 4')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.legend(loc='best')
    plt.ylim([-15, 15])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Elevon [deg]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['i1_A'][:,n_t],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['VX_mps'] , TrimRes['i2_A'][:,n_t],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.legend(loc='best')
    plt.ylim([0, 180])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Current [A]')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['V1_V'],color +'-', linewidth = 2,label='Motor 1')
    # plt.plot(TrimVec['VX_mps'] , TrimRes['V2_V'],color +'--', linewidth = 2,label='Motor 5')
    # # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([0, 450])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('Voltage [V]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['W1_CLS'][:,n_t],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['VX_mps'] , TrimRes['W2_CLS'][:,n_t],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.legend(loc='best')
    plt.ylim([0, 2])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Wing CLS')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle1_p'],color +'-', linewidth = 2,label='Motor 1')
    # plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle5_p'],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='best')
    # # plt.ylim([100, 150])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('Throttle [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['AX_mps'][:,n_t],color +'-', linewidth = 2)
    plt.legend(loc='best')
    # plt.ylim([100, 150])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Inertial X Acceleration [m/s2]')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['Reward'],color +'-', linewidth = 2,label='Motor 1')
    # plt.legend(loc='best')
    # # plt.ylim([100, 150])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('Reward')


    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['dq_dElev'][:,n_t],color +'-', linewidth = 2)
    # plt.legend(loc='best')
    # # plt.ylim([100, 150])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('dq_dElev')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['dq_dq'][:,n_t],color +'-', linewidth = 2)
    # plt.legend(loc='best')
    # # plt.ylim([100, 150])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('dq_dq')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['dq_dW'][:,n_t],color +'-', linewidth = 2)
    # plt.legend(loc='best')
    # # plt.ylim([100, 150])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('dq_dW')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['test'][:,n_t],color +'-', linewidth = 2)
    # plt.legend(loc='best')
    # # plt.ylim([100, 150])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('test')

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


# %% PLOT
plt_l = 3
plt_c = 4

plt.rcParams.update({'font.size': 10})

fig = plt.figure()

colors = ['r', 'm', 'k', 'c', 'b']
colors = ['m', 'k', 'c', 'b']

for n_sp in range(len(TrimVec['VX_mps'])):
    color = colors[n_sp]
    plt_n = 1
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['W1_Tilt_deg'][n_sp,:],color +'-', linewidth = 2, label='W1 Tilt')
    # plt.plot(TrimVec['Throttle'] , TrimRes['W2_Tilt_deg'],color +'--', linewidth = 2,label='W2 Tilt')
    plt.plot(TrimVec['Throttle'][TrimRes['Trimmed'][n_sp,:]==1] , TrimRes['W1_Tilt_deg'][n_sp,TrimRes['Trimmed'][n_sp,:]==1],'ob', markerfacecolor = 'b', linewidth = 2,label='W1 Tilt')
    plt.plot(TrimVec['Throttle'][TrimRes['Trimmed'][n_sp,:]==0] , TrimRes['W1_Tilt_deg'][n_sp,TrimRes['Trimmed'][n_sp,:]==0],'or', markerfacecolor = 'r', linewidth = 2,label='W1 Tilt')
    # plt.legend(loc='best')
    # plt.ylim([0, 90])
    # plt.yticks(np.arange(0,120,30))
    plt.xlabel('Tilt [deg]')
    plt.ylabel('Wing Tilt [deg]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['Throttle'][n_sp,:],color +'-', linewidth = 2,label='W1 Throtte')
    plt.plot(TrimVec['Throttle'][TrimRes['Trimmed'][n_sp,:]==1] , TrimRes['Throttle'][n_sp,TrimRes['Trimmed'][n_sp,:]==1],'ob', markerfacecolor = 'b', linewidth = 2,label='W1 Throtte')
    plt.plot(TrimVec['Throttle'][TrimRes['Trimmed'][n_sp,:]==0] , TrimRes['Throttle'][n_sp,TrimRes['Trimmed'][n_sp,:]==0],'or', markerfacecolor = 'r', linewidth = 2,label='W1 Throtte')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.ylim([-1, +1])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('Throttle Action')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['PitchThrottle'][n_sp,:],color +'-', linewidth = 2,label='W1 Tilt')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.ylim([-0.10, 0.05])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('PitchThrottle [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['Thrust_1'][n_sp,:],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['Throttle'] , TrimRes['Thrust_5'][n_sp,:],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([0, 1250])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('Thrust [N]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['RPM_1'][n_sp,:],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['Throttle'] , TrimRes['RPM_5'][n_sp,:],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([1000, 3000])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('RPM')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['CT_1'][n_sp,:],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['Throttle'] , TrimRes['CT_5'][n_sp,:],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([1000, 3000])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('CT')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['J_1'][n_sp,:],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['Throttle'] , TrimRes['J_5'][n_sp,:],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([1000, 3000])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('J')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['Elevon3'][n_sp,:],color +'-', linewidth = 2,label='Elevon 3')
    plt.plot(TrimVec['Throttle'] , TrimRes['Elevon4'][n_sp,:],color +'--', linewidth = 2,label='Elevon 4')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.legend(loc='best')
    plt.ylim([-15, 15])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('Elevon [deg]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['i1_A'][n_sp,:],color +'-', linewidth = 2,label='Motor 1')
    plt.plot(TrimVec['Throttle'] , TrimRes['i2_A'][n_sp,:],color +'--', linewidth = 2,label='Motor 5')
    # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.legend(loc='best')
    plt.ylim([0, 180])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('Current [A]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['AX_mps'][n_sp,:],color +'-', linewidth = 2)
    plt.legend(loc='best')
    # plt.ylim([100, 150])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('Inertial X Acceleration [m/s2]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['Throttle']),np.max(TrimVec['Throttle'])])
    plt.plot(TrimVec['Throttle'] , TrimRes['AZ_mps'][n_sp,:],color +'-', linewidth = 2)
    plt.legend(loc='best')
    # plt.ylim([100, 150])
    plt.xlabel('Tilt [deg]')
    plt.ylabel('Inertial Z Acceleration [m/s2]')
    
fig.set_size_inches(14, 8)
fig.tight_layout() 


plt.show()





