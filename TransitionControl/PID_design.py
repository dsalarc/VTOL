#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 20:21:50 2023

@author: dsalarc
"""
import gym
import numpy as np
import matplotlib.pyplot as plt
from AuxFunctions import SaveSelection
import control as ct

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
except:
    pass

# %% 
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')

obs = TestEnv.reset(VX_mps = 0, VZ_mps = 0.0, THETA = 0.0, DispMessages = False, Linearize = True,
                    TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)

# %%
Aircraft = {}
Aircraft['SpeedIncluded'] = {}
Aircraft['SpeedIncluded']['SS'] = ct.ss(TestEnv.TrimData['Linear']['A'] , 
                                          TestEnv.TrimData['Linear']['B'] , 
                                          TestEnv.TrimData['Linear']['C'] , 
                                          TestEnv.TrimData['Linear']['D'] , 
                                          inputs=TestEnv.TrimData['Linear']['InpNames'] , 
                                          outputs = TestEnv.TrimData['Linear']['OutNames'],
                                          name = 'Aircraft' )
# %%
Sensor_q = {}
Sensor_q['wn_radps']  = 40
Sensor_q['SS'] = ct.ss(np.array([-Sensor_q['wn_radps'] ]),
                       np.array([1]),
                       np.array([Sensor_q['wn_radps'] ]),
                       np.array([0]) ,
                       inputs='Q_degps', outputs='Q_sen_degps', name = 'Sensor_q')

Sensor_t = {}
Sensor_t['wn_radps']  = 40
Sensor_t['TF'] = ct.tf([1],[1/Sensor_t['wn_radps'] , 1 ])
Sensor_t['SS'] = ct.ss(np.array([-Sensor_t['wn_radps'] ]),
                       np.array([1]),
                       np.array([Sensor_t['wn_radps'] ]),
                       np.array([0]) ,
                       inputs='Theta_deg', outputs='Theta_sen_deg', name = 'Sensor_t')

# M,P,w = ct.bode(Sensor['TF'],omega = np.linspace(0,100,num=100))

# plt.figure
# plt.subplot(2,1,1)
# plt.grid('on')
# plt.plot(w,20*np.log10(M))
# plt.xscale('log')

# plt.subplot(2,1,2)
# plt.grid('on')
# plt.plot(w,np.rad2deg(P))
# plt.xscale('log')

# plt.show()
# %%
Actuator_eng = {}
Actuator_eng['wn_radps']  = 40
Actuator_eng['SS'] = ct.ss(np.array([-Actuator_eng['wn_radps'] ]),
                           np.array([1]),
                           np.array([Actuator_eng['wn_radps'] ]),
                           np.array([0]) ,
                           inputs='PitchCmd_u', outputs='PitchThrottle_u', name = 'Actuator_eng')

# %%
Kqp = 0.02
Kqi = 0.0
Kt  = 1

Control = {}
Control['SS'] = ct.ss(np.array([0])               ,  
                   np.array([[-1, -Kt, +1, +Kt]])   ,
                   np.array([[Kqi],
                             [  0]])                , 
                   np.array([[-Kqp, -Kt*Kqp, +Kqp, +Kt*Kqp],
                             [   0,     -Kt,    1,     +Kt]]) , 
                   inputs = ['Q_degps', 'Theta_deg', 'Q_ref_degps', 'Theta_ref_deg'] , 
                   outputs = ['PitchCmd', 'Q_cmd_degps'],
                   name = 'Control' )

# plt.figure
# for i in range(Control['SS'].ninputs):
#     plt.subplot(2,2,i+1)
#     T, yout = ct.step_response(Control['SS'] , T=10, input = i)
#     plt.plot(T,yout[0][0])
#     plt.grid('on')
#     plt.ylabel('TrimPitch / ' + Control['SS'].input_labels[i])
#     plt.xlim([0,10] )
# plt.show()

# %%
ClosedLoop = ct.interconnect(
                            [Aircraft['SpeedIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , Actuator_eng['SS'] ],
                            connections=[
                                         ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                         ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                         ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                         ['Control.Theta_deg'  , 'Sensor_t.Theta_sen_deg'],
                                         ['Actuator_eng.PitchCmd_u' , 'Control.PitchCmd'],
                                         ['Aircraft.PitchThrottle' , 'Actuator_eng.PitchThrottle_u']],
                            name = 'ClosedLoop' , 
                            inplist = ['Control.Q_ref_degps', 'Control.Theta_ref_deg'],
                            inputs = ['Q_ref_degps', 'Theta_ref_deg'],
                            outlist = ['Aircraft.Theta_deg', 'Aircraft.Q_degps', 'Control.PitchCmd', 'Control.Q_cmd_degps'],
                            outputs = ['Theta_deg', 'Q_degps' , 'PitchCmd_u' , 'Q_cmd_degps'])

M,P,w = ct.bode(ClosedLoop,omega = np.linspace(0,100,num=100))
# %%
T, yout = ct.step_response(ClosedLoop , T=5, input = 1)

# plt.close('all')
plt.figure(2)
for i in range(len(yout)):
    plt.subplot(2,2,i+1)
    plt.plot(T,yout[i][0])
    plt.grid('on')
    plt.ylabel(ClosedLoop.output_labels[i])
    plt.xlim([0,max(T)] )
plt.show()

