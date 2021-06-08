#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 18:29:29 2021

@author: dsalarc
"""

import gym
import numpy as np
import matplotlib.pyplot as plt

# %% FUNCTIONS
def AppendValue(SaveVec,name,Value,):
    if name in SaveVec:
        SaveVec[name] = np.append(SaveVec[name],Value)
    else:
        SaveVec[name] = np.array([Value])
         
    return SaveVec
    
def SaveSelection(step,SaveVec,info):
    SaveVec = AppendValue(SaveVec,'X_m',info['EQM']['PosLin_EarthAx_m'][0])
    SaveVec = AppendValue(SaveVec,'Y_m',info['EQM']['PosLin_EarthAx_m'][1])
    SaveVec = AppendValue(SaveVec,'Z_m',-info['EQM']['PosLin_EarthAx_m'][2])

    SaveVec = AppendValue(SaveVec,'U_mps',info['EQM']['VelLin_BodyAx_mps'][0])
    SaveVec = AppendValue(SaveVec,'V_mps',info['EQM']['VelLin_BodyAx_mps'][1])
    SaveVec = AppendValue(SaveVec,'W_mps',-info['EQM']['VelLin_BodyAx_mps'][2])
    
    SaveVec = AppendValue(SaveVec,'AX_mps2',info['EQM']['AccLin_BodyAx_mps2'][0])
    SaveVec = AppendValue(SaveVec,'AY_mps2',info['EQM']['AccLin_BodyAx_mps2'][1])
    SaveVec = AppendValue(SaveVec,'AZ_mps2',-info['EQM']['AccLin_BodyAx_mps2'][2])
    
    SaveVec = AppendValue(SaveVec,'Phi_deg',np.rad2deg(info['EQM']['EulerAngles_rad'][0]))
    SaveVec = AppendValue(SaveVec,'Theta_deg',np.rad2deg(info['EQM']['EulerAngles_rad'][1]))
    SaveVec = AppendValue(SaveVec,'Psi_deg',np.rad2deg(info['EQM']['EulerAngles_rad'][2]))
 
    SaveVec = AppendValue(SaveVec,'FX_N',info['EQM']['TotalForce'][0])
    SaveVec = AppendValue(SaveVec,'FY_N',info['EQM']['TotalForce'][1])
    SaveVec = AppendValue(SaveVec,'FZ_N',-info['EQM']['TotalForce'][2])
 
    SaveVec = AppendValue(SaveVec,'Alpha_deg',info['ATM']['Alpha_deg'])

    SaveVec = AppendValue(SaveVec,'W1_Alpha_deg',info['AERO']['Wing']['Alpha_deg'][0])
    SaveVec = AppendValue(SaveVec,'W2_Alpha_deg',info['AERO']['Wing']['Alpha_deg'][1])
    SaveVec = AppendValue(SaveVec,'W1_CLS',info['AERO']['Wing']['CLS'][0])
    SaveVec = AppendValue(SaveVec,'W2_CLS',info['AERO']['Wing']['CLS'][1])
    SaveVec = AppendValue(SaveVec,'W1_CDS',info['AERO']['Wing']['CDS'][0])
    SaveVec = AppendValue(SaveVec,'W2_CDS',info['AERO']['Wing']['CDS'][1])

    return SaveVec


# %% START ENV
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')

# %% RUN SIM
obs = TestEnv.reset(np.array([0,0]))
TestEnv.render()
SaveVec = {}

# %
# Hardcoded best agent: always go left!
n_steps = 1000
for step in range(n_steps):

    obs, reward, done, info = TestEnv.step(np.array([0]))
    SaveVec = SaveSelection(step,SaveVec,info)
    if done:
      print("Goal reached!", "reward=", reward)
      break

# % PLOT IN TIME
t_step  = 0.05
SimTime = n_steps*t_step
TimeVec = np.arange(t_step,SimTime,t_step)
if (SimTime - TimeVec[-1]) > t_step/2:
    TimeVec = np.append(TimeVec,SimTime)
    
fig = plt.figure()

plt_l = 4
plt_c = 2
plt_n = 1

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['Z_m'])
plt.ylabel('Z [m]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['W_mps'])
plt.ylabel('W [m/s]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['AZ_mps2'])
plt.ylabel('AZ [m/s²]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['FZ_N'])
plt.ylabel('FZ [N]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['Theta_deg'])
plt.ylabel('Theta [deg]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['Phi_deg'])
plt.ylabel('Phi [deg]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['Alpha_deg'],label = 'Alpha_deg')
plt.plot(TimeVec,SaveVec['W1_Alpha_deg'],label = 'W1_Alpha_deg')
plt.plot(TimeVec,SaveVec['W2_Alpha_deg'],label = 'W2_Alpha_deg')
plt.ylabel('Alpha [deg]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['Phi_deg'])
plt.ylabel('Phi [deg]')
plt.xlabel('Time [s]')

fig.set_size_inches(6, 7.5)
fig.tight_layout() 

plt.show()

fig = plt.figure()
plt.plot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['Alpha_deg'],label = 'Alpha_deg')
plt.plot(TimeVec,SaveVec['W1_Alpha_deg'],label = 'W1_Alpha_deg')
plt.plot(TimeVec,SaveVec['W2_Alpha_deg'],label = 'W2_Alpha_deg')
plt.legend()
plt.ylabel('Alpha [deg]')
plt.xlabel('Time [s]')

fig = plt.figure()
plt.plot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['W1_CLS'],label = 'W1_CLS')
plt.plot(TimeVec,SaveVec['W2_CLS'],label = 'W2_CLS')
plt.legend()
plt.ylabel('Alpha [deg]')
plt.xlabel('Time [s]')

fig = plt.figure()
plt.plot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,SimTime])
plt.plot(TimeVec,SaveVec['U_mps'],label = 'U_mps')
plt.plot(TimeVec,SaveVec['V_mps'],label = 'V_mps')
plt.legend()
plt.ylabel('Alpha [deg]')
plt.xlabel('Time [s]')
