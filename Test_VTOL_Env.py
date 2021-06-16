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
 
    SaveVec = AppendValue(SaveVec,'MX_Nm',info['EQM']['TotalMoment'][0])
    SaveVec = AppendValue(SaveVec,'MY_Nm',info['EQM']['TotalMoment'][1])
    SaveVec = AppendValue(SaveVec,'MZ_Nm',-info['EQM']['TotalMoment'][2])
 
    SaveVec = AppendValue(SaveVec,'Alpha_deg',info['ATM']['Alpha_deg'])

    SaveVec = AppendValue(SaveVec,'W1_Alpha_deg',info['AERO']['Wing']['Alpha_deg'][0])
    SaveVec = AppendValue(SaveVec,'W2_Alpha_deg',info['AERO']['Wing']['Alpha_deg'][1])
    SaveVec = AppendValue(SaveVec,'W1_CLS',info['AERO']['Wing']['CLS'][0])
    SaveVec = AppendValue(SaveVec,'W2_CLS',info['AERO']['Wing']['CLS'][1])
    SaveVec = AppendValue(SaveVec,'W1_CDS',info['AERO']['Wing']['CDS'][0])
    SaveVec = AppendValue(SaveVec,'W2_CDS',info['AERO']['Wing']['CDS'][1])

    SaveVec = AppendValue(SaveVec,'RPM_1',info['MOT']['RPM'][0])
    SaveVec = AppendValue(SaveVec,'RPM_4',info['MOT']['RPM'][3])
    SaveVec = AppendValue(SaveVec,'RPM_5',info['MOT']['RPM'][4])
    SaveVec = AppendValue(SaveVec,'RPM_8',info['MOT']['RPM'][7])

    SaveVec = AppendValue(SaveVec,'Thrust1_N',info['MOT']['Thrust_N'][0])
    SaveVec = AppendValue(SaveVec,'Thrust4_N',info['MOT']['Thrust_N'][3])
    SaveVec = AppendValue(SaveVec,'Thrust5_N',info['MOT']['Thrust_N'][4])
    SaveVec = AppendValue(SaveVec,'Thrust8_N',info['MOT']['Thrust_N'][7])

    return SaveVec

def PID_eVTOL(Reference,obs,Last_u):
    # VERTICAL CONTROL
    KP_v = 0
    KD_v = 0
    KI_v = 0
    # INP_v = - KP_v * obs[] 
    #         - KD_v* + KI_v
    

    SUM_INP = np.sum(np.array([PitchControlAllocation(0),
                           RollControlAllocation(0),
                           YawControlAllocation(0),
                           VerticalControlAllocation(0)]),
                           axis=0)   
    return SUM_INP
    
    '''
    sta: vetor de estados na ordem:
    XL_e: vetor posição lineares no eixo da terra (X, Y, Z)
    XR_e: vetor posição rotacional no eixo da terra (phi, theta psi)  
    VL_b: vetor velocidades lineares no eixo do corpo (u,v,w)
    VR_b: vetor velocidade rotacional no eixo do corpo (p, q, r)
    '''
    
    
    return np.ones(8), np.ones(4)

def PID_Vert(WRef_mps,W_mps,AZ_mps2, Wint_m, KI=0, KD=0, KP=0, gamma = 0.99):
    Wint_m = Wint_m*gamma + (WRef_mps-W_mps)
    u = -(KP*(WRef_mps-W_mps) + KD*AZ_mps2 + KI*Wint_m)
    return u,Wint_m

def PIDd_Vert(WRef_mps,W_mps,AZ_mps2, KI=0, KP=0, gamma = 0.99):
    du = -(KI*(WRef_mps-W_mps) + KP*AZ_mps2)
    return du

def PID_Pitch(ThetaRef_rad,Theta_rad,Q_radps,KD=0,KP=0):
    u = -(KP*(Theta_rad-ThetaRef_rad) + KD*Q_radps)
    return u

def PID_Roll(PhiRef_rad,Phi_rad,P_radps,KD=0,KP=0):
    u = -(KP*(Phi_rad-PhiRef_rad) + KD*P_radps)
    return u

def PitchControlAllocation(u):    
    return np.array([+1,+1,+1,+1,-1,-1,-1,-1]) * u

def RollControlAllocation(u):    
    return np.array([+1,+1,-1,-1,+1,+1,-1,-1]) * u

def YawControlAllocation(u):    
    return np.array([-1,+1,-1,+1,+1,-1,+1,-1]) * u

def VerticalControlAllocation(u):    
    return np.array([+1,+1,+1,+1,+1,+1,+1,+1]) * u

def ControlMixer(u_Vert,u_Pitch,u_Roll,u_Yaw):
    SUM_INP = np.sum(np.array([u_Vert,u_Pitch,u_Roll,u_Yaw]),axis=0)
    
    INP_SAT = np.min( np.vstack((SUM_INP,np.ones(8) )) , axis=0)
    INP_SAT = np.max( np.vstack((INP_SAT,np.zeros(8))) , axis=0)
    
    return INP_SAT
# %% START ENV
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')

# %% RUN SIM
obs = TestEnv.reset(Z=0)
TestEnv.render()
SaveVec = {}

Reference = np.array([0,0,0,0]) # W,p,q,r
Control_u = np.array([0,0,0,0]) # 
# FORCE INPUT
INP_RPM_p = np.array([[0, 5  , 10 , 11  , 12   , 13 , 30 , 35 , 40 , 50 , 60  ],
                      np.array([1, 1  , 1  , 1   , 1    , 1  , 1  , 1  , 1  , 1  , 1   ])*0.6,
                      [0, 0  , 0  , 0.01, -0.015, 0  , 0  , 0  , 0  , 0  , 0   ]])

WRef = np.array([[0 , 10 , 15 , 20 , 60  ],
                 [0 , 0  , -5  , 0  , 0   ]])

ThetaRef = np.array([[0 , 30 , 35 , 40 , 60  ],
                     [0 , 0  , 5  , 0  , 0   ]])

PhiRef = np.array([[0 , 40 , 45 , 50 , 60  ],
                   [0 , 0  , 5  , 0  , 0   ]])

# %
# Hardcoded best agent: always go left!
n_steps = 1200
W_int = 0

u_Vert=0.7
for step in range(n_steps):
    
    u_Vert,W_int  = PID_Vert(np.interp(step*0.05,WRef[0,:],WRef[1,:]),
                              obs[8],obs[20],W_int,KD=0,KP=0.5, KI=0.05, gamma=0.99)
    
    # u_Vert  += PIDd_Vert(np.interp(step*0.05,WRef[0,:],WRef[1,:]),
    #                          obs[8],obs[20],KP=0.0, KI=0.01)
    
    u_Pitch = PID_Pitch(np.interp(step*0.05,ThetaRef[0,:],np.deg2rad(ThetaRef[1,:])),
                        obs[4],obs[10],KD=0.5,KP=1)
    
    u_Roll  = PID_Roll(np.interp(step*0.05,PhiRef[0,:],np.deg2rad(PhiRef[1,:])),
                        obs[3],obs[9],KD=0.5,KP=1)
    
    RPM_p  = ControlMixer(VerticalControlAllocation(u_Vert),
                          PitchControlAllocation(u_Pitch),
                          RollControlAllocation(u_Roll),
                          YawControlAllocation(0))
    Tilt_p = np.ones(2)
    InputVec = np.hstack((RPM_p,Tilt_p))
    if step >= 350:
        a=1
    
    obs, reward, done, info = TestEnv.step(InputVec)
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

PlotTimeLim = SimTime
fig = plt.figure()

plt_l = 4
plt_c = 2
plt_n = 1

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
plt.plot(TimeVec,SaveVec['Z_m'])
plt.ylabel('Z [m]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
plt.plot(TimeVec,SaveVec['W_mps'])
plt.plot(WRef[0,:],-WRef[1,:],'k--')
plt.ylabel('W [m/s]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
plt.plot(TimeVec,SaveVec['AZ_mps2'])
plt.ylabel('AZ [m/s²]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
# plt.plot(TimeVec,SaveVec['FZ_N'])
# plt.ylabel('FZ [N]')
plt.plot(TimeVec,SaveVec['MY_Nm'])
plt.ylabel('MY [N]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
plt.plot(TimeVec,SaveVec['Theta_deg'])
plt.plot(ThetaRef[0,:],ThetaRef[1,:],'k--')
plt.ylabel('Theta [deg]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
plt.plot(TimeVec,SaveVec['Phi_deg'])
plt.plot(PhiRef[0,:],PhiRef[1,:],'k--')
plt.ylabel('Phi [deg]')
plt.xlabel('Time [s]')

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([0,PlotTimeLim])
# plt.plot(TimeVec,SaveVec['Alpha_deg'],label = 'Alpha_deg')
# plt.plot(TimeVec,SaveVec['W1_Alpha_deg'],label = 'W1_Alpha_deg')
# plt.plot(TimeVec,SaveVec['W2_Alpha_deg'],label = 'W2_Alpha_deg')
# plt.ylabel('Alpha [deg]')
# plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
plt.plot(TimeVec,SaveVec['RPM_1'],label='1')
plt.plot(TimeVec,SaveVec['RPM_4'],label='4')
plt.plot(TimeVec,SaveVec['RPM_5'],label='5')
plt.ylabel('RPM')
plt.legend()
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([0,PlotTimeLim])
plt.plot(TimeVec,SaveVec['Thrust1_N'],label='1')
plt.plot(TimeVec,SaveVec['Thrust4_N'],label='4')
plt.plot(TimeVec,SaveVec['Thrust5_N'],label='5')
plt.ylabel('Thrust [N]')
plt.legend()
plt.xlabel('Time [s]')
fig.set_size_inches(6, 7.5)
fig.tight_layout() 

plt.show()

# fig = plt.figure()
# plt.plot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([0,PlotTimeLim])
# plt.plot(TimeVec,SaveVec['Alpha_deg'],label = 'Alpha_deg')
# plt.plot(TimeVec,SaveVec['W1_Alpha_deg'],label = 'W1_Alpha_deg')
# plt.plot(TimeVec,SaveVec['W2_Alpha_deg'],label = 'W2_Alpha_deg')
# plt.legend()
# plt.ylabel('Alpha [deg]')
# plt.xlabel('Time [s]')

# fig = plt.figure()
# plt.plot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([0,PlotTimeLim])
# plt.plot(TimeVec,SaveVec['W1_CLS'],label = 'W1_CLS')
# plt.plot(TimeVec,SaveVec['W2_CLS'],label = 'W2_CLS')
# plt.legend()
# plt.ylabel('Alpha [deg]')
# plt.xlabel('Time [s]')

# fig = plt.figure()
# plt.plot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([0,PlotTimeLim])
# plt.plot(TimeVec,SaveVec['U_mps'],label = 'U_mps')
# plt.plot(TimeVec,SaveVec['V_mps'],label = 'V_mps')
# plt.legend()
# plt.ylabel('Alpha [deg]')
# plt.xlabel('Time [s]')
