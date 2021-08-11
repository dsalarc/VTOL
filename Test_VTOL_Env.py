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
    
    SaveVec = AppendValue(SaveVec,'P_degps',np.rad2deg(info['EQM']['VelRot_BodyAx_radps'][0]))
    SaveVec = AppendValue(SaveVec,'Q_degps',np.rad2deg(info['EQM']['VelRot_BodyAx_radps'][1]))
    SaveVec = AppendValue(SaveVec,'R_degps',np.rad2deg(info['EQM']['VelRot_BodyAx_radps'][2]))
    
    SaveVec = AppendValue(SaveVec,'Pdot_radps2',np.rad2deg(info['EQM']['AccRot_BodyAx_radps2'][0]))
    SaveVec = AppendValue(SaveVec,'Qdot_radps2',np.rad2deg(info['EQM']['AccRot_BodyAx_radps2'][1]))
    SaveVec = AppendValue(SaveVec,'Rdot_radps2',np.rad2deg(info['EQM']['AccRot_BodyAx_radps2'][2]))
 
    SaveVec = AppendValue(SaveVec,'FX_N',info['EQM']['TotalForce'][0])
    SaveVec = AppendValue(SaveVec,'FY_N',info['EQM']['TotalForce'][1])
    SaveVec = AppendValue(SaveVec,'FZ_N',-info['EQM']['TotalForce'][2])
 
    SaveVec = AppendValue(SaveVec,'MX_Nm',info['EQM']['TotalMoment'][0])
    SaveVec = AppendValue(SaveVec,'MY_Nm',info['EQM']['TotalMoment'][1])
    SaveVec = AppendValue(SaveVec,'MZ_Nm',-info['EQM']['TotalMoment'][2])
 
    SaveVec = AppendValue(SaveVec,'MXaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][0])
    SaveVec = AppendValue(SaveVec,'MYaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][1])
    SaveVec = AppendValue(SaveVec,'MZaero_Nm',-info['AERO']['TotalMoment_BodyAx_Nm'][2])
 
    SaveVec = AppendValue(SaveVec,'FXaero_N',info['AERO']['TotalForce_BodyAx_N'][0])
    SaveVec = AppendValue(SaveVec,'FYaero_N',info['AERO']['TotalForce_BodyAx_N'][1])
    SaveVec = AppendValue(SaveVec,'FZaero_N',info['AERO']['TotalForce_BodyAx_N'][2])
 
    SaveVec = AppendValue(SaveVec,'Alpha_deg',info['ATM']['Alpha_deg'])
    SaveVec = AppendValue(SaveVec,'Beta_deg',info['ATM']['Beta_deg'])
    SaveVec = AppendValue(SaveVec,'DynPres_Pa',info['ATM']['DynPres_Pa'])

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
    u = -(KP*(WRef_mps-W_mps) - KD*AZ_mps2 + KI*Wint_m)
    return u,Wint_m

def PIDd_Vert(WRef_mps,W_mps,AZ_mps2, KI=0, KP=0, gamma = 0.99):
    du = -(KI*(WRef_mps-W_mps) - KP*AZ_mps2)
    return du

def PID_Pitch(ThetaRef_rad,Theta_rad,Q_radps,KD=0,KP=0):
    u = -(KP*(Theta_rad-ThetaRef_rad) + KD*Q_radps)
    return u

def PID_Roll(PhiRef_rad,Phi_rad,P_radps,KD=0,KP=0):
    u = -(KP*(Phi_rad-PhiRef_rad) + KD*P_radps)
    return u

# %% START ENV
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')

# %% RUN SIM
obs = TestEnv.reset(Z=0)
# TestEnv.render()
SaveVec = {}

Reference = np.array([0,0,0,0]) # W,p,q,r
Control_u = np.array([0,0,0,0]) # 
# FORCE INPUT
INP_RPM_p = np.array([[0, 5  , 10 , 11  , 12   , 13 , 30 , 35 , 40 , 50 , 1000  ],
                      np.array([1, 1  , 1  , 1   , 1    , 1  , 1  , 1  , 1  , 1  , 1   ])*0.6,
                      [0, 0  , 0  , 0.01, -0.015, 0  , 0  , 0  , 0  , 0  , 0   ]])

WRef = np.array([[0 , 10 , 15 , 20 , 1000  ],
                 [0 , 0  , -5  , 0  , 0   ]])

ThetaRef = np.array([[0 , 30 , 35 , 40 , 1000  ],
                     [0 , 0  , -5  , -0.0  , -0.0   ]])

PhiRef = np.array([[0 , 40 , 45 , 50 , 1000  ],
                   [0 , 0  , -5  , 0  , 0   ]])

# %
# Hardcoded best agent: always go left!
n_steps = int(70/TestEnv.t_step)
W_int = 0

u_Vert    = np.ones(n_steps+1)*0.0
u_Pitch   = np.zeros(n_steps+1)
u_Roll    = np.zeros(n_steps+1)

for step in range(n_steps):
    
            
    Tilt_p = np.ones(2)
    InputVec = np.hstack((u_Vert[step],u_Pitch[step],u_Roll[step],0,Tilt_p))
    
    obs, reward, done, info = TestEnv.step(InputVec)
    SaveVec = SaveSelection(step,SaveVec,info)
    if done:
      print("Goal reached!", "reward=", reward)
      break
  
    # u_Vert[step+1],W_int  = PID_Vert(np.interp(step*0.05,WRef[0,:],WRef[1,:]),
    #                           obs[8],obs[20],W_int,KD=0,KP=0.5, KI=0.05, gamma=1)
  
    u_Vert[step+1]  = u_Vert[step] + PIDd_Vert(np.interp(step*TestEnv.t_step,WRef[0,:],WRef[1,:]),
                                                obs[8],obs[20],KP=0.01, KI=0.05)
  
    u_Pitch[step+1] = PID_Pitch(np.interp(step*TestEnv.t_step,ThetaRef[0,:],np.deg2rad(ThetaRef[1,:])),
                              obs[4],obs[10],KD=0.5,KP=1)
  
    u_Roll[step+1]  = PID_Roll(np.interp(step*TestEnv.t_step,PhiRef[0,:],np.deg2rad(PhiRef[1,:])),
                              obs[3],obs[9],KD=0.5,KP=1)



# %% CALL PLOT FILE

exec(open("./Test_VTOL_GeneralPlot.py").read())
