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
    SaveVec = AppendValue(SaveVec,'Z_m',info['EQM']['PosLin_EarthAx_m'][2])
    SaveVec = AppendValue(SaveVec,'H_m',-info['EQM']['PosLin_EarthAx_m'][2])

    SaveVec = AppendValue(SaveVec,'U_mps',info['EQM']['VelLin_BodyAx_mps'][0])
    SaveVec = AppendValue(SaveVec,'V_mps',info['EQM']['VelLin_BodyAx_mps'][1])
    SaveVec = AppendValue(SaveVec,'W_mps',info['EQM']['VelLin_BodyAx_mps'][2])
    
    SaveVec = AppendValue(SaveVec,'dU_mps2',info['EQM']['AccLin_BodyAx_mps2'][0])
    SaveVec = AppendValue(SaveVec,'dV_mps2',info['EQM']['AccLin_BodyAx_mps2'][1])
    SaveVec = AppendValue(SaveVec,'dW_mps2',info['EQM']['AccLin_BodyAx_mps2'][2])
    
    SaveVec = AppendValue(SaveVec,'VX_mps',info['EQM']['VelLin_EarthAx_mps'][0])
    SaveVec = AppendValue(SaveVec,'VY_mps',info['EQM']['VelLin_EarthAx_mps'][1])
    SaveVec = AppendValue(SaveVec,'VZ_mps',info['EQM']['VelLin_EarthAx_mps'][2])
    
    SaveVec = AppendValue(SaveVec,'dVX_mps2',info['EQM']['AccLin_EarthAx_mps2'][0])
    SaveVec = AppendValue(SaveVec,'dVY_mps2',info['EQM']['AccLin_EarthAx_mps2'][1])
    SaveVec = AppendValue(SaveVec,'dVZ_mps2',info['EQM']['AccLin_EarthAx_mps2'][2])
    
    SaveVec = AppendValue(SaveVec,'NX_g',info['EQM']['LoadFactor_g'][0])
    SaveVec = AppendValue(SaveVec,'NY_g',info['EQM']['LoadFactor_g'][1])
    SaveVec = AppendValue(SaveVec,'NZ_g',info['EQM']['LoadFactor_g'][2])
    
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
    SaveVec = AppendValue(SaveVec,'FZ_N',info['EQM']['TotalForce'][2])
 
    SaveVec = AppendValue(SaveVec,'MX_Nm',info['EQM']['TotalMoment'][0])
    SaveVec = AppendValue(SaveVec,'MY_Nm',info['EQM']['TotalMoment'][1])
    SaveVec = AppendValue(SaveVec,'MZ_Nm',info['EQM']['TotalMoment'][2])
 
    SaveVec = AppendValue(SaveVec,'MXaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][0])
    SaveVec = AppendValue(SaveVec,'MYaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][1])
    SaveVec = AppendValue(SaveVec,'MZaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][2])
 
    SaveVec = AppendValue(SaveVec,'FXaero_N',info['AERO']['TotalForce_BodyAx_N'][0])
    SaveVec = AppendValue(SaveVec,'FYaero_N',info['AERO']['TotalForce_BodyAx_N'][1])
    SaveVec = AppendValue(SaveVec,'FZaero_N',info['AERO']['TotalForce_BodyAx_N'][2])
    
    SaveVec = AppendValue(SaveVec,'Alpha_deg',info['ATM']['Alpha_deg'])
    SaveVec = AppendValue(SaveVec,'Beta_deg',info['ATM']['Beta_deg'])
    SaveVec = AppendValue(SaveVec,'DynPres_Pa',info['ATM']['DynPres_Pa'])

    try:
        SaveVec = AppendValue(SaveVec,'W1_Alpha_deg',info['AERO']['Wing1']['Alpha_deg'])
        SaveVec = AppendValue(SaveVec,'W2_Alpha_deg',info['AERO']['Wing2']['Alpha_deg'])
        SaveVec = AppendValue(SaveVec,'W1_Incidence_deg',info['AERO']['Wing1']['Incidence_deg'])
        SaveVec = AppendValue(SaveVec,'W2_Incidence_deg',info['AERO']['Wing2']['Incidence_deg'])
        SaveVec = AppendValue(SaveVec,'W1_CLS',info['AERO']['Wing1']['CLS_25Local'])
        SaveVec = AppendValue(SaveVec,'W2_CLS',info['AERO']['Wing2']['CLS_25Local'])
        SaveVec = AppendValue(SaveVec,'W1_CDS',info['AERO']['Wing1']['CDS_25Local'])
        SaveVec = AppendValue(SaveVec,'W2_CDS',info['AERO']['Wing2']['CDS_25Local'])
    except:

        SaveVec = AppendValue(SaveVec,'W1_Alpha_deg',info['AERO']['Wing']['Alpha_deg'][0])
        SaveVec = AppendValue(SaveVec,'W2_Alpha_deg',info['AERO']['Wing']['Alpha_deg'][1])
        SaveVec = AppendValue(SaveVec,'W1_Incidence_deg',info['AERO']['Wing']['Incidence_deg'][0])
        SaveVec = AppendValue(SaveVec,'W2_Incidence_deg',info['AERO']['Wing']['Incidence_deg'][1])
        SaveVec = AppendValue(SaveVec,'W1_CLS',info['AERO']['Wing']['CLS_25Local'][0])
        SaveVec = AppendValue(SaveVec,'W2_CLS',info['AERO']['Wing']['CLS_25Local'][1])
        SaveVec = AppendValue(SaveVec,'W1_CDS',info['AERO']['Wing']['CDS_25Local'][0])
        SaveVec = AppendValue(SaveVec,'W2_CDS',info['AERO']['Wing']['CDS_25Local'][1])

    SaveVec = AppendValue(SaveVec,'RPM_1',info['MOT']['RPM'][0])
    SaveVec = AppendValue(SaveVec,'RPM_2',info['MOT']['RPM'][1])
    SaveVec = AppendValue(SaveVec,'RPM_4',info['MOT']['RPM'][3])
    SaveVec = AppendValue(SaveVec,'RPM_5',info['MOT']['RPM'][4])
    SaveVec = AppendValue(SaveVec,'RPM_8',info['MOT']['RPM'][7])

    SaveVec = AppendValue(SaveVec,'Thrust1_N',info['MOT']['Thrust_N'][0])
    SaveVec = AppendValue(SaveVec,'Thrust2_N',info['MOT']['Thrust_N'][1])
    SaveVec = AppendValue(SaveVec,'Thrust3_N',info['MOT']['Thrust_N'][2])
    SaveVec = AppendValue(SaveVec,'Thrust4_N',info['MOT']['Thrust_N'][3])
    SaveVec = AppendValue(SaveVec,'Thrust5_N',info['MOT']['Thrust_N'][4])
    SaveVec = AppendValue(SaveVec,'Thrust6_N',info['MOT']['Thrust_N'][5])
    SaveVec = AppendValue(SaveVec,'Thrust7_N',info['MOT']['Thrust_N'][6])
    SaveVec = AppendValue(SaveVec,'Thrust8_N',info['MOT']['Thrust_N'][7])
    SaveVec = AppendValue(SaveVec,'J1',info['MOT']['J'][0])

    SaveVec = AppendValue(SaveVec,'Weight_kgf',info['MASS']['Weight_kgf'])
    SaveVec = AppendValue(SaveVec,'XCG_m',info['MASS']['CG_m'][0])
    SaveVec = AppendValue(SaveVec,'YCG_m',info['MASS']['CG_m'][0])
    SaveVec = AppendValue(SaveVec,'ZCG_m',info['MASS']['CG_m'][0])
    
    SaveVec = AppendValue(SaveVec,'Reward',reward)

    return SaveVec

def PID_eVTOL(Reference,obs,Last_u):
    # VERTICAL CONTROL
    

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

def PID_Vert(W_Ref_mps,W_mps,AZ_mps2, Wint_m, KP=0, KI=0, KD=0, KFF=0, gamma = 0.99):
    Wint_m = Wint_m*gamma + (W_Ref_mps-W_mps)*0.05
    u = -(KP*(W_Ref_mps-W_mps) - KD*AZ_mps2 + KI*Wint_m + KFF*W_Ref_mps)
    return u,Wint_m

def PIDd_Vert(W_Ref_mps,W_mps,AZ_mps2, KI=0, KP=0, gamma = 0.99):
    du = -(KI*(W_Ref_mps-W_mps) - KP*AZ_mps2)
    return du

def PID_Pitch(The_Ref_rad,Theta_rad,Q_radps,The_int_rad,KP=0, KI=0, KD=0, gamma = 0.99):
    The_int_rad = The_int_rad*gamma + (Theta_rad - The_Ref_rad)*0.05
    u = -(KP*(Theta_rad - The_Ref_rad) + KD*Q_radps + KI*The_int_rad)
    return u, The_int_rad

def PID_Roll(Phi_Ref_rad,Phi_rad,P_radps,Phi_int_rad,KP=0, KI=0, KD=0, gamma = 0.99):
    Phi_int_rad = Phi_int_rad*gamma + (Phi_rad - Phi_Ref_rad)*0.05
    u = -(KP*(Phi_rad-Phi_Ref_rad) + KD*P_radps + KI*Phi_int_rad)
    return u, Phi_int_rad

def PID_Yaw(Psi_Ref_rad,Psi_rad,R_radps,Psi_int_rad,KP=0, KI=0, KD=0, gamma = 0.99):
    Psi_int_rad = Psi_int_rad*gamma + (Psi_rad - Psi_Ref_rad)*0.05
    u = -(KP*(Psi_rad-Psi_Ref_rad) + KD*R_radps + KI*Psi_int_rad)
    return u, Psi_int_rad

# %% START ENV
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')
        

# %% RUN SIM
# obs,TrimAction = TestEnv.reset(Z=0,W=0,THETA=np.deg2rad(0), PHI=np.deg2rad(0), PSI=np.deg2rad(0), PaxIn = np.array([1,1]))


obs = TestEnv.reset(VX_mps = 20, VZ_mps = 0.0, THETA = 0.0)

print(" ")
print("Trimmed: " + str(TestEnv.TrimData['Trimmed']))
print("Trimmed Action: " + str(TestEnv.TrimData['Action']))

# if 1==0:
# TestEnv.render()
SaveVec = {}


VZ_Ref = np.array([[0 , 5 , 100 , 1000  ],
                [0 , 0  , 0 ,  0    ]])


The_Ref = np.array([[0 , 1000  ],
                    [0 , 0     ]])

Phi_Ref = np.array([[0 , 1000  ],
                    [0 , 0     ]])

Psi_Ref = np.array([[0 , 1000  ],
                    [0 , 0     ]])

Tilt_Inp = np.array([[0 , 5 , 20 ],
                     [0 , 0 , 0  ]])

# %
SimTime = 15
TimeVec = np.arange(0,SimTime,TestEnv.t_step)
if abs(TimeVec[-1] - SimTime) > (TestEnv.t_step/2): TimeVec = np.append(TimeVec,SimTime)
n_steps = np.size(TimeVec)

VZ_int  = 0
The_int = 0
Phi_int = 0
Psi_int = 0

u_Vert  = np.ones(n_steps+1)*TestEnv.TrimData['Action'][0]
u_Pitch = np.ones(n_steps+1)*TestEnv.TrimData['Action'][1]
u_Roll  = np.ones(n_steps+1)*TestEnv.TrimData['Action'][2]
u_Yaw   = np.ones(n_steps+1)*TestEnv.TrimData['Action'][3]
u_Tilt1 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][4]
u_Tilt2 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][5]
u_Elev1 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][6]
u_Elev2 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][7]
u_Ail1  = np.ones(n_steps+1)*TestEnv.TrimData['Action'][8]
u_Ail2  = np.ones(n_steps+1)*TestEnv.TrimData['Action'][9]


Return = 0

adm_vec = TestEnv.adm_vec
for step in range(n_steps):
    CurrentTime_s = step*TestEnv.t_step
    
    InputVec = np.array([u_Vert[step] ,
                        u_Pitch[step],
                        u_Roll[step] ,
                        u_Yaw[step]  ,
                        u_Tilt1[step],
                        u_Tilt2[step],
                        u_Elev1[step],
                        u_Elev2[step],
                        u_Ail1[step] ,
                        u_Ail2[step] ])
    
    obs, reward, done, info = TestEnv.step(InputVec)
    
    obs = obs*adm_vec
    Return = Return + reward
    SaveVec = SaveSelection(step,SaveVec,info)
    if done:
        print("Goal reached!", "reward=", reward)
        break

    # u_Vert[step+1],VZ_int = PID_Vert(np.interp(CurrentTime_s,VZ_Ref[0,:] , VZ_Ref[1,:]),
    #                           obs[4],obs[5],VZ_int,
    #                           KD=0.0,KP=1.0, KI=0.1, KFF=0.0, gamma=1)

    # u_Pitch[step+1],The_int = PID_Pitch(np.interp(CurrentTime_s,The_Ref[0,:],np.deg2rad(The_Ref[1,:])),
    #                           obs[8],obs[9],The_int,
    #                           KD=0.5,KP=1.0,KI=0, gamma=1)


    u_Tilt1 [step+1] = u_Tilt1 [step+1] + np.interp(CurrentTime_s , Tilt_Inp[0,:],Tilt_Inp[1,:])       
    u_Tilt2 [step+1] = u_Tilt2 [step+1] + np.interp(CurrentTime_s , Tilt_Inp[0,:],Tilt_Inp[1,:])       


# % CALL PLOT FILE
print('Return = {:0.1f}'.format(Return))
exec(open("./Test_VTOL_GeneralPlot.py").read())

