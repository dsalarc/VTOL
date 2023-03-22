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
    
def SaveSelection(SaveVec,info):
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

    SaveVec = AppendValue(SaveVec,'W1_Alpha_deg',info['AERO']['Wing1']['Alpha_deg'])
    SaveVec = AppendValue(SaveVec,'W2_Alpha_deg',info['AERO']['Wing2']['Alpha_deg'])
    SaveVec = AppendValue(SaveVec,'W1_Incidence_deg',info['AERO']['Wing1']['Incidence_deg'])
    SaveVec = AppendValue(SaveVec,'W2_Incidence_deg',info['AERO']['Wing2']['Incidence_deg'])
    SaveVec = AppendValue(SaveVec,'W1_CLS',info['AERO']['Wing1']['CLS_25Local'])
    SaveVec = AppendValue(SaveVec,'W2_CLS',info['AERO']['Wing2']['CLS_25Local'])
    SaveVec = AppendValue(SaveVec,'W1_CDS',info['AERO']['Wing1']['CDS_25Local'])
    SaveVec = AppendValue(SaveVec,'W2_CDS',info['AERO']['Wing2']['CDS_25Local'])
    SaveVec = AppendValue(SaveVec,'Elevon1',info['CONT']['Elevon_deg'][0])
    SaveVec = AppendValue(SaveVec,'Elevon2',info['CONT']['Elevon_deg'][1])
    SaveVec = AppendValue(SaveVec,'Elevon3',info['CONT']['Elevon_deg'][2])
    SaveVec = AppendValue(SaveVec,'Elevon4',info['CONT']['Elevon_deg'][3])

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
    SaveVec = AppendValue(SaveVec,'J1',info['MOT']['ASSEMBLY']['obj'][0].PROPELLER.J)

    SaveVec = AppendValue(SaveVec,'Weight_kgf',info['MASS']['Weight_kgf'])
    SaveVec = AppendValue(SaveVec,'XCG_m',info['MASS']['CG_m'][0])
    SaveVec = AppendValue(SaveVec,'YCG_m',info['MASS']['CG_m'][0])
    SaveVec = AppendValue(SaveVec,'ZCG_m',info['MASS']['CG_m'][0])
    
    SaveVec = AppendValue(SaveVec,'Reward',reward)

    return SaveVec


# %% START ENV
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')

obs = TestEnv.reset(VX_mps = 0, VZ_mps = 0.0, THETA = 0.0, DispMessages = False, TermTheta_deg = 45)

# %% PARAMS
SimTime = 5

print(" ")
print("Trimmed: " + str(TestEnv.TrimData['Trimmed']))
print("Trimmed Action: " + str(TestEnv.TrimData['Action']))

SaveVec = {}



# %%
TimeVec = np.arange(0,SimTime,TestEnv.t_step)
if abs(TimeVec[-1] - SimTime) > (TestEnv.t_step/2): TimeVec = np.append(TimeVec,SimTime)
n_steps = np.size(TimeVec)

u_Vert  = np.ones(n_steps)*TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')]
u_Pitch = np.ones(n_steps)*TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')]
u_Tilt1 = np.ones(n_steps)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W1_Tilt')]
u_Tilt2 = np.ones(n_steps)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Tilt')]
u_Elev2 = np.ones(n_steps)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Elevator')]


#ELEV
pulse_size = 0
pulse_ini  = 2
pulse_time = 2
pulse_n    = 5

#WING
pulse_size = 0
pulse_ini  = 2
pulse_time = 2
pulse_n    = 3

u_step = np.zeros(n_steps)
u_step[TimeVec >= pulse_ini]  = pulse_size
u_step[TimeVec >= pulse_ini+pulse_time]  = 0

if pulse_n == 1:
    u_Vert = u_Vert + u_step
elif pulse_n == 2:
    u_Pitch = u_Pitch + u_step
elif pulse_n == 3:
    u_Tilt1 = u_Tilt1 + 2*(u_step - TestEnv.CONT['MinTilt_deg'][0]) / (TestEnv.CONT['TiltRange_deg'][0])
    u_Tilt2 = u_Tilt2 + 2*(u_step - TestEnv.CONT['MinTilt_deg'][1]) / (TestEnv.CONT['TiltRange_deg'][1])
elif pulse_n == 5:
    u_Elev2 = u_Elev2 + u_step / (TestEnv.CONT['ElevRange_deg'][2]/2)
    

Return = 0
for step in range(n_steps):
    
    CurrentTime_s = step*TestEnv.t_step
    
    InputVec = np.array([u_Vert[step] ,
                        u_Pitch[step],
                        u_Tilt1[step],
                        u_Tilt2[step],
                        u_Elev2[step]])
    
    obs, reward, done, info = TestEnv.step(InputVec)
    
    obs = obs*TestEnv.adm_vec
    Return = Return + reward
    SaveVec = SaveSelection(SaveVec,TestEnv.info)
    if done:
        print("Goal reached!", "reward=", reward)
        break


# % CALL PLOT FILE
print('Return = {:0.1f}'.format(Return))
exec(open("./Test_VTOL_CheckTrim_plot.py").read())

