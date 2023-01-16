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
    SaveVec = AppendValue(SaveVec,'Elevon1',info['AERO']['Elevon']['Deflection_deg'][0])
    SaveVec = AppendValue(SaveVec,'Elevon2',info['AERO']['Elevon']['Deflection_deg'][1])
    SaveVec = AppendValue(SaveVec,'Elevon3',info['AERO']['Elevon']['Deflection_deg'][2])
    SaveVec = AppendValue(SaveVec,'Elevon4',info['AERO']['Elevon']['Deflection_deg'][3])

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


# %% START ENV
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')     
# obs = TestEnv.reset(VX_mps = 0, VZ_mps = 0.0, THETA = 0.0, DispMessages = False, 
#                     TermTheta_deg = 45, StaFreezeList = ['X_m', 'Z_m', 'U_mps','W_mps'])
obs = TestEnv.reset(VX_mps = 0, VZ_mps = 0.0, THETA = 0.0, DispMessages = False, 
                    TermTheta_deg = 45, StaFreezeList = [])

# %% PARAMS
SimTime = 30

print(" ")
print("Trimmed: " + str(TestEnv.TrimData['Trimmed']))
print("Trimmed Action: " + str(TestEnv.TrimData['Action']))

SaveVec = {}
# %% OUTSIDE INPUTS
Inp = {}
PulseSize = 0.5
Inp['Pitch_u']= np.array([[0 , 5.0 , 5.1 , 5.+PulseSize , 5.1+PulseSize , SimTime  ],
                          [0 ,  0  ,  1  ,  1           ,  1            ,  1 ]])
Inp['Pitch_u'][1,:] = 0*1e-1 * Inp['Pitch_u'][1,:]

# %% REFERENCES
Ref = {}
Ref['VZ_mps'] = np.array([[0 , SimTime ],
                          [0 ,  0      ]])
Ref['VX_mps'] = np.array([[0 , 0 , 5, SimTime ],
                          [0 , 0 , 60 , 60    ]])

Ref['Theta_deg'] = np.array([[0 , 5, 5.01, 1000  ],
                             [0 , 0, 0   , 0     ]])


# %% GAINS
Gains = {}
KKK = 4.0
Gains['EAS_mps']     = np.array([0       , 5       , 10      , 20      , 30      , 35      , 40      , 50      , 60     ])
Gains['Pitch2ThrP']  = np.array([0.06735 , 0.06642 , 0.06417 , 0.06304 , 0.07338 , 0.04137 , 0.00000 , 0.00000 , 0.00000])
Gains['Pitch2W2E']   = np.array([0.00000 , 0.00000 , 0.00000 , 0.00000 , 0.00000 ,-0.51702 ,-0.49712 ,-0.22094 ,-0.15343])
# Gains['q2Pitch']     = 2*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])
Gains['q2Pitch']     = KKK*np.array([2.00000 , 2.06509 , 2.25284 , 2.38227 , 2.38823 , 2.14081 , 1.97851 , 1.73990 , 1.53331])
Gains['T2Pitch']     = 0*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])
# Gains['i2Pitch']     = 2.0*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])
Gains['i2Pitch']     = KKK*np.array([1.00    , 1.05      ,  1.10   ,  1.20    ,  1.20   ,  1.10    ,  1.00   ,  0.90     ,  0.80    ])
Gains['Theta2q']     = KKK*0.5*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])

Gains['Target_VX_mps'] = Ref['VX_mps'] 
Gains['Tilt_Trim_deg'] = np.array([[0    , 5     , 10   , 20   , 30   , 35   , 40   , 50   , 60],
                                   [90.0 ,  88.3 , 83.2 , 64.4 , 42.6 , 34.1 , 27.6 , 18.8 , 13.4]])
Gains['Throttle_Trim_deg'] = np.array([[0      , 5      , 10     , 20     , 30     , 35     , 40     , 50     , 60],
                                       [-0.078 , -0.079 , -0.079 , -0.095 , -0.194 , -0.247 , -0.284 , -0.321 , -0.307]])

# %
TimeVec = np.arange(0,SimTime,TestEnv.t_step)
if abs(TimeVec[-1] - SimTime) > (TestEnv.t_step/2): TimeVec = np.append(TimeVec,SimTime)
n_steps = np.size(TimeVec)

VZ_int  = 0
The_int = 0

u_Vert  = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')]
u_Pitch = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')]
u_Tilt1 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W1_Tilt')]
u_Tilt2 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Tilt')]
u_Elev2 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Elevator')]


Return = 0

def TiltController(Gains,t):
    VX_tgt_mps = np.interp(t,Gains['Target_VX_mps'][0,:],Gains['Target_VX_mps'][1,:])
    TiltCmd_deg = np.interp(VX_tgt_mps,Gains['Tilt_Trim_deg'][0,:],Gains['Tilt_Trim_deg'][1,:])
    return TiltCmd_deg,TiltCmd_deg

def ThrottleController(Gains,t):
    VX_tgt_mps = np.interp(t,Gains['Target_VX_mps'][0,:],Gains['Target_VX_mps'][1,:])
    ThrottleCmd_u = np.interp(VX_tgt_mps,Gains['Throttle_Trim_deg'][0,:],Gains['Throttle_Trim_deg'][1,:])
    return ThrottleCmd_u

class GenPitchController:

    def __init__(self,Gains):
        self.Gains = Gains.copy()
        self.int_q = 0
        self.int_T = 0
        self.Last_t = 0

    def CalcCmd (self,Ref,OutPitchCmd_u , info,t):
        # Calculate Gains
        Pitch2ThrP = np.interp(info['ATM']['EAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Pitch2ThrP'])
        Pitch2W2E  = np.interp(info['ATM']['EAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Pitch2W2E'])
        q2Pitch    = np.interp(info['ATM']['EAS_mps'] , self.Gains['EAS_mps'] , self.Gains['q2Pitch'])
        i2Pitch    = np.interp(info['ATM']['EAS_mps'] , self.Gains['EAS_mps'] , self.Gains['i2Pitch'])
        T2Pitch    = np.interp(info['ATM']['EAS_mps'] , self.Gains['EAS_mps'] , self.Gains['T2Pitch'])
        Theta2q    = np.interp(info['ATM']['EAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Theta2q'])

        # Calculate References and Errors
        Theta_ref_deg = np.interp(t,Ref['Theta_deg'][0,:],Ref['Theta_deg'][1,:])
        Theta_error_rad = np.deg2rad(Theta_ref_deg) - info['EQM']['EulerAngles_rad'][1]
        # print("Theta_ref_rad: {Theta_ref_rad:0.3f}" 
        #       ", Theta_rad: {Theta_rad:0.3f}"
        #       .format(Theta_ref_rad = np.deg2rad(Theta_ref_deg), Theta_rad=info['EQM']['EulerAngles_rad'][1]))

        q_ref_radps   = Theta_error_rad*Theta2q
        q_error_radps   = q_ref_radps - info['EQM']['VelRot_BodyAx_radps'][1]

        self.int_q += q_error_radps * (t-self.Last_t)
        self.int_T += Theta_error_rad * (t-self.Last_t)
        self.Last_t = t

        # Calculate Pitch Cmd
        # print("t: " + str(t) + " | int: " + str())
        PitchCmd_u = q_error_radps * q2Pitch + self.int_T*i2Pitch + Theta_error_rad*T2Pitch + OutPitchCmd_u
        # print("Theta_error_rad: {Theta_error_rad:0.3f}" 
        #       ", q_ref_radps: {q_ref_radps:0.3f}" 
        #       ", q_error_radps: {q_error_radps:0.3f}"
        #       ", PitchCmd_u: {PitchCmd_u:0.3f}"
        #       .format(Theta_error_rad = Theta_error_rad, q_ref_radps=q_ref_radps, q_error_radps=q_error_radps, PitchCmd_u=PitchCmd_u))
        
        # Calculate Cmd Allocation
        PitchThrottle_u = PitchCmd_u * Pitch2ThrP
        W2_ElevCmd_u    = PitchCmd_u * Pitch2W2E

        return PitchThrottle_u , W2_ElevCmd_u


adm_vec = TestEnv.adm_vec


PitchController = GenPitchController(Gains)

for step in range(n_steps):
    CurrentTime_s = step*TestEnv.t_step
    
    InputVec = np.array([u_Vert[step] ,
                        u_Pitch[step],
                        u_Tilt1[step],
                        u_Tilt2[step],
                        u_Elev2[step]])
    
    obs, reward, done, info = TestEnv.step(InputVec)
    
    obs = obs*adm_vec
    Return = Return + reward
    SaveVec = SaveSelection(SaveVec,TestEnv.info)
    # if done:
    #     print("Goal reached!", "reward=", reward)
    #     break


    OutPitchCmd_u =  np.interp(CurrentTime_s,Inp['Pitch_u'][0,:],Inp['Pitch_u'][1,:])

    W1_TiltCmd_deg , W2_TiltCmd_deg  = TiltController(Gains,CurrentTime_s)
    PitchThrottle_u , W2_ElevCmd_u = PitchController.CalcCmd(Ref, OutPitchCmd_u, info, CurrentTime_s)
    
    u_Vert [step+1]  = ThrottleController(Gains,CurrentTime_s)
    
    u_Tilt1 [step+1] = 2 * (W1_TiltCmd_deg - TestEnv.CONT['MinTilt_deg'][0]) / TestEnv.CONT['TiltRange_deg'][0] - 1
    u_Tilt2 [step+1] = 2 * (W2_TiltCmd_deg - TestEnv.CONT['MinTilt_deg'][1]) / TestEnv.CONT['TiltRange_deg'][1] - 1

    u_Pitch [step+1] = u_Pitch [step+1] + PitchThrottle_u
    u_Elev2 [step+1] = u_Elev2 [step+1] + W2_ElevCmd_u

    # u_Tilt1 [step+1] = u_Tilt1 [step+1] + np.interp(CurrentTime_s , Tilt_Inp[0,:],Tilt_Inp[1,:])       
    # u_Tilt2 [step+1] = u_Tilt2 [step+1] + np.interp(CurrentTime_s , Tilt_Inp[0,:],Tilt_Inp[1,:])       

        # self.AERO['Wing2']['Incidence_deg'] = (TestEnv.CONT['MinTilt_deg'][1] 
        # #                                      + TestEnv.CONT['TiltRange_deg'][1] * self.CONT['Tilt_p'][1])
        # self.AERO['Elevon']['Deflection_deg'] = (self.CONT['ElevCenter_deg']
        #                                        + self.CONT['ElevRange_deg']/2 * self.CONT['Elevon_p'])

# % CALL PLOT FILE
print('Return = {:0.1f}'.format(Return))
exec(open("./Test_VTOL_PIDPlot2.py").read())

