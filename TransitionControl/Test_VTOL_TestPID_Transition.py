#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 18:29:29 2021

@author: dsalarc
"""

import gym
import numpy as np
import matplotlib.pyplot as plt
from AuxFunctions import SaveSelection

# %% START ENV
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')     
reset_INPUT_VEC = {}
reset_INPUT_VEC['FAILURE_MOT_5'] = np.array([[0, 20, 20.01, 40] , [0, 0, 0, 0]])
reset_INPUT_VEC['WIND_TowerX_mps'] = np.array([[0, 20, 40] , [0, 0, 0]])
reset_INPUT_VEC['WIND_TowerY_mps'] = np.array([[0, 20, 40] , [0, 0, 0]])
reset_INPUT_VEC['WIND_TurbON'] = np.array([[0, 40] , [0, 0]])
obs = TestEnv.reset(VX_mps = 0, VZ_mps = 0.0, THETA = 0.0, DispMessages = False, 
                    TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0,
                    reset_INPUT_VEC = reset_INPUT_VEC)

# %% PARAMS
SimTime = 40

print(" ")
print("Trimmed: " + str(TestEnv.TrimData['Trimmed']))
print("Trimmed Action: " + str(TestEnv.TrimData['Action']))

SaveVec = {}
# %% OUTSIDE INPUTS
Inp = {}
PulseSize = 0.5*0
Inp['Pitch_u']= np.array([[0 , 5.0 , 5.1 , 5.+PulseSize , 5.1+PulseSize , SimTime  ],
                          [0 ,  0  ,  1  ,  1           ,  1            ,  1 ]])
Inp['Pitch_u'][1,:] = 0*1e-1 * Inp['Pitch_u'][1,:]

# %% REFERENCES
Ref = {}
Ref['VZ_mps'] = np.array([[0 , SimTime ],
                          [0 ,  0      ]])
Ref['VX_mps'] = np.array([[0 , 5 , 45, SimTime ],
                          [0 , 0 , 60 , 60    ]])
Ref['Z_m'] = np.array([[0    , 5 , 45, SimTime ],
                       [-100 , -100 , -100 , -100    ]])

Ref['Theta_deg'] = np.array([[0 , 5, 5.01, 1000  ],
                             [0 , 0, 0   , 0     ]])


# %% PID GAINS
Gains = {}
KKK = 1.0
KK2 = 1.0
Gains['EAS_mps']     = np.array([0       , 5       , 10      , 20      , 30      , 35      , 40      , 50      , 60     ])
Gains['Pitch2ThrP']  = np.array([0.06256, 0.06175, 0.06004, 0.05993, 0.06987, 0.0787 , 0.06039,   0.     , 0.     ])
Gains['Pitch2W2E']   = np.array([0.      , 0.       ,0.      , 0.      ,-0.      ,-0.25851 ,-0.24856 ,-0.22094,-0.15343])
Gains['q2Pitch']     = np.array([4.00000 , 4.06509 , 4.25284 , 4.38227 , 4.38823 , 4.14081 , 3.97851 , 3.73990 , 3.53331])
Gains['T2Pitch']     = 0*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])
Gains['i2Pitch']     = 2*np.array([1.00    , 1.05      ,  1.10   ,  1.20    ,  1.20   ,  1.10    ,  1.00   ,  0.90     ,  0.80    ])
Gains['Theta2q']     = np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])

k2=2
Gains['Alt2VZ']     = 0.5/k2*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])
Gains['Alt2NZ']     = 0*0.5*0.2*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])
Gains['VZ2NZ']      = 0.2*k2*np.array([1       , 1       ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     ])
Gains['Az2Thr']     = 0.5*np.array([-0.09398 , -0.09402 , -0.09474 , -0.10837 , -0.15627 , -0.19797 , -0.25114 , -0.39827 , -0.60136])


# %% FEED FORWARD
Gains['Target_VX_mps'] = Ref['VX_mps'] 
Gains['Tilt_Trim_deg'] = np.array([[0   ,  5  , 10  , 20    , 30    , 35    , 40    , 42, 45, 50, 55, 58, 60, 62, 65],
                                   [90. , 88.3, 83.3, 64.9, 43.5, 35. , 28.4, 25. ,  9.8,  8.0 ,  6.6, 5.9,  5.5,  5.1,  4.7]])
Gains['Throttle_Trim_u'] = np.array([[0   ,  5  , 10  , 20, 30, 35, 40, 42, 45, 50, 55, 58, 60, 62, 65],
                                       [-0.139, -0.138, -0.132, -0.134, -0.232, -0.289, -0.332, -0.407, -0.749, -0.705, -0.654, -0.622, -0.598, -0.574, -0.536]])

Gains['ThrPit_Trim_u'] = np.array([[0     ,  5    , 10    , 20    , 30    , 35    , 40, 42, 45, 50, 55, 58, 60, 62, 65],
                                   [-0.059, -0.059, -0.056, -0.041, -0.012,  0.   ,  0.   ,  0.   ,
        0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ]])

Gains['Elev2_Trim_u'] = np.array([[0   ,  5  , 10  , 20, 30, 35, 40, 42, 45, 50, 55, 58, 60, 62, 65],
                                  [0.667,  0.667,  0.667,  0.667,  0.432,  0.083, -0.018, -0.211,
       -0.119, -0.116, -0.107, -0.103, -0.101, -0.099, -0.097]])

Gains['Throttle_Trim_u'][1,:] = Gains['Throttle_Trim_u'][1,:] - Gains['Throttle_Trim_u'][1,0] +  TestEnv.TrimData['Action'][0]
Gains['ThrPit_Trim_u'][1,:]   = Gains['ThrPit_Trim_u'][1,:] - Gains['ThrPit_Trim_u'][1,0] +  TestEnv.TrimData['Action'][1]
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

class GenThrottleController:
    
    def __init__(self,Gains):
        self.Gains = Gains.copy()
        self.int_Alt = 0
        self.int_VZ  = 0
        self.Last_t  = 0
    
    def CalcCmd(self,Ref, obs,t):
        Alt2VZ = np.interp(obs[7] , self.Gains['EAS_mps'] , self.Gains['Alt2VZ'])
        VZ2NZ  = np.interp(obs[7] , self.Gains['EAS_mps'] , self.Gains['VZ2NZ'])
        Alt2NZ = np.interp(obs[7] , self.Gains['EAS_mps'] , self.Gains['Alt2NZ'])
        Az2Thr = np.interp(obs[7] , self.Gains['EAS_mps'] , self.Gains['Az2Thr'])
       
        VX_tgt_mps = np.interp(t,Gains['Target_VX_mps'][0,:],Gains['Target_VX_mps'][1,:])
        FF_Cmd_u = np.interp(VX_tgt_mps,Gains['Throttle_Trim_u'][0,:],Gains['Throttle_Trim_u'][1,:])
        
        Z_ref_m   = np.interp(t,Ref['Z_m'][0,:],Ref['Z_m'][1,:])
        Z_error_m = Z_ref_m - obs[2]
        
        VZ_ref_mps   = np.interp(t,Ref['VZ_mps'][0,:],Ref['VZ_mps'][1,:])
        VZ_error_mps = VZ_ref_mps + Z_error_m * Alt2VZ - obs[3]
        
        CL_Cmd_u = (VZ_error_mps*VZ2NZ + Z_error_m*Alt2NZ - (obs[4]+9.806) )*Az2Thr
        
        ThrottleCmd_u = FF_Cmd_u + CL_Cmd_u
        return ThrottleCmd_u

class GenPitchController:

    def __init__(self,Gains):
        self.Gains = Gains.copy()
        self.int_q = 0
        self.int_T = 0
        self.Last_t = 0

    def CalcCmd (self,Ref,OutPitchCmd_u , obs,t):
        # Calculate Gains
        Pitch2ThrP = np.interp(obs[7]  , self.Gains['EAS_mps'] , self.Gains['Pitch2ThrP'])
        Pitch2W2E  = np.interp(obs[7]  , self.Gains['EAS_mps'] , self.Gains['Pitch2W2E'])
        q2Pitch    = np.interp(obs[7]  , self.Gains['EAS_mps'] , self.Gains['q2Pitch'])
        i2Pitch    = np.interp(obs[7]  , self.Gains['EAS_mps'] , self.Gains['i2Pitch'])
        T2Pitch    = np.interp(obs[7]  , self.Gains['EAS_mps'] , self.Gains['T2Pitch'])
        Theta2q    = np.interp(obs[7]  , self.Gains['EAS_mps'] , self.Gains['Theta2q'])

        # Calculate References and Errors
        Theta_ref_deg = np.interp(t,Ref['Theta_deg'][0,:],Ref['Theta_deg'][1,:])
        Theta_error_rad = np.deg2rad(Theta_ref_deg) - obs[5]

        q_ref_radps   = Theta_error_rad*Theta2q
        q_error_radps   = q_ref_radps - obs[6]

        self.int_q += q_error_radps * (t-self.Last_t)
        self.int_T += Theta_error_rad * (t-self.Last_t)
        self.Last_t = t

        # Calculate Closed Loop Pitch Cmd
        PitchCmd_u = q_error_radps * q2Pitch + self.int_T*i2Pitch + Theta_error_rad*T2Pitch + OutPitchCmd_u
        
        # Calculate Feef Forward Pitch Cmd
        FF_PitThr = np.interp(obs[7] , self.Gains['ThrPit_Trim_u'][0,:] , self.Gains['ThrPit_Trim_u'][1,:])
        FF_Elev2  = np.interp(obs[7]  , self.Gains['Elev2_Trim_u'][0,:]  , self.Gains['Elev2_Trim_u'][1,:] )
       
        
        
        # Calculate Cmd Allocation
        PitchThrottle_u = PitchCmd_u * Pitch2ThrP + FF_PitThr
        W2_ElevCmd_u    = PitchCmd_u * Pitch2W2E + FF_Elev2

        return PitchThrottle_u , W2_ElevCmd_u


adm_vec = TestEnv.adm_vec


PitchController = GenPitchController(Gains)
ThrottleController = GenThrottleController(Gains)

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
    PitchThrottle_u , W2_ElevCmd_u = PitchController.CalcCmd(Ref, OutPitchCmd_u, obs, CurrentTime_s)
    
    u_Vert [step+1]  = ThrottleController.CalcCmd(Ref, obs, CurrentTime_s)
    
    u_Tilt1 [step+1] = 2 * (W1_TiltCmd_deg - TestEnv.CONT['MinTilt_deg'][0]) / TestEnv.CONT['TiltRange_deg'][0] - 1
    u_Tilt2 [step+1] = 2 * (W2_TiltCmd_deg - TestEnv.CONT['MinTilt_deg'][1]) / TestEnv.CONT['TiltRange_deg'][1] - 1

    u_Pitch [step+1] = PitchThrottle_u
    u_Elev2 [step+1] = W2_ElevCmd_u


# % CALL PLOT FILE
print('Return = {:0.1f}'.format(Return))
exec(open("./Test_VTOL_TestPID_Transition_plot.py").read())
fig.savefig('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PID_transition.pdf', bbox_inches='tight')

# exec(open("./Test_VTOL_TestPID_Transition_CheckSensors.py").read())
# exec(open("./Test_VTOL_TestPID_Transition_CheckAero.py").read())

print('Max Alt  : {:0.1f}'.format(np.max(SaveVec['H_m'])))
print('Min Alt  : {:0.1f}'.format(np.min(SaveVec['H_m'])))
print('Max Pitch: {:0.1f}'.format(np.max(SaveVec['Theta_deg'])))
print('Min Pitch: {:0.1f}'.format(np.min(SaveVec['Theta_deg'])))
# %% PLOT GAINS
if False:
    CmdPitThrWeight_vec = np.array([[-5 , 35 , 50 , 100] ,
                                    [ 1 , 1  , 0  , 0  ]])
    
    
    
    plt_l = 3
    plt_c = 3
    plt_n = 1
    
    XLIM = [np.min(Gains['EAS_mps'] ),np.max(Gains['EAS_mps'] )]
    
    fig = plt.figure()
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(CmdPitThrWeight_vec[0,:]  , CmdPitThrWeight_vec[1,:],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{pt}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['Pitch2ThrP'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{pt} * PT_{eff}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['Pitch2W2E'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{pt} * ELEV_{eff}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['q2Pitch'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{q}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['Theta2q'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{\theta}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['i2Pitch'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{i}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['Az2Thr'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{NZ}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['Alt2VZ'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{H}$')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim(XLIM)
    plt.plot(Gains['EAS_mps']  , Gains['VZ2NZ'],'k-', linewidth = 2)
    # plt.legend(loc='best')
    plt.xlabel('EAS [m/s]')
    plt.ylabel('$K_{VZ}$')
    
    fig.set_size_inches(7, 4)
    fig.tight_layout() 
    
    fig.savefig('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PID_gains.pdf', bbox_inches='tight')
    print('Fig Saved')
    
    plt.show()
    
    
    
