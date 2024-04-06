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
import pickle

# %% START ENV
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')   

# %% DEFINE DISTURBANCE OVER TIME  
reset_INPUT_VEC = {}
reset_INPUT_VEC['FAILURE_MOT_5']   = np.array([[0, 20, 20.01, 40] , [0, 0, 0, 0]])
reset_INPUT_VEC['WIND_TowerX_mps'] = np.array([[0, 20, 40] , [0, 0, 0]])
reset_INPUT_VEC['WIND_TowerY_mps'] = np.array([[0, 20, 40] , [0, 0, 0]])
reset_INPUT_VEC['WIND_TurbON']     = np.array([[0, 40] , [0, 0]])

Vtrim_mps = 50.0

# %% TRIM ENV
obs = TestEnv.reset(VX_mps = Vtrim_mps, VZ_mps = 0.0, THETA = 0.0, DispMessages = False, 
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
Ref['VX_mps'] = np.array([[0 , 5 , 30, SimTime ],
                          [50 , 50 , 50 , 50    ]])
Ref['Z_m'] = np.array([[0    , 2.0 , 5.01, SimTime ],
                       [-100 , -100 , -100 , -100    ]])

Ref['Theta_deg'] = np.array([[0 , 2, 2.01, 1000  ],
                             [0 , 0, 5*0   , 5*0     ]])

Ref['ControlMerge'] = np.array([[0 , 45.0 , 60.0, 100.0 ],
                                [0 ,    1 ,    1,     1 ]])


# %% LOAD GAINS

Gains = {}

with open('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PID/SavedGains_20231222_1134_Nelder-Mead_smooth.pkl', 'rb') as fp:
    std = pickle.load(fp)

Gains['Pitch'] = {}
Gains['Pitch']['EAS_mps']        = std['TrimVec']['VX_mps']
for kk in std['GainsVec'].keys():
    Gains['Pitch'][kk] = std['GainsVec'][kk]

Gains['Tilt'] = {}
Gains['Tilt']['Target_VX_mps'] = Ref['VX_mps'] 
Gains['Tilt']['Tilt_Trim_deg'] = np.array([std['TrimVec']['VX_mps'],np.transpose(std['TrimVec']['TestTilt_deg'][:,2])])
Gains['Tilt']['Tilt_AX2p5']    = np.array([std['TrimVec']['VX_mps'],np.transpose(std['TrimVec']['TestTilt_deg'][:,3])])
Gains['Tilt']['Tilt_AX2n5']    = np.array([std['TrimVec']['VX_mps'],np.transpose(std['TrimVec']['TestTilt_deg'][:,1])])

with open('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PID/VerticalDesign/SavedGains_VerticalController_20231229_1038_Nelder-Mead_smooth.pkl', 'rb') as fp:
    std = pickle.load(fp)

Gains['Vert'] = {}
Gains['Vert']['EAS_mps'] = std['TrimVec']['VX_mps']
for kk in std['GainsVec'].keys():
    Gains['Vert'][kk] = std['GainsVec'][kk]

with open('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PID/VerticalForward_Design/SavedGains_VertFwdController_20240313_2235_Nelder-Mead_smooth.pkl', 'rb') as fp:
    std = pickle.load(fp)

Gains['FwdVert'] = {}
Gains['FwdVert']['EAS_mps'] = std['TrimVec']['VX_mps']
for kk in std['GainsVec'].keys():
    Gains['FwdVert'][kk] = std['GainsVec'][kk]


# %%
TimeVec = np.arange(0,SimTime+TestEnv.t_step/2,TestEnv.t_step)
n_steps = np.size(TimeVec)

VZ_int  = 0
The_int = 0

u_Vert  = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')]
u_PThro = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')]
u_Tilt1 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W1_Tilt')]
u_Tilt2 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Tilt')]
u_Elev2 = np.ones(n_steps+1)*TestEnv.TrimData['Action'][TestEnv.action_names.index('W2_Elevator')]
vec_Theta_ref_deg = np.zeros(n_steps+1)


# ContAlloc_Thr  = np.interp(Vtrim_mps  , Gains['EAS_mps'] , Gains['ContAlloc_Thr'])
# u0_Pitch = TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')]/ContAlloc_Thr

Return = 0

def TiltController(Gains,VX_tgt_mps , VX_cur_mps):
    TiltCmd_min_deg = TiltCmd_deg = np.interp(VX_tgt_mps,Gains['Tilt_AX2p5'][0,:],Gains['Tilt_AX2p5'][1,:])
    TiltCmd_max_deg = TiltCmd_deg = np.interp(VX_tgt_mps,Gains['Tilt_AX2n5'][0,:],Gains['Tilt_AX2n5'][1,:])
    
    TiltCmd_deg = np.min((TiltCmd_max_deg , np.max((TiltCmd_min_deg , 
                          np.interp(VX_tgt_mps,Gains['Tilt_Trim_deg'][0,:],Gains['Tilt_Trim_deg'][1,:])))))
           
    return TiltCmd_deg,TiltCmd_deg

class GenThrottleController:
    
    def __init__(self,Gains,int0 = 0.0):
        self.Gains = Gains.copy()
        self.int_VZ  = 0.0
        self.Last_t  = 0.0
        self.int0    = int0
    
    def CalcCmd(self,Ref, info,t):       
        Kvzp  = np.interp(info['SENS']['CAS_mps'], self.Gains['EAS_mps'] , self.Gains['Kvzp'])
        Kvzi  = np.interp(info['SENS']['CAS_mps'], self.Gains['EAS_mps'] , self.Gains['Kvzi'])
        Kz    = np.interp(info['SENS']['CAS_mps'], self.Gains['EAS_mps'] , self.Gains['Kz'])
        Kvzff = np.interp(info['SENS']['CAS_mps'], self.Gains['EAS_mps'] , self.Gains['Kvzff'])
        Knzp  = np.interp(info['SENS']['CAS_mps'], self.Gains['EAS_mps'] , self.Gains['Knzp'])
       
        Z_ref_m = np.interp(t,Ref['Z_m'][0,:],Ref['Z_m'][1,:])
        Z_error_m = Z_ref_m - info['SENS']['Z_m']

        VZ_cmd_mps   = Z_error_m*Kz
        VZ_error_mps = VZ_cmd_mps - info['SENS']['VZ_mps']
 
        NZi_mps2 =  info['SENS']['NZi_mps2']+9.806

        self.int_VZ += VZ_error_mps * (t-self.Last_t)
        self.Last_t = t
        
        ThrottleCmd_u = VZ_error_mps * Kvzp + self.int_VZ*Kvzi + VZ_cmd_mps*Kvzff + NZi_mps2*Knzp + self.int0
        return ThrottleCmd_u

class GenPitchController:

    def __init__(self,Gains,int0 = 0.0):
        self.Gains = Gains.copy()
        self.int_q = 0.0
        self.Last_t = 0.0
        self.int0 = int0

    def CalcCmd (self,Theta_ref_deg , OutPitchCmd_u , info,t):
        # Calculate Gains
        Kqp            = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kqp'])
        Kqi            = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kqi'])
        Kt             = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kt'])
        Kff            = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kff'])
        ContAlloc_Thr  = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['ContAlloc_Thr'])


        # Calculate References and Errors
        Theta_error_deg = Theta_ref_deg - np.rad2deg(info['SENS']['Theta_rad'])

        q_cmd_degps   = Theta_error_deg*Kt
        q_error_degps = q_cmd_degps - np.rad2deg(info['SENS']['Q_radps'])

        self.int_q += q_error_degps * (t-self.Last_t)
        self.Last_t = t

        # Calculate Closed Loop Pitch Cmd
        PitchCmd_u = q_error_degps * Kqp + self.int_q*Kqi + q_cmd_degps*Kff + self.int0        
        
        # Calculate Cmd Allocation
        PitchThrottle_u = PitchCmd_u * ContAlloc_Thr
        W2_ElevCmd_u    = -PitchCmd_u * (1-ContAlloc_Thr)

        return PitchThrottle_u , W2_ElevCmd_u
    
class GenFWController:

    def __init__(self,Gains, Last_t = 0.0, Throttle0 = 0.0 , Theta0 = 0.0):
        self.Gains  = Gains.copy()
        self.int_VX      = 0.0
        self.int_VZ      = 0.0
        self.Last_t      = Last_t
        self.Throttle0   = Throttle0
        self.Theta0      = Theta0

    def CalcCmd (self , Z_ref_m , VX_ref_mps , info , t,  IntGain = 1.0):
        # Calculate Gains
        Kvzp     = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kvzp'])
        Kvzi     = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kvzi'])
        Kz       = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kz'])
        Kvzff    = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kvzff'])
        Knzp     = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Knzp'])
        Kvxp     = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kvxp'])
        Kvxi     = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kvxi'])
        Knxp     = np.interp(info['SENS']['CAS_mps'] , self.Gains['EAS_mps'] , self.Gains['Kaxp'])
        Kvxff    = 0


        # Calculate Theta Command
        Z_error_m = Z_ref_m - info['SENS']['Z_m']

        VZ_cmd_mps   = Z_error_m*Kz
        VZ_error_mps = VZ_cmd_mps - info['SENS']['VZ_mps']
 
        NZi_mps2 =  info['SENS']['NZi_mps2']+9.806

        self.int_VZ += VZ_error_mps * (t-self.Last_t) * IntGain
        
        ThetaCmd_deg = VZ_error_mps * Kvzp + self.int_VZ*Kvzi + VZ_cmd_mps*Kvzff - NZi_mps2*Knzp + self.Theta0
        
        # Calculate Throttle Command
        VX_error_mps = VX_ref_mps - info['SENS']['VX_mps']
 
        NXi_mps2 =  info['SENS']['NXi_mps2']

        self.int_VX += VX_error_mps * (t-self.Last_t) * IntGain
        self.Last_t = t
        
        ThrottleCmd_u = VX_error_mps * Kvxp + self.int_VX*Kvxi + VX_ref_mps*Kvxff + NXi_mps2*Knxp + self.Throttle0

        return ThetaCmd_deg , ThrottleCmd_u
    

adm_vec = TestEnv.adm_vec


PitchController = GenPitchController(Gains['Pitch'] , int0 =  TestEnv.TrimData['Action'][TestEnv.action_names.index('PitchThrottle')])
ThrottleController = GenThrottleController(Gains['Vert'] , int0 =  TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')])

start_FWControl = False

for step in range(n_steps):
    CurrentTime_s = step*TestEnv.t_step
    
    VX_ref_mps    = np.interp(CurrentTime_s , Ref['VX_mps'][0,:]   , Ref['VX_mps'][1,:]   )
    Z_ref_m       = np.interp(CurrentTime_s , Ref['Z_m'][0,:]      , Ref['Z_m'][1,:]      )
    MC_Theta_ref_deg = np.interp(CurrentTime_s , Ref['Theta_deg'][0,:], Ref['Theta_deg'][1,:])
    OutPitchCmd_u = np.interp(CurrentTime_s , Inp['Pitch_u'][0,:]  , Inp['Pitch_u'][1,:])
    
    InputVec = np.array([u_Vert[step] ,
                        u_PThro[step],
                        u_Tilt1[step],
                        u_Tilt2[step],
                        u_Elev2[step]])
    
    obs, reward, done, info = TestEnv.step(InputVec)
    
    Return = Return + reward
    SaveVec = SaveSelection(SaveVec,TestEnv.info)
    # if done:
    #     print("Goal reached!", "reward=", reward)
    #     break
    
    # Check if FW control shall start
    ControlMerge = np.interp(info['SENS']['VX_mps'] , Ref['ControlMerge'][0,:] , Ref['ControlMerge'][1,:])
    if ((not start_FWControl) and (ControlMerge > 0.0001)):
        start_FWControl = True
        FWController = GenFWController(Gains['FwdVert'], Last_t = 0.0, Throttle0 = TestEnv.TrimData['Action'][TestEnv.action_names.index('Throttle')] , Theta0 = 0.0)
        
    if start_FWControl:
        FW_ThetaCmd_deg , FW_ThrottleCmd_u = FWController.CalcCmd(Z_ref_m = Z_ref_m , VX_ref_mps = VX_ref_mps  , info = info, t = CurrentTime_s, IntGain = ControlMerge)
    else:
        FW_ThetaCmd_deg = 0.0
        FW_ThrottleCmd_u = 0.0
        
    MC_ThrottleCmd_u = ThrottleController.CalcCmd(Ref, info, CurrentTime_s)
    
    ThrottleCmd_u = ControlMerge * FW_ThrottleCmd_u + (1-ControlMerge) * MC_ThrottleCmd_u
    Theta_ref_deg = ControlMerge * FW_ThetaCmd_deg  + (1-ControlMerge) * MC_Theta_ref_deg
        
    PitchThrottle_u , W2_ElevCmd_u = PitchController.CalcCmd(Theta_ref_deg, OutPitchCmd_u, info, CurrentTime_s)
    
    VX_tgt1_mps = np.interp(CurrentTime_s,Gains['Tilt']['Target_VX_mps'][0,:],Gains['Tilt']['Target_VX_mps'][1,:])
    VX_tgt2_mps = np.interp(np.max((0,CurrentTime_s-0)),Gains['Tilt']['Target_VX_mps'][0,:],Gains['Tilt']['Target_VX_mps'][1,:])
    W1_TiltCmd_deg , dum  = TiltController(Gains['Tilt'],VX_tgt1_mps , info['ATM']['CAS_mps'])
    dum , W2_TiltCmd_deg  = TiltController(Gains['Tilt'],VX_tgt2_mps , info['ATM']['CAS_mps'])
    
    
    u_Vert [step+1]  = ThrottleCmd_u
    
    u_Tilt1 [step+1] = 2 * (W1_TiltCmd_deg - TestEnv.CONT['MinTilt_deg'][0]) / TestEnv.CONT['TiltRange_deg'][0] - 1
    u_Tilt2 [step+1] = 2 * (W2_TiltCmd_deg - TestEnv.CONT['MinTilt_deg'][1]) / TestEnv.CONT['TiltRange_deg'][1] - 1

    u_PThro [step+1] = PitchThrottle_u
    u_Elev2 [step+1] = W2_ElevCmd_u
    
    vec_Theta_ref_deg[step+1] = Theta_ref_deg


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
    
    
    
