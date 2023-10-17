#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 20:21:50 2023

@author: dsalarc
"""
import gym
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
from PID_design_PitchCosts import CalculateIndividualCosts, CalculateTotalCost
from PID_design_PitchClosedLoops import PitchClosedLoops
from PID_design_PitchPlots import PitchPlots
from PID_design_PitchFunctions import gen_EngActuator, gen_ElevActuator, Controller, Sensor, gen_ControlAllocation, gen_Aircraft
import control as ct
import time
import pickle

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
except:
    pass

# %%LOAD MODEL
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')


# %% DEFINE TRIM VECTOR
TrimVec = {}
TrimVec['VX_mps']        = np.array([  0.0 ,   5.0 ,   10.0 ,   20.0 ,   30.0 ,  35.0 ,   40.0 ,   50.0 ,   60.0 ])
TrimVec['VX_mps']        = np.array([  0.0 ,   5.0 ,   10.0 ,   20.0 ,   25.0, 30.0 ,  35.0 ,   40.0 ,   50.0, 60.0])
TrimVec['VX_mps']        = np.arange(0.0, 60.1, 5.0)
TrimVec['TrimTilt_deg']  = np.arange(0.0, 90.1, 5.0)

# TrimVec['VX_mps']        = np.array([ 27.0, 28.0, 29.0, 29.5 ,   30.0 ,  30.5, 31.0, 32.0, 33.0])
# TrimVec['TrimTilt_deg']  = np.arange(0.0, 90.1, 90.0,)

TrimVec['PitchThrottle_to_qdot']  = 0.0*TrimVec['VX_mps']
TrimVec['Elevator_to_qdot']       = 0.0*TrimVec['VX_mps']
for nv_trim in range(len(TrimVec['VX_mps'])):
    obs = TestEnv.reset(VX_mps = TrimVec['VX_mps'][nv_trim], VZ_mps = 0.0, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, Elevator_deg = 0, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)
    TrimVec['PitchThrottle_to_qdot'][nv_trim] = abs(TestEnv.TrimData['Linear']['B'][0,1])
    TrimVec['Elevator_to_qdot'][nv_trim]      = abs(TestEnv.TrimData['Linear']['B'][0,4]) 

TrimVec['ContAlloc_Thr'] =   TrimVec['PitchThrottle_to_qdot'] /(TrimVec['PitchThrottle_to_qdot'] + TrimVec['Elevator_to_qdot'])  
# TrimVec['ContAlloc_Thr'] =   0.1*TrimVec['PitchThrottle_to_qdot'] /(0.1*TrimVec['PitchThrottle_to_qdot'] + 0.0*TrimVec['Elevator_to_qdot'])  
  
GainVec = {}

# %% INITIALIZE GAINS VECTOR
GainsVec = {}
GainsVec['Kqp'] = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['TrimTilt_deg'])))
GainsVec['Kqi'] = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['TrimTilt_deg'])))
GainsVec['Kt']  = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['TrimTilt_deg'])))
GainsVec['Kff'] = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['TrimTilt_deg'])))
GainsVec['ContAlloc_Thr'] = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['TrimTilt_deg'])))
CostVec = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['TrimTilt_deg'])))

def GeneralPitchFunction(InpGains = [0.04, 0.02, 0.9, 0.0], ContAlloc_Thr = 1):

    Gains = {}
    Gains['Kqp'] = InpGains[0]
    Gains['Kqi'] = InpGains[1]
    Gains['Kt']  = InpGains[2]
    Gains['Kff'] = InpGains[3]
    Gains['ContAlloc_Thr'] = ContAlloc_Thr
    
    PitchController = Controller(Gains)
    ControlAllocation = gen_ControlAllocation(Gains)
    
    ClosedLoops = PitchClosedLoops(Aircraft , PitchController, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation)
    Criteria    = CalculateIndividualCosts(ClosedLoops)
    TotalCost   = CalculateTotalCost(Criteria)
    
    
    return TotalCost


SaveAircraft = []
Trimmed = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['TrimTilt_deg'])))

for nv_trim in range(len(TrimVec['VX_mps'])):
    for nt_trim in range(len(TrimVec['TrimTilt_deg'])):
        print(' ')
        print("Optimizing Speed %d / %d" %(nv_trim+1,len(TrimVec['VX_mps'])))
        print("Optimizing Tilt %d / %d" %(nt_trim+1,len(TrimVec['TrimTilt_deg'])))
        
               
       
        # %%
        # Aircraft,TrimData = gen_Aircraft(TestEnv, VX_mps = TrimVec['VX_mps'][nv_trim], Tilt_deg = None, Elevator_deg = 0) #TrimVec['TrimTilt_deg'][nt_trim])
        Aircraft,TrimData = gen_Aircraft(TestEnv, VX_mps = TrimVec['VX_mps'][nv_trim], Tilt_deg = TrimVec['TrimTilt_deg'][nt_trim], Elevator_deg = 0) #TrimVec['TrimTilt_deg'][nt_trim])
        Trimmed[nv_trim, nt_trim] = TrimData['Trimmed'] 
        if np.abs(Trimmed[nv_trim, nt_trim] - 1) > 0.1:
            # SAVE GAINS
            GainsVec['Kqp']           [nv_trim,nt_trim] = np.nan
            GainsVec['Kqi']           [nv_trim,nt_trim] = np.nan
            GainsVec['Kt']            [nv_trim,nt_trim] = np.nan
            GainsVec['Kff']           [nv_trim,nt_trim] = np.nan
            GainsVec['ContAlloc_Thr'] [nv_trim,nt_trim] = np.nan
        
            SaveAircraft.append(Aircraft)
            print('Condition not trimmed')

        else:
            EngActuator       = gen_EngActuator(wn_radps = 40)
            ElevActuator      = gen_ElevActuator(wn_radps = 40)
            Sensor_q          = Sensor(wn_radps = 40, inp_name = 'Q_degps', out_name = 'Q_sen_degps' , sensor_name = 'Sensor_q')
            Sensor_t          = Sensor(wn_radps = 40, inp_name = 'Theta_deg', out_name = 'Theta_sen_deg' , sensor_name = 'Sensor_t')
            
            SaveAircraft.append(Aircraft)
            # %%
               
            TotalCost = GeneralPitchFunction()
            
            # if nv_trim == 0:
            x0 = [0.01, 0.015, 2.0, 0.0]
            # else:
            #     x0 = OptGains['x']
                
            bnds = ((0.0, 0.1),
                    (0.0, 0.2),
                    (0.1, 5.0),
                    (0.0, 0.1))
            
            # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][nv_trim]), method = 'Nelder-Mead' , bounds=bnds)
           
            # t = time.time()
            # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][nv_trim]), method = 'TNC' , bounds=bnds)
            # elapsed = time.time() - t
            # print('TNC Elapsed Time [s]: %0.1f' %(elapsed))
           
            # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][nv_trim]), method = 'SLSQP' , bounds=bnds)
            
            t = time.time()    
            OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][nv_trim]), method = 'Powell' , bounds=bnds)
            elapsed = time.time() - t
            print('Powell Elapsed Time [s]: %0.1f' %(elapsed))
           
            # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][nv_trim]), method = 'L-BFGS-B' , bounds=bnds)
            
            # SAVE GAINS
            GainsVec['Kqp']           [nv_trim,nt_trim] = OptGains['x'][0]
            GainsVec['Kqi']           [nv_trim,nt_trim] = OptGains['x'][1]
            GainsVec['Kt']            [nv_trim,nt_trim] = OptGains['x'][2]
            GainsVec['Kff']           [nv_trim,nt_trim] = OptGains['x'][3]
            GainsVec['ContAlloc_Thr'] [nv_trim,nt_trim] = TrimVec['ContAlloc_Thr'][nv_trim]
        
            Gains = {}
            for kk in GainsVec.keys():
                Gains[kk] = GainsVec[kk][nv_trim,nt_trim]
            
            PitchController = Controller(Gains)
            ControlAllocation = gen_ControlAllocation(Gains)
            ClosedLoops = PitchClosedLoops(Aircraft , PitchController, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation)
            Criteria    = CalculateIndividualCosts(ClosedLoops)
            TotalCost   = CalculateTotalCost(Criteria)
            CostVec[nv_trim,nt_trim] = TotalCost
        
            print('Final Cost: %0.2f' %(TotalCost))
            
            print_gains = ''
            for kk in GainsVec.keys():
                print_gains = print_gains + kk + ': ' + format(GainsVec[kk][nv_trim,nt_trim],'0.4f')+ ';  '
            print(print_gains)
    
# %% PLOT RESULTS
plt.close('all')
n_save = -1
for nv_trim in range(len(TrimVec['VX_mps'])):
    for nt_trim in range(len(TrimVec['TrimTilt_deg'])):
        n_save += 1
        if Trimmed[nv_trim, nt_trim] != 1:
            print("Speed: %0.1f / Tilt: %02.0f / Not trimmed" %(TrimVec['VX_mps'][nv_trim] , TrimVec['TrimTilt_deg'][nt_trim]))            
        else:              
            Gains = {}
            for kk in GainsVec.keys():
                Gains[kk] = GainsVec[kk][nv_trim,nt_trim]
               
            PitchController = Controller(Gains)
            ControlAllocation = gen_ControlAllocation(Gains)
            ClosedLoops = PitchClosedLoops(SaveAircraft[n_save] , PitchController, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation)
            Criteria    = CalculateIndividualCosts(ClosedLoops)
            TotalCost   = CalculateTotalCost(Criteria)
            
            print("Speed: %0.1f / Tilt: %02.0f / TotalCost: %0.2f " %(TrimVec['VX_mps'][nv_trim] , TrimVec['TrimTilt_deg'][nt_trim] , TotalCost))
            PitchPlots(ClosedLoops , Criteria, (str(TrimVec['VX_mps'][nv_trim]) + 'm/s | Tilt ' + str(TrimVec['TrimTilt_deg'][nt_trim])) )
    
# %% PLOT GAINS

plt.figure('Gains')

keys = GainsVec.keys()
l = int(np.ceil(np.sqrt(1+len(keys))))
c = int(np.ceil(len(keys) / l))

for i in range(len(keys)-1):
    plt.subplot(l,c,i+1)
    for nt_trim in range(len(TrimVec['TrimTilt_deg'])):
        plt.plot(TrimVec['VX_mps'] , GainsVec[list(keys)[i]][:,nt_trim],'o-')
        
    plt.grid('on')
    plt.ylabel(list(keys)[i])
    # plt.ylim(bnds[i])
    plt.xlabel('VX [mps]')
    
plt.subplot(l,c,i+2)
for nt_trim in range(len(TrimVec['TrimTilt_deg'])):
    plt.plot(TrimVec['VX_mps'] , CostVec[:,nt_trim],'o-')
plt.grid('on')
plt.ylabel('Cost')
plt.xlabel('VX [mps]')

    
plt.tight_layout()
plt.show()

File2Save = {'GainsVec': GainsVec, 'TrimVec': TrimVec, 'CostVec': CostVec}
with open('SavedGains3.pkl', 'wb') as fp:
    pickle.dump(File2Save, fp)


# with open('SavedGains3.pkl', 'rb') as fp:
#     std = pickle.load(fp)
