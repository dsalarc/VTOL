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
from PID_design_PitchFunctions import gen_EngActuator, gen_ElevActuator, Controller, Sensor, gen_ControlAllocation
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
# TrimVec['VX_mps']        = np.array([  0.0 ,   5.0])

TrimVec['PitchThrottle_to_qdot']  = 0.0*TrimVec['VX_mps']
TrimVec['Elevator_to_qdot']       = 0.0*TrimVec['VX_mps']
for n_trim in range(len(TrimVec['VX_mps'])):
    obs = TestEnv.reset(VX_mps = TrimVec['VX_mps'][n_trim], VZ_mps = 0.0, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)
    TrimVec['PitchThrottle_to_qdot'][n_trim] = abs(TestEnv.TrimData['Linear']['B'][0,1])
    TrimVec['Elevator_to_qdot'][n_trim]      = abs(TestEnv.TrimData['Linear']['B'][0,4]) 

TrimVec['ContAlloc_Thr'] =   TrimVec['PitchThrottle_to_qdot'] /(TrimVec['PitchThrottle_to_qdot'] + 5*TrimVec['Elevator_to_qdot'])  
    
GainVec = {}

# %% INITIALIZE GAINS VECTOR
GainsVec = {}
GainsVec['Kqp'] = 0.0*TrimVec['VX_mps'] 
GainsVec['Kqi'] = 0.0*TrimVec['VX_mps'] 
GainsVec['Kt']  = 0.0*TrimVec['VX_mps'] 
GainsVec['Kff'] = 0.0*TrimVec['VX_mps'] 
GainsVec['ContAlloc_Thr'] = 0*TrimVec['VX_mps'] 
CostVec = 0.0*TrimVec['VX_mps'] 

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

for n_trim in range(len(TrimVec['VX_mps'])):
    print(' ')
    print("Optimizing Speed %d / %d" %(n_trim+1,len(TrimVec['VX_mps'])))
    obs = TestEnv.reset(VX_mps = TrimVec['VX_mps'][n_trim], VZ_mps = 0.0, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)
    
    # %%
    Aircraft = {}
    Aircraft['SpeedIncluded'] = {}
    Aircraft['SpeedIncluded']['SS'] = ct.ss(TestEnv.TrimData['Linear']['A'] , 
                                              TestEnv.TrimData['Linear']['B'] , 
                                              TestEnv.TrimData['Linear']['C'] , 
                                              TestEnv.TrimData['Linear']['D'] , 
                                              inputs=TestEnv.TrimData['Linear']['InpNames'] , 
                                              states=TestEnv.TrimData['Linear']['StaNames'] , 
                                              outputs = TestEnv.TrimData['Linear']['OutNames'],
                                              name = 'Aircraft' )
    # %%
    
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
    EngActuator     = gen_EngActuator(wn_radps = 40)
    ElevActuator    = gen_ElevActuator(wn_radps = 40)
    Sensor_q        = Sensor(wn_radps = 40, inp_name = 'Q_degps', out_name = 'Q_sen_degps' , sensor_name = 'Sensor_q')
    Sensor_t        = Sensor(wn_radps = 40, inp_name = 'Theta_deg', out_name = 'Theta_sen_deg' , sensor_name = 'Sensor_t')
    # %%
    
    # TotalCost = GeneralPitchFunction(InpGains = [0.04, 0.02, 0.1, 0.0])   
    
    TotalCost = GeneralPitchFunction()
    
    if n_trim == 0:
        x0 = [0.04, 0.02, 0.9, 0.0]
    else:
        x0 = OptGains['x']
        
    bnds = ((0.0, 0.1),
            (0.0, 0.2),
            (0.1, 3.0),
            (0.0, 0.1))
    
    # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][n_trim]), method = 'Nelder-Mead' , bounds=bnds)
   
    # t = time.time()
    # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][n_trim]), method = 'TNC' , bounds=bnds)
    # elapsed = time.time() - t
    # print('TNC Elapsed Time [s]: %0.1f' %(elapsed))
   
    # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][n_trim]), method = 'SLSQP' , bounds=bnds)
    
    t = time.time()    
    OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][n_trim]), method = 'Powell' , bounds=bnds)
    elapsed = time.time() - t
    print('Powell Elapsed Time [s]: %0.1f' %(elapsed))
   
    # OptGains = opt.minimize(GeneralPitchFunction , x0, args = (TrimVec['ContAlloc_Thr'][n_trim]), method = 'L-BFGS-B' , bounds=bnds)
    
    # SAVE GAINS
    GainsVec['Kqp']           [n_trim] = OptGains['x'][0]
    GainsVec['Kqi']           [n_trim] = OptGains['x'][1]
    GainsVec['Kt']            [n_trim] = OptGains['x'][2]
    GainsVec['Kff']           [n_trim] = OptGains['x'][3]
    GainsVec['ContAlloc_Thr'] [n_trim] = TrimVec['ContAlloc_Thr'][n_trim]

    Gains = {}
    for kk in GainsVec.keys():
        Gains[kk] = GainsVec[kk][n_trim]
    
    PitchController = Controller(Gains)
    ControlAllocation = gen_ControlAllocation(Gains)
    ClosedLoops = PitchClosedLoops(Aircraft , PitchController, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation)
    Criteria    = CalculateIndividualCosts(ClosedLoops)
    TotalCost   = CalculateTotalCost(Criteria)
    CostVec[n_trim] = TotalCost

    print('Final Cost: %0.2f' %(TotalCost))
    
    print_gains = ''
    for kk in GainsVec.keys():
        print_gains = print_gains + kk + ': ' + format(GainsVec[kk][n_trim],'0.4f')+ ';  '
    print(print_gains)
    
# %% PLOT RESULTS
plt.close('all')
for n_trim in range(len(TrimVec['VX_mps'])):
    Gains = {}
    for kk in GainsVec.keys():
        Gains[kk] = GainsVec[kk][n_trim]
    
    PitchController = Controller(Gains)
    ControlAllocation = gen_ControlAllocation(Gains)
    ClosedLoops = PitchClosedLoops(Aircraft , PitchController, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation)
    Criteria    = CalculateIndividualCosts(ClosedLoops)
    TotalCost   = CalculateTotalCost(Criteria)
    
    PitchPlots(ClosedLoops , Criteria, (str(TrimVec['VX_mps'][n_trim]) + 'm/s') )
    
# %% PLOT GAINS

plt.figure('Gains')

keys = GainsVec.keys()
l = int(np.ceil(np.sqrt(1+len(keys))))
c = int(np.ceil(len(keys) / l))

for i in range(len(keys)-1):
    plt.subplot(l,c,i+1)
    plt.plot(TrimVec['VX_mps'] , GainsVec[list(keys)[i]])
    plt.grid('on')
    plt.ylabel(list(keys)[i])
    plt.ylim(bnds[i])
    plt.xlabel('VX [mps]')
    
plt.subplot(l,c,i+2)
plt.plot(TrimVec['VX_mps'] , CostVec)
plt.grid('on')
plt.ylabel('Cost')
plt.xlabel('VX [mps]')

    
plt.tight_layout()
plt.show()

File2Save = {'GainsVec': GainsVec, 'TrimVec': TrimVec}
with open('SavedGains3.pkl', 'wb') as fp:
    pickle.dump(File2Save, fp)


with open('SavedGains3.pkl', 'rb') as fp:
    std = pickle.load(fp)
