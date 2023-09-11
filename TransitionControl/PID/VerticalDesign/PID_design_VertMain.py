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
from PID_design_VertCosts import CalculateIndividualCosts, CalculateTotalCost
from PID_design_VertClosedLoops import gen_ClosedLoops
from PID_design_VertPlots import gen_Plots
from PID_design_VertFunctions import gen_EngActuator, gen_Controller, gen_Sensor
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
# TrimVec['VX_mps']        = np.array([  0.0 ])
# TrimVec['VX_mps']        = np.array([  0.0 , 10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
# TrimVec['VX_mps']        = np.array([  0.0 , 60.0])

# %% INITIALIZE GAINS VECTOR
GainsVec = {}
GainsVec['Kvzp']  = 0.0*TrimVec['VX_mps'] 
GainsVec['Kvzi']  = 0.0*TrimVec['VX_mps'] 
GainsVec['Kz']    = 0.0*TrimVec['VX_mps'] 
GainsVec['Kvzff'] = 0.0*TrimVec['VX_mps'] 
GainsVec['Knzp']  = 0.0*TrimVec['VX_mps'] 
CostVec = 0.0*TrimVec['VX_mps'] 

def GeneralFunction(InpGains = [0.04, 0.02, 0.9, 0.0, 0.0]):

    Gains = {}
    Gains['Kvzp']  = InpGains[0]
    Gains['Kvzi']  = InpGains[1]
    Gains['Kz']    = InpGains[2]
    Gains['Kvzff'] = InpGains[3]
    Gains['Knzp']  = InpGains[4]
    
    Controller = gen_Controller(Gains)
    
    ClosedLoops = gen_ClosedLoops(Aircraft , Controller, Sensor_vz , Sensor_z, Sensor_az, EngActuator)
    Criteria    = CalculateIndividualCosts(ClosedLoops)
    TotalCost   = CalculateTotalCost(Criteria)
    
    
    return TotalCost

SaveAircraft = []
for n_trim in range(len(TrimVec['VX_mps'])):
    print(' ')
    print("Optimizing Speed %d / %d" %(n_trim+1,len(TrimVec['VX_mps'])))
    obs = TestEnv.reset(VX_mps = TrimVec['VX_mps'][n_trim], VZ_mps = 0.0, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)
    
    # %%
    Aircraft = {}
    Aircraft['PitchIncluded'] = {}
    Aircraft['PitchIncluded']['SS'] = ct.ss(TestEnv.TrimData['Linear']['A'] , 
                                              TestEnv.TrimData['Linear']['B'] , 
                                              TestEnv.TrimData['Linear']['C'] , 
                                              TestEnv.TrimData['Linear']['D'] , 
                                              inputs=TestEnv.TrimData['Linear']['InpNames'] , 
                                              states=TestEnv.TrimData['Linear']['StaNames'] , 
                                              outputs = TestEnv.TrimData['Linear']['OutNames'],
                                              name = 'Aircraft' )
    Aircraft['PitchNotIncluded'] = {}
    States_to_remove = [TestEnv.TrimData['Linear']['StaNames'].index('Q_radps') , 
                        TestEnv.TrimData['Linear']['StaNames'].index('Theta_rad')]
    States_to_include = list(np.arange(0, len(TestEnv.TrimData['Linear']['StaNames'])))
    for i in range(len(States_to_remove)):
        States_to_include.remove(States_to_remove[i])
    Aircraft['PitchNotIncluded']['SS'] = ct.ss(TestEnv.TrimData['Linear']['A'][States_to_include,:][:,States_to_include] , 
                                              TestEnv.TrimData['Linear']['B'][States_to_include,:] , 
                                              TestEnv.TrimData['Linear']['C'][:,States_to_include] , 
                                              TestEnv.TrimData['Linear']['D'] , 
                                              inputs=TestEnv.TrimData['Linear']['InpNames'] , 
                                              states=[TestEnv.TrimData['Linear']['StaNames'][i] for i in States_to_include], 
                                              outputs = TestEnv.TrimData['Linear']['OutNames'],
                                              name = 'Aircraft' )
    
    SaveAircraft.append(Aircraft)
    # %% 
    EngActuator     = gen_EngActuator(wn_radps = 40,  inp_name = 'ThrottleCmd_u', out_name = 'Throttle_u' , act_name = 'EngActuator')
    Sensor_vz       = gen_Sensor(wn_radps = 40, inp_name = 'VZ_mps', out_name = 'VZ_sen_mps' , sensor_name = 'Sensor_vz')
    Sensor_z        = gen_Sensor(wn_radps = 40, inp_name = 'Z_m', out_name = 'Z_sen_m' , sensor_name = 'Sensor_z')
    Sensor_az       = gen_Sensor(wn_radps = 40, inp_name = 'AZi_mps2', out_name = 'AZi_sen_mps2' , sensor_name = 'Sensor_az')
    # %%
       
    TotalCost = GeneralFunction()
    
    if n_trim == 0:
        x0 = [-0.10, -0.00, 1.0, 0.0, 0.0]
    else:
        x0 = OptGains['x']
        
    bnds = ((-2.0, +0.0),
            (-0.1, +0.0),
            (+0.1, +5.0),
            (-0.1, +0.0),
            (-0.1, -0.0))
    
    optim_set = {'maxiter':100, 'disp': True}
    t = time.time()    
    OptGains = opt.minimize(GeneralFunction , x0, method = 'Powell' , bounds=bnds, options = optim_set)
    elapsed = time.time() - t
    print('Powell Elapsed Time [s]: %0.1f' %(elapsed))
   
    
    # SAVE GAINS
    GainsVec['Kvzp']   [n_trim] = OptGains['x'][0]
    GainsVec['Kvzi']   [n_trim] = OptGains['x'][1]
    GainsVec['Kz']     [n_trim] = OptGains['x'][2]
    GainsVec['Kvzff']  [n_trim] = OptGains['x'][3]
    GainsVec['Knzp']   [n_trim] = OptGains['x'][4]   

    Gains = {}
    for kk in GainsVec.keys():
        Gains[kk] = GainsVec[kk][n_trim]
    
    Controller = gen_Controller(Gains)
    ClosedLoops = gen_ClosedLoops(Aircraft , Controller, Sensor_vz , Sensor_z, Sensor_az, EngActuator)
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
print("Start plotting ...")
for n_trim in range(len(TrimVec['VX_mps'])):
    Gains = {}
    for kk in GainsVec.keys():
        Gains[kk] = GainsVec[kk][n_trim]
    
    Controller  = gen_Controller(Gains)
    ClosedLoops = gen_ClosedLoops(SaveAircraft[n_trim] , Controller, Sensor_vz , Sensor_z, Sensor_az, EngActuator)
    Criteria    = CalculateIndividualCosts(ClosedLoops)
    TotalCost   = CalculateTotalCost(Criteria)
    
    print("Speed: %0.1f / TotalCost: %0.2f " %(TrimVec['VX_mps'][n_trim] , TotalCost))
    gen_Plots(ClosedLoops , Criteria, (str(TrimVec['VX_mps'][n_trim]) + 'm/s') )
    
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
with open('VertGains.pkl', 'wb') as fp:
    pickle.dump(File2Save, fp)
