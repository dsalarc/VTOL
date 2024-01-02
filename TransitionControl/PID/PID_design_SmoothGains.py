#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 16 10:47:41 2023

@author: dsalarc
"""

import gym
import pickle
import numpy as np
import matplotlib.pyplot as plt
import copy
from scipy import signal

env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]

SavedGainsFile = 'SavedGains_20231222_0054_Nelder-Mead.pkl'

# %% LOAD OLD FILE
with open(SavedGainsFile, 'rb') as fp:
    OLD = pickle.load(fp)

# %% GENERATE NEW FILE
NEW = copy.deepcopy(OLD)

for gain in ['Kqp', 'Kqi', 'Kt', 'Kff']:
    for i in range(np.size(NEW['GainsVec']['Kqp'],0)):
        NEW['GainsVec'][gain][i,:] = np.median(OLD['GainsVec'][gain][i,:])
    
    
    smooth = signal.savgol_filter(NEW['GainsVec'][gain][:,2],
                           3, # window size used for filtering
                           1) # order of fitted polynomial
    NEW['GainsVec'][gain][:,:] = np.transpose(np.tile(smooth,(np.size(NEW['GainsVec'][gain],1),1)))
# %% PLOT COMPARISON
plt.close('all')

plt.figure()

plt.subplot(2,3,1)
for i in range(5):
    plt.plot(OLD['TrimVec']['VX_mps'] , OLD['GainsVec']['Kqp'][:,i], label = 'AX: ' + str(OLD['TrimVec']['AX_mps2'][i]))
plt.plot(NEW['TrimVec']['VX_mps'] , NEW['GainsVec']['Kqp'][:,2], 'k-', label = 'NEW')
plt.ylabel('Kqp')

plt.subplot(2,3,2)
for i in range(5):
    plt.plot(OLD['TrimVec']['VX_mps'] , OLD['GainsVec']['Kqi'][:,i], label = 'AX: ' + str(OLD['TrimVec']['AX_mps2'][i]))
plt.plot(NEW['TrimVec']['VX_mps'] , NEW['GainsVec']['Kqi'][:,2], 'k-', label = 'NEW')
plt.ylabel('Kqi')

plt.subplot(2,3,3)
for i in range(5):
    plt.plot(OLD['TrimVec']['VX_mps'] , OLD['GainsVec']['Kt'][:,i], label = 'AX: ' + str(OLD['TrimVec']['AX_mps2'][i]))
plt.plot(NEW['TrimVec']['VX_mps'] , NEW['GainsVec']['Kt'][:,2], 'k-', label = 'NEW')
plt.ylabel('Kt')

plt.subplot(2,3,4)
for i in range(5):
    plt.plot(OLD['TrimVec']['VX_mps'] , OLD['GainsVec']['Kff'][:,i], label = 'AX: ' + str(OLD['TrimVec']['AX_mps2'][i]))
plt.plot(NEW['TrimVec']['VX_mps'] , NEW['GainsVec']['Kff'][:,2], 'k-', label = 'NEW')
plt.ylabel('Kff')
plt.legend()

plt.subplot(2,3,5)
for i in range(5):
    plt.plot(OLD['TrimVec']['VX_mps'] , OLD['CostVec'][:,i], label = 'AX: ' + str(OLD['TrimVec']['AX_mps2'][i]))
plt.ylabel('Cost')
# %% SAVE NEW FILE
    
File2Save = {'GainsVec': NEW['GainsVec'], 'TrimVec': NEW['TrimVec'], 'CostVec': NEW['CostVec'], 'SaveTrim': NEW['SaveTrim']}

save_name = SavedGainsFile.replace('.pkl','_smooth.pkl')
with open(save_name, 'wb') as fp:
    pickle.dump(File2Save, fp)

