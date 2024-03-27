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

SavedGainsFile = 'SavedGains_VerticalController_20231229_1038_Nelder-Mead.pkl'

# %% LOAD OLD FILE
with open(SavedGainsFile, 'rb') as fp:
    OLD = pickle.load(fp)

# %% GENERATE NEW FILE
NEW = copy.deepcopy(OLD)

for gain in OLD['GainsVec'].keys(): 
    NEW['GainsVec'][gain] = signal.savgol_filter(OLD['GainsVec'][gain],
                           3, # window size used for filtering
                           1) # order of fitted polynomial
# %% PLOT COMPARISON
plt.close('all')

plt.figure()
l = np.floor(np.sqrt(len(OLD['GainsVec'].keys())))
c = np.ceil(len(OLD['GainsVec'].keys()) / l)
n = 1    

for kk in OLD['GainsVec'].keys():
   plt.subplot(l,c,n); n+= 1; plt.grid('on')
   plt.plot(OLD['TrimVec']['VX_mps'] , OLD['GainsVec'][kk], 'b-o' , label = 'Original')
   plt.plot(NEW['TrimVec']['VX_mps'] , NEW['GainsVec'][kk], 'r-o' , label = 'Smooth')
   plt.ylabel(kk)
   plt.xlabel('EAS [m/s]')
   plt.legend()

# %% SAVE NEW FILE
    
File2Save = {'GainsVec': NEW['GainsVec'], 'TrimVec': NEW['TrimVec'], 'CostVec': NEW['CostVec'], 'SaveTrim': NEW['SaveTrim']}

save_name = SavedGainsFile.replace('.pkl','_smooth.pkl')
with open(save_name, 'wb') as fp:
    pickle.dump(File2Save, fp)

