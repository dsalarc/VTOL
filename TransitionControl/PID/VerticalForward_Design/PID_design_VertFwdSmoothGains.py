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

SavedGainsFile = 'SavedGains_VertFwdController_20240313_2235_Nelder-Mead.pkl'

# %% LOAD OLD FILE
with open(SavedGainsFile, 'rb') as fp:
    OLD = pickle.load(fp)

# %% GENERATE NEW FILE
NEW = copy.deepcopy(OLD)

for gain in OLD['GainsVec'].keys(): 
    NEW['GainsVec'][gain] = signal.savgol_filter(OLD['GainsVec'][gain],
                           3, # window size used for filtering
                           1) # order of fitted polynomial
    
SmoothList = [['Kvzi', 55, 0],
              ['Kvzi', 65, 0],
              ['Kvzff', 55, 0],
              ['Kvxp', 50, 0],
              ['Kvxp', 65, 0],
              ['Kvxp', 60, 0],
              ['Kz', 50, (45,70)],
              ['Kz', 55, (45,70)],
              ['Kz', 60, (45,70)],
              ['Kz', 65, (45,70)]]

for i in range(np.size(SmoothList, 0)):
    j = np.where(NEW['TrimVec']['VX_mps'] == SmoothList[i][1])[0][0]
    if SmoothList[i][2] == 0:
        x1 = j-1
        x2 = j+1
    else:
        x1 = np.where(NEW['TrimVec']['VX_mps'] == SmoothList[i][2][0])[0][0]
        x2 = np.where(NEW['TrimVec']['VX_mps'] == SmoothList[i][2][1])[0][0]
        
    NEW['GainsVec'][SmoothList[i][0]][j] = ((NEW['GainsVec'][SmoothList[i][0]][x2] - NEW['GainsVec'][SmoothList[i][0]][x1]) / 
                                           (NEW['TrimVec']['VX_mps'][x2] - NEW['TrimVec']['VX_mps'][x1]) * 
                                           (NEW['TrimVec']['VX_mps'][j] - NEW['TrimVec']['VX_mps'][x1]) + 
                                            NEW['GainsVec'][SmoothList[i][0]][x1])
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

