#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 21:41:55 2023

@author: dsalarc
"""
import gym
import pickle
import numpy as np
import matplotlib.pyplot as plt

env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]

with open('SavedGains_VertFwdController_20240313_2235_Nelder-Mead_smooth.pkl', 'rb') as fp:
    NelderMead = pickle.load(fp)
# %%  fdsgfds
plt.figure()

n=0
plt.subplot(2,3,n+1)
plt.title('AX: ' + str(TNC['TrimVec']['AX_mps2'][n]))
plt.plot(TNC['TrimVec']['VX_mps'] , TNC['CostVec'][:,n], label = 'TNC')
# plt.plot(SLSQP['TrimVec']['VX_mps'] , SLSQP['CostVec'][:,n], label = 'SLSQP')
plt.plot(NelderMead['TrimVec']['VX_mps'] , NelderMead['CostVec'][:,n], label = 'NelderMead')
plt.plot(Powell['TrimVec']['VX_mps'] , Powell['CostVec'][:,n], label = 'Powell')
plt.plot(LBFGS['TrimVec']['VX_mps'] , LBFGS['CostVec'][:,n], label = 'LBFGS')

n=1
plt.subplot(2,3,n+1)
plt.title('AX: ' + str(TNC['TrimVec']['AX_mps2'][n]))
plt.plot(TNC['TrimVec']['VX_mps'] , TNC['CostVec'][:,n], label = 'TNC')
# plt.plot(SLSQP['TrimVec']['VX_mps'] , SLSQP['CostVec'][:,n], label = 'SLSQP')
plt.plot(NelderMead['TrimVec']['VX_mps'] , NelderMead['CostVec'][:,n], label = 'NelderMead')
plt.plot(Powell['TrimVec']['VX_mps'] , Powell['CostVec'][:,n], label = 'Powell')
plt.plot(LBFGS['TrimVec']['VX_mps'] , LBFGS['CostVec'][:,n], label = 'LBFGS')

n=2
plt.subplot(2,3,n+1)
plt.title('AX: ' + str(TNC['TrimVec']['AX_mps2'][n]))
plt.plot(TNC['TrimVec']['VX_mps'] , TNC['CostVec'][:,n], label = 'TNC')
# plt.plot(SLSQP['TrimVec']['VX_mps'] , SLSQP['CostVec'][:,n], label = 'SLSQP')
plt.plot(NelderMead['TrimVec']['VX_mps'] , NelderMead['CostVec'][:,n], label = 'NelderMead')
plt.plot(Powell['TrimVec']['VX_mps'] , Powell['CostVec'][:,n], label = 'Powell')
plt.plot(LBFGS['TrimVec']['VX_mps'] , LBFGS['CostVec'][:,n], label = 'LBFGS')

n=3
plt.subplot(2,3,n+1)
plt.title('AX: ' + str(TNC['TrimVec']['AX_mps2'][n]))
plt.plot(TNC['TrimVec']['VX_mps'] , TNC['CostVec'][:,n], label = 'TNC')
# plt.plot(SLSQP['TrimVec']['VX_mps'] , SLSQP['CostVec'][:,n], label = 'SLSQP')
plt.plot(NelderMead['TrimVec']['VX_mps'] , NelderMead['CostVec'][:,n], label = 'NelderMead')
plt.plot(Powell['TrimVec']['VX_mps'] , Powell['CostVec'][:,n], label = 'Powell')
plt.plot(LBFGS['TrimVec']['VX_mps'] , LBFGS['CostVec'][:,n], label = 'LBFGS')

n=4
plt.subplot(2,3,n+1)
plt.title('AX: ' + str(TNC['TrimVec']['AX_mps2'][n]))
plt.plot(TNC['TrimVec']['VX_mps'] , TNC['CostVec'][:,n], label = 'TNC')
# plt.plot(SLSQP['TrimVec']['VX_mps'] , SLSQP['CostVec'][:,n], label = 'SLSQP')
plt.plot(NelderMead['TrimVec']['VX_mps'] , NelderMead['CostVec'][:,n], label = 'NelderMead')
plt.plot(Powell['TrimVec']['VX_mps'] , Powell['CostVec'][:,n], label = 'Powell')
plt.plot(LBFGS['TrimVec']['VX_mps'] , LBFGS['CostVec'][:,n], label = 'LBFGS')
plt.legend()
    
# %%  GAINS
Select = NelderMead.copy()
plt.figure()

plt.subplot(2,3,1)
for i in range(5):
    plt.plot(Select['TrimVec']['VX_mps'] , Select['GainsVec']['Kqp'][:,i], label = 'AX: ' + str(TNC['TrimVec']['AX_mps2'][i]))
plt.ylabel('Kqp')

plt.subplot(2,3,2)
for i in range(5):
    plt.plot(Select['TrimVec']['VX_mps'] , Select['GainsVec']['Kqi'][:,i], label = 'AX: ' + str(TNC['TrimVec']['AX_mps2'][i]))
plt.ylabel('Kqi')

plt.subplot(2,3,3)
for i in range(5):
    plt.plot(Select['TrimVec']['VX_mps'] , Select['GainsVec']['Kt'][:,i], label = 'AX: ' + str(TNC['TrimVec']['AX_mps2'][i]))
plt.ylabel('Kt')

plt.subplot(2,3,4)
for i in range(5):
    plt.plot(Select['TrimVec']['VX_mps'] , Select['GainsVec']['Kff'][:,i], label = 'AX: ' + str(TNC['TrimVec']['AX_mps2'][i]))
plt.ylabel('Kff')

plt.subplot(2,3,5)
for i in range(5):
    plt.plot(Select['TrimVec']['VX_mps'] , Select['CostVec'][:,i], label = 'AX: ' + str(TNC['TrimVec']['AX_mps2'][i]))
plt.ylabel('Cost')
plt.legend()

