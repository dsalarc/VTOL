#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 22:56:47 2021

@author: dsalarc
"""

from spinup import ppo_tf1 as ppo
import tensorflow as tf
import gym
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# %% SIM OPTIONS

SavePath = '/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/PPO/Save'
EnvName    = 'gym_VTOL:Vahana_VertFlight-v0'


# %% RUN TRAINING

env_fn = lambda : gym.make(EnvName)

ac_kwargs = dict(hidden_sizes=[64,64], activation=tf.nn.relu)

logger_kwargs = dict(output_dir=SavePath, exp_name='VTOL_v0')

ppo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=6000, epochs=50, 
    logger_kwargs=logger_kwargs, gamma = 1, max_ep_len=1200)

# ppo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=3000, epochs=50, 
#     logger_kwargs=logger_kwargs, gamma = 0.9995, max_ep_len=600)

# %% LOAD RESULTS AND RUN PLOT
Results = pd.read_csv(SavePath+ '/progress.txt', sep='\t')

# PID_result = 2633.4

fig = plt.figure()
plt.plot(Results['Epoch'].to_numpy(),Results['AverageEpRet'].to_numpy(),linewidth=2,color='red',label='AvgReturn')
plt.plot(Results['Epoch'].to_numpy(),Results['MaxEpRet'].to_numpy(),'--',linewidth=0.5,color='red',label='MaxEpRet')
plt.plot(Results['Epoch'].to_numpy(),Results['MinEpRet'].to_numpy(),'-.',linewidth=0.5,color='red',label='MinEpRet')   
# plt.plot(np.array([0,Results['Epoch'].to_numpy()[-1]+1]),np.array([PID_result,PID_result]),'k--',linewidth=2,label='PD Baseline')
plt.grid('on')    
plt.xlim([0,Results['Epoch'].to_numpy()[-1]+1])    

plt.xlabel('Epoch')
plt.ylabel('Return')
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',)
plt.show()

fig.savefig(SavePath + '/' + 'PPO_DefaultPar_Epochs.svg', bbox_inches='tight', format='svg')
fig.savefig(SavePath + '/' + 'PPO_DefaultPar_Epochs.png', bbox_inches='tight', format='png')

