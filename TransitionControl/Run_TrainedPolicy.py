#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 19 18:45:06 2021

@author: dsalarc
"""

from spinup.utils.test_policy import load_policy_and_env, run_policy
import tensorflow as tf
import gym 
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import shutil
from AuxFunctions import SaveSelection

# %% SIM OPTIONS

CaseList = np.array([['01'  , '01' , 's0' , 'NNarchitecture_Full_01', '_gam0-95_hid32-32','last'],
                     ['02'  , '02' , 's0' , 'NNarchitecture_Full_01', '_gam0-95_hid32-32','last'],
                     ['03'  , '03' , 's0' , 'NNarchitecture_Full_01', '_gam0-95_hid32-32','last'],
                     ['04'  , '04' , 's0' , 'NNarchitecture_Full_01', '_gam0-95_hid32-32','last'],
                     ['05'  , '05' , 's0' , 'NNarchitecture_Full_01', '_gam0-99_hid32-32','last'],
                     ['06'  , '06' , 's0' , 'NNarchitecture_Full_01', '_gam0-95_hid32-32','last'],
                     ['07'  , '07' , 's0' , 'NNarchitecture_Full_01', '_gam0-95'         , 4000 ],
                     ['08'  , '08' , 's0' , 'NNarchitecture_Full_01', '_gam0-95_hid16-16','last'],
                     ['12'  , '12' , 's0' , 'Final_01'              , ''                 ,1000],
                     ['13'  , '13' , 's0' , 'Test_02'               , ''                 ,2000],
                     ['14'  , '14' , 's0' , 'Test_03'               , ''                 ,'last'],
                     ['15'  , '15' , 's0' , 'Test_01'               , '_gam0-99_hid64-64','last'],
                     ['16'  , '16' , 's0' , 'Test_01'               , '_gam0-95_hid64-64','last'],
                     ['17'  , '17' , 's0' , 'Test_01'               , '_gam0-95','last'],
                     ['18a' , '18' , 's0' , 'Test_01'               , '_gam0-99','last'],
                     ['18b' , '18' , 's0' , 'Test_02'               , '','last'],
                     ['19'  , '18' , 's0' , 'Test_02'               , '_gam0-99_max2000_hid64-64_selu','last'],
                     ['20a' , '20' , 's0' , 'Test_01'               , '_hid16-16_tanh','last'],
                     ['20b' , '20' , 's0' , 'Test_01'               , '_hid128-128_tanh','last'],
                     ['20c' , '20' , 's0' , 'Test_01'               , '_hid128-128_selu','last'],
                     ['21a' , '21' , 's0' , 'Test_02'               , '_hid64-64_selu','last'],
                     ['21b' , '21' , 's0' , 'Test_02'               , '_hid128-128_selu','last'],
                     ['22a' , '22' , 's0' , 'Test_01'               , '_hid16-16','last'],
                     ['22b' , '22' , 's0' , 'Test_01'               , '_hid64-64','last'],
                     ['22c' , '22' , 's0' , 'Test_02'               , '','last'],
                     ['22d' , '22' , 's0' , 'Test_03'               , '','last'],
                     ['22e' , '22' , 's0' , 'Test_04'               , '',1000],
                     ['22f' , '22' , 's0' , 'Test_05'               , '','last'],
                     ['22g' , '22' , 's0' , 'Test_07'               , '',3000],
                     ['22h' , '22' , 's0' , 'Test_08'               , '','last'],
                     ['22i' , '22' , 's0' , 'Test_09'               , '','last'],
                     ['23'  , '23' , 's0' , 'Test_01'               , '','last'],
                     ['101a', '101', 's0' , 'Test_01'               , '_gam0-99_hid64-64_selu','last'],
                     ['101b', '101', 's0' , 'Test_01'               , '_gam0-99_hid32-32_tanh','last'],
                     ['102' , '102', 's0' , 'Test_01'               , '','last'],
                     ['103' , '103', 's0' , 'Test_01'               , '','last'],
                     ['104' , '104', 's0' , 'Test_01'               , '','last'],
                     ['105' , '105', 's0' , 'Test_01'               , '','last'],
                     ['106' , '106', 's0' , 'Test_01'               , '','last'],
                     ['107' , '107', 's0' , 'Test_01'               , '','last'],
                     ['108' , '108', 's0' , 'Test_01'               , '','last'],
                     ['109' , '109', 's0' , 'Test_01'               , '','last'],
                     ['201' , '201', 's0' , 'Test_01'               , '','last'],
                     ['301' , '301', 's0' , 'Test_01'               , '','last'],
                     ['302' , '302', 's0' , 'Test_01'               , '',4000],
                     ['303' , '303', 's0' , 'Test_01'               , '','last'],
                     ['304' , '304', 's0' , 'Test_01'               , '','last'],
                     ['401' , '401', 's0' , 'Test_01'               , '','last'],
                     ['402' , '402', 's0' , 'Test_01'               , '','last'],
                     ['501' , '501', 's0' , 'Test_01'               , '','last'],
                     ['502' , '502', 's0' , 'Test_01'               , '',4000],
                     ['504' , '504', 's0' , 'Test_01'               , '',5000],
                     ['505' , '505', 's0' , 'Test_01'               , '',4000],
                     ['506' , '506', 's2' , 'Test_01'               , '','last'],
                     ['511' , '511', 's4' , 'Test_01'               , '','last'],
                     ['521' , '521', 's1' , 'Test_01'               , '','last'],
                     ['531' , '531', 's4' , 'Test_01'               , '','last'],
                     ['541' , '541', 's2' , 'Test_01'               , '','last'],
                     ['551' , '551', 's2' , 'Test_01'               , '','last'],
                     ['601' , '601', 's0' , 'Test_01'               , '','last'],
                     ['603' , '603', 's5' , 'Test_01'               , '','last'],
                     ['701' , '701', 's5' , 'Test_01'               , '','last'],
                     ['xxx' , 'xxx', 's0' , 'Test_01'               , '','last']])

CaseNumber = '701'
INP = {}
INP['WIND_TurbON'] = 0
INP['WIND_TowerX_mps'] = 0
GridExper  = CaseList[np.where(CaseList[:,0]==CaseNumber)[0][0],1]
Seed       = CaseList[np.where(CaseList[:,0]==CaseNumber)[0][0],2]
NNFolder   = CaseList[np.where(CaseList[:,0]==CaseNumber)[0][0],3]
NNname     = CaseList[np.where(CaseList[:,0]==CaseNumber)[0][0],4]
train_iter = CaseList[np.where(CaseList[:,0]==CaseNumber)[0][0],5]
train_iter = CaseList[np.where(CaseList[:,0]==CaseNumber)[0][0],5]

Agent_Path = ('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PPO/GridExperiments_v' 
              + GridExper + '/' + NNFolder + '/ppo-tf-bench_gym_vtol-vahana_vertflight-v0' + NNname + '/ppo-tf-bench_gym_vtol-vahana_vertflight-v0' + NNname + '_' + Seed)

# Agent_Path = ('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/WTPPcontrol/PPO/GridExperiments_v' + str(CaseNumber) + '/Final_01/ppo-tf-bench_gym_vtol-vahana_vertflight-v0/ppo-tf-bench_gym_vtol-vahana_vertflight-v0_s0')

# shutil.copy2('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PPO/VTOL_env_'+ str(CaseNumber) +'.py','/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/gym-VTOL/gym_VTOL/envs'+ '/VTOL_env.py')

Agent_Type = 'PPO'
TrainType  = 'NN_01'
try:
    train_iter = int(train_iter)
except:
    pass
ActionNum  = 1


# TestType = 'VariableTarget'

EnvName    = 'gym_VTOL:Vahana_VertFlight-v0'
TrimTime   = 0
SimTime    = 30

VZ_Ref = np.array([[0 , 5 , 100 , 1000  ],
                [0 , 0  , 0 ,  0    ]])


The_Ref = np.array([[0 , 1000  ],
                    [0 , 0     ]])

Phi_Ref = np.array([[0 , 1000  ],
                    [0 , 0     ]])

Psi_Ref = np.array([[0 , 1000  ],
                    [0 , 0     ]])

reset_INPUT_VEC = {}
reset_INPUT_VEC['WIND_TowerX_mps'] = np.array([[0                      , 15                      , 20                      , 30  ],
                                               [INP['WIND_TowerX_mps'] , INP['WIND_TowerX_mps']  , INP['WIND_TowerX_mps']  , INP['WIND_TowerX_mps']   ]])
reset_INPUT_VEC['WIND_TurbON']     = np.array([[0                  , 15                  , 20                  , 30  ],
                                               [INP['WIND_TurbON'] , INP['WIND_TurbON']  , INP['WIND_TurbON']  , INP['WIND_TurbON']   ]])

def MovingAverage(arr , window_size = 1):
      
    # Initialize an empty list to store moving averages
    moving_averages = np.zeros(len(arr))
      
    # Loop through the array to consider
    # every window of size 3
    for i in range(len(arr)):
        
        # Store elements from i to i+window_size
        # in list to get the current window
        if i < window_size:
            window = arr[0 : i + window_size]
        elif len(arr)-1 - i < window_size:
             window = arr[i - window_size:]
        else:
             window = arr[i - window_size : i + window_size]
     
        # Calculate the average of current window        
        moving_averages[i] = np.sum(window) / len(window)
          
    
    return moving_averages

# %% INITIALIZE

_, GA = load_policy_and_env(Agent_Path,deterministic=True,itr=train_iter)


# env_dict = gym.envs.registration.registry.env_specs.copy()
# for env in env_dict:
#     if 'Vahana_VertFlight-v0' in env:
#         print("Remove {} from registry".format(env))
#         del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make(EnvName)

# print(TestEnv.observation_space)
# print(TestEnv.action_space)
# print(TestEnv.action_space.sample())

# %% RESET INVIRONMENT

obs = TestEnv.reset(TermTheta_deg = 45, reset_INPUT_VEC = reset_INPUT_VEC, Training_Turb = False, Training_WindX = False)
# obs = TestEnv.reset(TermTheta_deg = 45)

reward = 0
SaveVec = {}
SaveVec = SaveSelection(SaveVec,TestEnv.info)

AllStates = obs
AllActions = np.ones(len(TestEnv.action_names))
TimeVec = np.arange(0,SimTime,TestEnv.t_step)
if abs(TimeVec[-1] - SimTime) > (TestEnv.t_step/2): TimeVec = np.append(TimeVec,SimTime)
n_steps = np.size(TimeVec)
info = []

Return = 0
MaxStep = n_steps

# % RUN SIM
print("  ")
print("  ")
try:
    print("Running case " + CaseNumber + ", Iter " + str(train_iter))
except:
    print("Running case " + CaseNumber + ", Iter " + train_iter)

for step in range(np.size(TimeVec)-1):
  
  if TimeVec[step]<= TrimTime:
      inp_act = TestEnv.TrimData['Action']
  else:
      obs_tgt = obs.copy()
      
      for i in range(ActionNum):
          act = GA(obs_tgt)  
          # act[2] = 0
          # act[3] = 0
          if i==0:
                  act_vec = act
          else:
                  act_vec = np.vstack((act_vec,act))
                  
      if ActionNum == 1:
          inp_act = act_vec
      else:
          # inp_act = np.array([np.mean(act_vec,0)])
          inp_act = np.mean(act_vec,0)

  AllActions = np.vstack((AllActions,inp_act))
  obs, reward, done, info = TestEnv.step(inp_act)
  AllStates = np.vstack((AllStates,obs))
  SaveVec = SaveSelection(SaveVec,TestEnv.info)

  Return = Return + reward

  if done:
    print("Goal reached!", "reward=", reward)
    MaxStep = step
    break

    
print(' ')
print('Return RL = {:0.1f}'.format(Return))
 
print(' ')
print('Max Alt  : {:0.1f}'.format(np.max(SaveVec['H_m'])))
print('Min Alt  : {:0.1f}'.format(np.min(SaveVec['H_m'])))
print('Max VZ   : {:0.1f}'.format(np.max(SaveVec['VZ_mps'])))
print('Min VZ   : {:0.1f}'.format(np.min(SaveVec['VZ_mps'])))
print('Max Pitch: {:0.1f}'.format(np.max(SaveVec['Theta_deg'])))
print('Min Pitch: {:0.1f}'.format(np.min(SaveVec['Theta_deg'])))
print('Max Q    : {:0.1f}'.format(np.max(SaveVec['Q_degps'])))
print('Min Q    : {:0.1f}'.format(np.min(SaveVec['Q_degps'])))
print('Max Vx   : {:0.1f}'.format(np.max(SaveVec['VX_mps'])))
print(' ')

# exec(open("./Test_VTOL_GeneralPlot_TrainedPolicy.py").read())
# exec(open("./Test_VTOL_GeneralPlot.py").read())
exec(open("./Test_VTOL_PIDPlot3.py").read())
try:
    fig.savefig(Agent_Path + '/Simulation_' + train_iter + '_' + Seed + '_Turb' + str(INP['WIND_TurbON']) + '_WindX' + str(INP['WIND_TowerX_mps']) + '_NumAct' + str(ActionNum) + '.pdf', bbox_inches='tight')
except:
    fig.savefig(Agent_Path + '/Simulation_' + str(train_iter) + '_' + Seed + 'Turb' + str(INP['WIND_TurbON']) + '_WindX' + str(INP['WIND_TowerX_mps']) + '_NumAct' + str(ActionNum) + '.pdf', bbox_inches='tight')

print(' ')
print("AgentPath: " + Agent_Path) 

# %% CALCULATE AVERAGE CURRENT IN LAST 10s
TimeAvg_s = 10
avg_n = (np.where(TimeVec>(TimeVec[-1]-TimeAvg_s)))[0][0]
Consummed_1_Ah = SaveVec['Charge1_Ah'][-1] - SaveVec['Charge1_Ah'][avg_n]
Consummed_5_Ah = SaveVec['Charge5_Ah'][-1] - SaveVec['Charge5_Ah'][avg_n]

AverageCurrent_A = (Consummed_1_Ah + Consummed_5_Ah) / 2 *3600/TimeAvg_s
print(' ')
print("Average Current in last %0.1f sec: %0.1f" %(TimeAvg_s, AverageCurrent_A)) 


plt.show()

plt.figure()
plt.subplot(2,3,1)
plt.plot(TimeVec,AllActions[:,2],'k', linewidth = linewidth, label='Tilt 1 [u]')
plt.grid('on')
plt.ylabel('Tilt 1 [u]')
plt.subplot(2,3,2)
plt.plot(TimeVec,AllActions[:,3],'k', linewidth = linewidth, label='Tilt 2 [u]')
plt.grid('on')
plt.ylabel('Tilt 2 [u]')
plt.subplot(2,3,3)
plt.plot(TimeVec,AllActions[:,4],'k', linewidth = linewidth, label='Elev 2 [u]')
plt.grid('on')
plt.ylabel('Elev 2 [u]')
plt.subplot(2,3,4)
plt.plot(TimeVec,AllActions[:,0],'k', linewidth = linewidth, label='Throttle [u]')
plt.grid('on')
plt.ylabel('Throttle [u]')
plt.subplot(2,3,5)
plt.plot(TimeVec,AllActions[:,1],'k', linewidth = linewidth, label='Pitch Throttle [u]')
plt.grid('on')
plt.ylabel('Pitch Throttle [u]')
plt.show()
