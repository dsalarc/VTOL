#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 18:29:29 2021

@author: dsalarc
"""

"""
from Env_SpringMassDiscrete import SpringMassDiscrete
import gym
import gym_SpringMass

TestEnv = SpringMassDiscrete()

obs = TestEnv.reset()
TestEnv.render()

print(TestEnv.observation_space)
print(TestEnv.action_space)
print(TestEnv.action_space.sample())

# Hardcoded best agent: always go left!
n_steps = 1000
for step in range(n_steps):
  #print("Step {}".format(step + 1))
  obs, reward, done, info = TestEnv.step(1)
  #print('obs=', obs, 'reward=', reward, 'done=', done)
  #TestEnv.render()
  if done:
    print("Goal reached!", "reward=", reward)
    break

TestEnv.render()
"""

import gym
import numpy as np
# import gym_SpringMass

TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')
# %%
obs = TestEnv.reset(np.array([0,0]))
TestEnv.render()

print(TestEnv.observation_space)
print(TestEnv.action_space)
print(TestEnv.action_space.sample())

# %
# Hardcoded best agent: always go left!
n_steps = 1000
for step in range(n_steps):
  #print("Step {}".format(step + 1))
  obs, reward, done, info = TestEnv.step(np.array([0]))
  #print('obs=', obs, 'reward=', reward, 'done=', done)
  #TestEnv.render()
  if done:
    print("Goal reached!", "reward=", reward)
    break

TestEnv.render()