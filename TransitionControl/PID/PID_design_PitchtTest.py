#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 21:35:32 2023

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

plt.close('all')

with open('SavedGains_final.pkl', 'rb') as fp:
    std = pickle.load(fp)
    
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')

VX_mps_vec = np.array([0, 10, 20, 30, 40, 50, 60])
VX_mps_vec = np.array([0, 10])


for nv in range(len(VX_mps_vec)):
    VX_mps = VX_mps_vec[nv]
    
    Gains = {}
    for kk in std['GainsVec'].keys():
        Gains[kk] = np.interp(VX_mps, std['TrimVec']['VX_mps'], std['GainsVec'][kk])
        
    # %%LOAD MODEL
    
    # %% GENERATE CLOSED LOOPS
    Aircraft        = gen_Aircraft(TestEnv, VX_mps)
    EngActuator     = gen_EngActuator(wn_radps = 40)
    ElevActuator    = gen_ElevActuator(wn_radps = 40)
    Sensor_q        = Sensor(wn_radps = 40, inp_name = 'Q_degps', out_name = 'Q_sen_degps' , sensor_name = 'Sensor_q')
    Sensor_t        = Sensor(wn_radps = 40, inp_name = 'Theta_deg', out_name = 'Theta_sen_deg' , sensor_name = 'Sensor_t')
    
    PitchController = Controller(Gains)
    ControlAllocation = gen_ControlAllocation(Gains)
    ClosedLoops = PitchClosedLoops(Aircraft , PitchController, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation)
    Criteria    = CalculateIndividualCosts(ClosedLoops)
    TotalCost   = CalculateTotalCost(Criteria)
    
    PitchPlots(ClosedLoops , Criteria, (str(VX_mps) + 'm/s') )
