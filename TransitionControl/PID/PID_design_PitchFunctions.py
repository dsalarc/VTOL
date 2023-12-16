#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 21 22:53:18 2023

@author: dsalarc
"""
import numpy as np
import control as ct

def gen_Aircraft (TestEnv , VX_mps = 0, AX_mps2 = None, Elevator_deg = None):
    TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, AX_mps2 = AX_mps2, Elevator_deg = Elevator_deg, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)
    if (TestEnv.TrimData['Trimmed'] < 0.5) :
        if AX_mps2 < 0:
            if VX_mps < 31:
                print('Not trimmed with required AX. Trimming with Tilt_deg = 90')
                TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, Tilt_deg = 90, Elevator_deg = Elevator_deg, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)
            else:
                print('Not trimmed with required AX. Trimming with Throttle = 0')
                TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, Throttle_u = -1.0, Elevator_deg = Elevator_deg, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0)
                
    if (TestEnv.TrimData['Trimmed'] < 0.5):
        print('VX[mps]: %0.0f, AX[mps2]: %0.1f : Not trimmed' %(VX_mps , AX_mps2))
    else:
        print('VX[mps]: %0.0f, AX[mps2]: %0.1f : Trimmed' %(VX_mps , AX_mps2))
  
    # %%
    Aircraft = {}
    Aircraft['AltitudeIncluded'] = {}
    Aircraft['AltitudeIncluded']['SS'] = ct.ss(TestEnv.TrimData['Linear']['A'] , 
                                              TestEnv.TrimData['Linear']['B'] , 
                                              TestEnv.TrimData['Linear']['C'] , 
                                              TestEnv.TrimData['Linear']['D'] , 
                                              inputs=TestEnv.TrimData['Linear']['InpNames'] , 
                                              states=TestEnv.TrimData['Linear']['StaNames'] , 
                                              outputs = TestEnv.TrimData['Linear']['OutNames'],
                                              name = 'Aircraft' )
    
    Aircraft['AltitudeNotIncluded'] = {}
    idx_Z = TestEnv.TrimData['Linear']['StaNames'].index('Z_m')
    idx   = np.arange(0,len(TestEnv.TrimData['Linear']['StaNames']))
    idx   = np.delete(idx,idx_Z)
    Aircraft['AltitudeNotIncluded']['SS'] = ct.ss(TestEnv.TrimData['Linear']['A'][:,idx][idx,:] , 
                                              TestEnv.TrimData['Linear']['B'][idx,:] , 
                                              TestEnv.TrimData['Linear']['C'][:,idx]  , 
                                              TestEnv.TrimData['Linear']['D'] , 
                                              inputs=TestEnv.TrimData['Linear']['InpNames'] , 
                                              states=[TestEnv.TrimData['Linear']['StaNames'][index] for index in idx], 
                                              outputs = TestEnv.TrimData['Linear']['OutNames'],
                                              name = 'Aircraft' )
    
    return Aircraft, TestEnv.TrimData
   
def gen_EngActuator(wn_radps = 40):
    EngActuator = {}
    EngActuator['wn_radps']  = wn_radps
    EngActuator['SS'] = ct.ss(np.array([-EngActuator['wn_radps'] ]),
                               np.array([1]),
                               np.array([EngActuator['wn_radps'] ]),
                               np.array([0]) ,
                               inputs='PitchCmd_u', outputs='PitchThrottle_u', name = 'EngActuator')
    
    return EngActuator

def gen_ElevActuator(wn_radps = 40):
    ElevActuator = {}
    ElevActuator['wn_radps']  = wn_radps
    ElevActuator['SS'] = ct.ss(np.array([-ElevActuator['wn_radps'] ]),
                               np.array([1]),
                               np.array([ElevActuator['wn_radps'] ]),
                               np.array([0]) ,
                               inputs='ElevatorCmd_u', outputs='Elevator_u', name = 'ElevActuator')
    
    return ElevActuator

def Controller(Gains):
    Controller = {}
    Controller['SS'] = ct.ss(np.array([0])               ,  
                       np.array([[-1, -Gains['Kt'], +1, +Gains['Kt'], 0]])   ,
                       np.array([[Gains['Kqi']],
                                 [  0],
                                 [  1]])                , 
                       np.array([[-Gains['Kqp'], -Gains['Kt']*(Gains['Kqp']+Gains['Kff']), +Gains['Kqp']+Gains['Kff'], +Gains['Kt']*(Gains['Kqp']+Gains['Kff']), 1],
                                 [   0,     -Gains['Kt']      ,    1    ,     +Gains['Kt']      , 0],
                                 [   0,       0      ,    0    ,       0      , 0]]) , 
                       inputs = ['Q_degps', 'Theta_deg', 'Q_ref_degps', 'Theta_ref_deg', 'PitchCmd_inp'] , 
                       outputs = ['PitchCmd', 'Q_cmd_degps', 'Q_err_int_deg'],
                       name = 'Control' )
    return Controller

def gen_ControlAllocation(Gains):
    ControlAllocation = {}
    ControlAllocation['SS'] = ct.ss(np.array([0]),  
                                   np.array([0]),
                                   np.array([[0],
                                             [0]]) , 
                                   np.array([[    Gains['ContAlloc_Thr']],
                                             [-(1 - Gains['ContAlloc_Thr'])]]) , 
                                   inputs = ['PitchCmd'] , 
                                   outputs = ['ThrottlePitch', 'Elevator_u'],
                                   name = 'ControlAllocation' )
    return ControlAllocation

def Sensor(wn_radps = 40, inp_name = 'inp_name', out_name = 'out_name' , sensor_name = 'sensor_name'):
    Sensor = {}
    Sensor['wn_radps']  = wn_radps
    Sensor['TF'] = ct.tf([1],[1/Sensor['wn_radps'] , 1 ])
    Sensor['SS'] = ct.ss(np.array([-Sensor['wn_radps'] ]),
                           np.array([1]),
                           np.array([Sensor['wn_radps'] ]),
                           np.array([0]) ,
                           inputs=inp_name, outputs=out_name, name = sensor_name)
    
    return Sensor

def gen_Gain(Gain_val = 1, inp_name = 'inp_name', out_name = 'out_name' , gain_name = 'gain_name'):
    Gain = {}
    Gain['Gain']  = Gain_val
    Gain['TF'] = ct.tf([Gain_val],[1])
    Gain['SS'] = ct.ss(np.array([0]),
                       np.array([0]),
                       np.array([0]),
                       np.array([Gain_val]) ,
                       inputs=inp_name, outputs=out_name, name = gain_name)
    
    return Gain
