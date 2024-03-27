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
    
    return Aircraft, TestEnv.TrimData

def gen_EngActuator(wn_radps = 40, inp_name = 'inp_name', out_name = 'out_name' , act_name = 'sensor_name'):
    EngActuator = {}
    EngActuator['wn_radps']  = wn_radps
    EngActuator['SS'] = ct.ss(np.array([-EngActuator['wn_radps'] ]),
                               np.array([1]),
                               np.array([EngActuator['wn_radps'] ]),
                               np.array([0]) ,
                               inputs=inp_name, outputs=out_name, name = act_name)
    
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

def gen_Controller(Gains):
    Controller = {}
    Controller['SS'] = ct.ss(np.array([0])               ,  
                       np.array([[-1, -Gains['Kz'], +1, +Gains['Kz'], 0, 0]])   ,
                       np.array([[Gains['Kvzi']],
                                 [  0],
                                 [  1]])                , 
                       np.array([[-Gains['Kvzp'], -Gains['Kz']*(Gains['Kvzp']+Gains['Kvzff']), +Gains['Kvzp']+Gains['Kvzff'], +Gains['Kz']*(Gains['Kvzp']+Gains['Kvzff']), -Gains['Knzp'], 1],
                                 [   0          , -Gains['Kz']                               ,    1                         , +Gains['Kz']                               , 0             , 0],
                                 [   0          , 0                                          ,    0                         ,       0                                    , 0             , 0]]) , 
                       inputs = ['VZ_mps', 'Z_m', 'VZ_ref_mps', 'Z_ref_m', 'AZi_mps2', 'ThrottleCmd_inp'] , 
                       outputs = ['ThrottleCmd', 'VZ_cmd_mps', 'VZ_err_int_m'],
                       name = 'Controller' )
    return Controller


def gen_Sensor(wn_radps = 40, inp_name = 'inp_name', out_name = 'out_name' , sensor_name = 'sensor_name'):
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
