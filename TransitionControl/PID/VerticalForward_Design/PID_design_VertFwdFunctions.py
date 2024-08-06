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
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0, Training_Turb = False, Training_WindX = False, 
                        Training_Trim = False)
    if (TestEnv.TrimData['Trimmed'] < 0.5) :
        if AX_mps2 < 0:
            if VX_mps < 31:
                print('Not trimmed with required AX. Trimming with Tilt_deg = 90')
                TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, Tilt_deg = 90, Elevator_deg = Elevator_deg, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0, Training_Turb = False, Training_WindX = False,
                        Training_Trim = False)
            else:
                print('Not trimmed with required AX. Trimming with Throttle = 0')
                TestEnv.reset(VX_mps = VX_mps, VZ_mps = 0.0, Throttle_u = -1.0, Elevator_deg = Elevator_deg, THETA = 0.0, DispMessages = False, Linearize = True,
                        TermTheta_deg = 45, StaFreezeList = [] , UNC_seed = None , UNC_enable = 0, Training_Turb = False, Training_WindX = False, 
                        Training_Trim = False)
                
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

def gen_Actuator(wn_radps = 40, inp_name = 'inp_name', out_name = 'out_name' , act_name = 'sensor_name'):
    Actuator = {}
    Actuator['wn_radps']  = wn_radps
    Actuator['SS'] = ct.ss(np.array([-Actuator['wn_radps'] ]),
                               np.array([1]),
                               np.array([Actuator['wn_radps'] ]),
                               np.array([0]) ,
                               inputs=inp_name, outputs=out_name, name = act_name)
    
    return Actuator


def gen_AltController(Gains):
    Controller = {}
    Controller['SS'] = ct.ss(np.array([0])               ,  
                       np.array([[-1, -Gains['Kz'], +1, +Gains['Kz'], 0, 0]])   ,
                       np.array([[Gains['Kvzi']],
                                 [  0],
                                 [  1]])                , 
                       np.array([[-Gains['Kvzp'], -Gains['Kz']*(Gains['Kvzp']+Gains['Kvzff']), +Gains['Kvzp']+Gains['Kvzff'], +Gains['Kz']*(Gains['Kvzp']+Gains['Kvzff']), -Gains['Knzp'], 1],
                                 [   0          , -Gains['Kz']                               ,    1                         , +Gains['Kz']                               , 0             , 0],
                                 [   0          , 0                                          ,    0                         ,       0                                    , 0             , 0]]) , 
                       inputs = ['VZ_mps', 'Z_m', 'VZ_ref_mps', 'Z_ref_m', 'AZi_mps2', 'ThetaCmd_inp'] , 
                       outputs = ['ThetaCmd_deg', 'VZ_cmd_mps', 'VZ_err_int_m'],
                       name = 'AltController' )
    return Controller

def gen_SpeedController(Gains):
    Controller = {}
    Controller['SS'] = ct.ss(np.array([0])          ,  
                       np.array([[0 , -1, +1, 0]])  ,
                       np.array([[Gains['Kvxi']],
                                 [  0          ],
                                 [  1          ]])   , 
                       np.array([[-Gains['Kaxp'], -Gains['Kvxp'], +Gains['Kvxp'], 1],
                                 [   0          , -1            ,    1          , 0],
                                 [   0          , 0             ,    0          , 0]]) , 
                       inputs = ['AXi_mps2', 'VX_mps', 'VX_ref_mps', 'ThrottleCmd_inp'] , 
                       outputs = ['ThrottleCmd_u', 'VX_err_mps', 'VX_err_int_m'],
                       name = 'SpeedController' )
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
