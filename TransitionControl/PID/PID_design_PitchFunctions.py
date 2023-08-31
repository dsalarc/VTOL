#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 21 22:53:18 2023

@author: dsalarc
"""
import numpy as np
import control as ct

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
