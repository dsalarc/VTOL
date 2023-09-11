#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 21 22:53:18 2023

@author: dsalarc
"""
import numpy as np
import control as ct

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
