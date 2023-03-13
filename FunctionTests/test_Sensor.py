#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 21:31:10 2023

@author: dsalarc
"""

import numpy as np
import matplotlib.pyplot as plt

class LowPassDiscreteFilter:
    # https://x-engineer.org/discretizing-transfer-function/
    
    def __init__(self, wc, time_sample_filter, time_sample_sim = -1, order = 1, DiscType = 'euler_back'):
        self.Tc = 1/wc
        self.Ts = time_sample_filter
        self.y   = 0
        self.ym1 = 0
        self.ym2 = 0
        self.u   = 0
        self.um1 = 0
        self.um2 = 0
        self.order = order
        self.DiscType = DiscType
        if time_sample_sim == -1:
            time_sample_sim = time_sample_filter
            
        if np.mod(time_sample_sim, time_sample_filter) > time_sample_filter/1000:
            raise('Simulation time sample shall me a multiple of filter time sample')
        else:
            self.Tscale = int(np.round(time_sample_sim / time_sample_filter))

    
    def set_zero(self,zero):
        self.y   = zero
        self.ym1 = zero
        self.ym2 = zero
        self.u   = zero
        self.um1 = zero
        self.um2 = zero
        
    def step(self,u):
        
        self.um1 = self.u
        self.um2 = self.um1
        self.u   = u
        
        for i in range(self.Tscale):
            if self.Tscale == 1:
                u_step = self.u
            else:
                u_step = self.um1 * i/(self.Tscale-1) + self.u*(self.Tscale-i-1)/(self.Tscale-1)
                       
            self.ym1 = self.y
            self.ym2 = self.ym1
            
            self.u = u
                
            if self.DiscType == 'tustin':
                self.y = 1 / (2 * self.Tc + self.Ts) * (self.Ts * (u_step + self.um1) - (self.Ts-2*self.Tc) * self.ym1)
            
            elif self.DiscType == 'euler_fwd':
                self.y = 1 / (self.Tc + self.Ts) * (self.Ts * self.um1 + self.Tc * self.ym1)
    
            elif self.DiscType == 'euler_back':
                self.y = 1 / (self.Tc + self.Ts) * (self.Ts * u_step + self.Tc * self.ym1)
           
            else:
                self.y = 0
            
        return self.y
    
class Sensor:
    def __init__(self, CutFreq_radps = 40, Delay_s = 0, time_sample_sensor = 0.001, time_sample_sim = -1):
        
        self.DelaySteps = int(Delay_s / time_sample_sensor)
        if self.DelaySteps == 0:
            self.BufferDelay = np.array([])
        else:
            self.BufferDelay = np.zeros(self.DelaySteps)
            
        if time_sample_sim == -1:
            time_sample_sim = time_sample_sensor
        # self.tsa = time_sample_actuator
        self.tss = time_sample_sim
        
        self.y = 0
        
        self.SensFilter = LowPassDiscreteFilter(wc = CutFreq_radps,
                                               time_sample_filter = time_sample_sensor,
                                               time_sample_sim = time_sample_sim,
                                               order = 1,
                                               DiscType = 'euler_back')
        
    def set_zero(self,zero):
        self.y    = zero
        self.BufferDelay[:]  = zero
        self.SensFilter.set_zero(zero)
        
    def step(self,u):
        
        yaux = self.SensFilter.step(u)
        
        if self.DelaySteps == 0:
            self.y = yaux
        else:
            self.y = self.BufferDelay[0]
            self.BufferDelay[0:-1] = self.BufferDelay[1:]
            self.BufferDelay[-1] = yaux
        
        return self.y
        
# %%  
t_sim = 0.05
t_act = 0.001
wc = 40
rate = 20

u_w = 0
u_d = 2
u_a = 4
u_ts = 0.2
y0 = 10

t_sim = 0.01
t_filt = 0.001

SENS = Sensor(CutFreq_radps = wc, Delay_s = 0, time_sample_sensor = t_act, time_sample_sim = t_sim)

SENS.set_zero(y0)

t_vec = np.arange(0,2,step=t_sim)
u_vec = y0 + u_a*np.sin(u_w * t_vec)
u_vec[t_vec>=u_ts] = u_vec[t_vec>=u_ts] + u_d
y_vec = np.zeros(np.shape(t_vec))
v_vec = np.zeros(np.shape(t_vec))

y_vec[0] = y0

for i in range(1,len(t_vec)):
    
    y_vec[i] = SENS.step(u_vec[i])
    
XLIM = (0,1)

plt.figure()
plt.subplot(2,1,1)
plt.plot(t_vec, u_vec, 'b')
plt.plot(t_vec, y_vec, 'r', label = 'sensor', markersize = 4)
plt.legend()
plt.xlim(XLIM)
    
                
                