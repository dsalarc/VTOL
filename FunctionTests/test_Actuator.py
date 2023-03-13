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
    
class Actuator:
    def __init__(self, CutFreq_radps = 40, MaxRate = 20, time_sample_actuator = 0.001, time_sample_sim = -1):
        
        self.MaxStepChange = MaxRate*time_sample_sim
        
        if time_sample_sim == -1:
            time_sample_sim = time_sample_actuator
        # self.tsa = time_sample_actuator
        self.tss = time_sample_sim
        
        self.y = 0
        self.v = 0
        self.a = 0
        
        self.ActFilter = LowPassDiscreteFilter(wc = CutFreq_radps,
                                               time_sample_filter = time_sample_actuator,
                                               time_sample_sim = time_sample_sim,
                                               order = 1,
                                               DiscType = 'euler_back')
        
    def set_zero(self,zero):
        self.y    = zero
        self.ActFilter.set_zero(zero)
        
    def step(self,u):
        
        ym1 = self.y
        vm1 = self.v
        
        yaux = self.ActFilter.step(u)
        
        if (yaux - ym1) > self.MaxStepChange:
            self.y = ym1 + self.MaxStepChange
        elif (yaux - ym1) < -self.MaxStepChange:
            self.y = ym1 - self.MaxStepChange
        else:
            self.y = yaux
        
        self.v = (self.y - ym1)/self.tss
        self.a = (self.v - vm1)/self.tss
         
        return self.y, self.v, self.a
        
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

Tilt = Actuator(CutFreq_radps = wc, MaxRate = rate, time_sample_actuator = t_act, time_sample_sim = t_sim)

Tilt.set_zero(y0)

t_vec = np.arange(0,2,step=t_sim)
u_vec = y0 + u_a*np.sin(u_w * t_vec)
u_vec[t_vec>=u_ts] = u_vec[t_vec>=u_ts] + u_d
y_vec = np.zeros(np.shape(t_vec))
v_vec = np.zeros(np.shape(t_vec))

y_vec[0] = y0

for i in range(1,len(t_vec)):
    
    y_vec[i],v_vec[i],a = Tilt.step(u_vec[i])
    
XLIM = (0,1)

plt.figure()
plt.subplot(2,1,1)
plt.plot(t_vec, u_vec, 'b')
plt.plot(t_vec, y_vec, 'r', label = 'actuator pos', markersize = 4)
plt.legend()
plt.xlim(XLIM)

plt.subplot(2,1,2)
plt.plot(t_vec, v_vec, 'r', label = 'actuator speed', markersize = 4)
plt.legend()
plt.xlim(XLIM)
    
                
                