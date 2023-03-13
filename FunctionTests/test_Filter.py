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
    
    def __init__(self, wc, Ts, Tsim = -1, order = 1, DiscType = 'tustin'):
        self.Tc = 1/wc
        self.Ts = Ts
        self.y   = 0
        self.ym1 = 0
        self.ym2 = 0
        self.u   = 0
        self.um1 = 0
        self.um2 = 0
        self.order = order
        self.DiscType = DiscType
        if Tsim == -1:
            Tsim = Ts
            
        if np.mod(Tsim, Ts) > Ts/1000:
            raise('Simulation time sample shall me a multiple of filter time sample')
        else:
            self.Tscale = int(np.round(Tsim / Ts))

    
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
# %%  
ts = 0.05
wc = 40
w_u = 10
d_u = 0
y0 = 0

# Engine1 = LowPassDiscreteFilter(wc = wc, Ts=ts, DiscType = 'tustin')
# Engine2 = LowPassDiscreteFilter(wc = wc, Ts=ts, DiscType = 'euler_fwd')
# Engine3 = LowPassDiscreteFilter(wc = wc, Ts=ts, DiscType = 'euler_back')

# Engine1.set_zero(y0)
# Engine2.set_zero(y0)
# Engine3.set_zero(y0)

# t1_vec = np.arange(0,2,step=ts)
# u1_vec = y0 + y0 + np.sin(w_u * t1_vec)
# u1_vec[t1_vec>=1] = u1_vec[t1_vec>=1] + d_u
# y1_vec = np.zeros(np.shape(t1_vec))
# y2_vec = np.zeros(np.shape(t1_vec))
# y3_vec = np.zeros(np.shape(t1_vec))

# for i in range(len(t1_vec)):
#     y1_vec[i] = Engine1.step(u1_vec[i])
#     y2_vec[i] = Engine2.step(u1_vec[i])
#     y3_vec[i] = Engine3.step(u1_vec[i])
    
    
# % 
ts = 0.001


Engine4 = LowPassDiscreteFilter(wc = wc, Ts=ts, DiscType = 'euler_back')

Engine4.set_zero(y0)

t2_vec = np.arange(0,2,step=ts)
u2_vec = y0 + np.sin(w_u * t2_vec)
u2_vec[t2_vec>=1] = u2_vec[t2_vec>=1] + d_u
y4_vec = np.zeros(np.shape(t2_vec))

for i in range(len(t2_vec)):
    y4_vec[i] = Engine4.step(u2_vec[i])

# plt.figure()
# plt.plot(t2_vec, u2_vec)
# plt.plot(t1_vec, y1_vec, label = 'tustin')
# plt.plot(t1_vec, y2_vec, label = 'euler_fwd')
# plt.plot(t1_vec, y3_vec, label = 'euler_back')
# plt.plot(t2_vec, y4_vec, 'k', linewidth = 1, label = 'baseline')
# plt.legend()
# # plt.xlim((0.9,1.3))
# plt.show()    
                
# %%  
t_sim = 0.01
t_filt = 0.001

Engine = LowPassDiscreteFilter(wc = wc, Ts=t_filt, Tsim = t_sim, DiscType = 'euler_back')

Engine.set_zero(y0)

t_vec = np.arange(0,2,step=t_sim)
u_vec = y0 + np.sin(w_u * t_vec)
u_vec[t_vec>=1] = u_vec[t_vec>=1] + d_u
y_vec = np.zeros(np.shape(t_vec))

sf = int(t_sim / t_filt)

for i in range(1,len(t_vec)):
    
    y_vec[i] = Engine.step(u_vec[i])
    

plt.figure()
plt.plot(t_vec, u_vec, 'b')
plt.plot(t2_vec, y4_vec, 'k', linewidth = 1, label = 'baseline')
plt.plot(t_vec, y_vec, 'ro', label = 'filt 1000Hz', markersize = 4)
plt.legend()
plt.xlim((0,1))
    
                
                