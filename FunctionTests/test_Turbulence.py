#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 30 14:33:03 2023

@author: dsalarc
"""

import numpy as np
import matplotlib.pyplot as plt

Turb_U = 0
Turb_V = 0
Turb_W = 0

dt = 0.01
t_sim = 100

TowerWind_mps = 5;
Altitude_ft = 328
TAS = 10

Turb = {}
Turb['U'] = np.zeros(int(t_sim/dt))
Turb['V'] = np.zeros(int(t_sim/dt))
Turb['W'] = np.zeros(int(t_sim/dt))
TimeVec_s = np.arange(0, t_sim, dt)

y = 0
y = 0
xw = 0

AltitudeLimited_ft = max(10,min(1000,Altitude_ft))

Lu = AltitudeLimited_ft / ((0.177 + 0.000823 * AltitudeLimited_ft)**1.2)
Lv = Lu
Lw = AltitudeLimited_ft


sigma_w = TowerWind_mps * 0.1
sigma_u = sigma_w / ((0.177 + 0.000823*min(1000,Altitude_ft))**0.4)
sigma_v = sigma_u

Dx = TAS * dt

Ru = Dx / (2*Lu)
Au = (1-Ru)/(1+Ru)
Bu = Ru / (1+Ru)

Rv = Dx / (2*Lv)
Av = (1-Rv)/(1+Rv)
Bv = Rv / (1+Rv)

Rw = Dx / (2*Lw)
Aw = (1-Rw)/(1+Rw)
Bw = Rw / (1+Rw)
Cw = 3**0.5 / (1+Rw)

sigma_wn_u = sigma_u *(2*Lu / Dx)**0.5
sigma_wn_v = sigma_v *(2*Lv / Dx)**0.5
sigma_wn_w = sigma_w *(Lw / Dx)**0.5

rdm_u = np.random.default_rng(1)
rdm_v = np.random.default_rng(1)
rdm_w = np.random.default_rng(1)

for i in range(1,len(Turb['U'])):
    xu = rdm_u.normal(0,sigma_wn_u)
    xv = rdm_v.normal(0,sigma_wn_v)
    xw = rdm_w.normal(0,sigma_wn_w)
    
    Turb['U'][i] = Au * Turb['U'][i-1] + 2*Bu * xu
    Turb['V'][i] = Av * Turb['V'][i-1] + 2*Bv * xv
    
    last_y = y
    y = Aw * last_y + 2*Bw * xw 
    Turb['W'][i] = Aw * Turb['W'][i-1] + Bw * (y + last_y) + Cw * (y - last_y)

plt.figure()
plt.subplot(2,2,1)
plt.grid('on')
plt.plot(TimeVec_s , Turb['U'])

plt.subplot(2,2,2)
plt.grid('on')
plt.plot(TimeVec_s , Turb['V'])

plt.subplot(2,2,3)
plt.grid('on')
plt.plot(TimeVec_s , Turb['W'])

plt.subplot(2,2,4)
plt.grid('on')
plt.plot(TimeVec_s , Turb['U'])
plt.plot(TimeVec_s , Turb['V'])
plt.plot(TimeVec_s , Turb['W'])
