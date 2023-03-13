#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 21:31:10 2023

@author: dsalarc
"""

import numpy as np
import matplotlib.pyplot as plt

class ElectricalMotor:
    def __init__(self, Kq_A_Nm = 0, Kv_rpm_V = 0, i0_A = 0, R_ohm = 0, imax_A = 200, Sim_dt = 0.001):
        self.Kq     = Kq_A_Nm
        self.Kv     = Kv_rpm_V
        self.i0_A   = i0_A
        self.R_ohm  = R_ohm
        self.imax_A = imax_A
        self.dt     = Sim_dt
        
        # self.set_zero(w0_radps = 0, Qtgt_Nm = 0,)
            
    def set_zero(self,w0_radps,Qtgt_Nm):
        self.w_radps     = w0_radps
        self.wdot_radps2 = 0
        self.RPS         = self.w_radps / (2*np.pi)
        self.RPM         = self.RPS / 60
        self.Vm_V        = self.RPM / self.Kv
        self.Q_Nm        = Qtgt_Nm
        self.i_A         = (self.Q_Nm * self.Kq) + self.i0_A
        self.Energy_Ah   = 0
        Vinp_V      = self.Vm_V + self.i_A * self.R_ohm
        
        return Vinp_V, self.i_A
        
    def step(self,Vinp_V,RPM):
        self.RPM     = RPM
        self.RPS     = self.RPM * 60
        self.w_radps = self.RPS * (2*np.pi)
        self.Vm_V    = self.RPM / self.Kv
        last_i_A     = self.i_A
        self.i_A     = min(self.imax_A, (Vinp_V - self.Vm_V) / self.R_ohm)
        self.Q_Nm    = (self.i_A - self.i0_A) / self.Kq 
        self.Energy_Ah += max(0 , (self.i_A + last_i_A)/2 * self.dt / 3600)
        
        return self.Q_Nm
    
class Propeller:
    # https://x-engineer.org/discretizing-transfer-function/
    
    def __init__(self, Jvec, CTvec, CPvec, I_kgm2, Diam_m, Sim_dt):
        self.Jvec   = Jvec
        self.CTvec  = CTvec
        self.CPvec  = CPvec
        self.I_kgm2 = I_kgm2
        self.Diam_m = Diam_m
        self.dt     = Sim_dt
        
    def set_zero(self,w0_radps,TAS_mps = 0, rho_kgm3 = 1.225):
        self.w     = w0_radps
        self.wdot  = 0
        self.RPS   = self.w / (2*np.pi)
        self.RPM   = self.RPS / 60
        self.cQ_Nm = self.Torque(RPS = self.RPS, TAS_mps = TAS_mps, rho_kgm3 = rho_kgm3)
        
        return self.cQ_Nm
        
    def Thrust (self,RPS,TAS_mps, rho_kgm3): 
        J  = TAS_mps / (RPS * self.Diam_m)
        CT = np.interp(J, self.Jvec, self.CTvec)
        T  =  CT * rho_kgm3 * RPS**2 * self.Diam_m**4
        return T
        
    def Power (self,RPS,TAS_mps, rho_kgm3): 
        J  = TAS_mps / (RPS * self.Diam_m)
        CP = np.interp(J, self.Jvec, self.CPvec)
        P =  CP * rho_kgm3 * RPS**3 * self.Diam_m**5
        return P
        
    def Torque (self,RPS,TAS_mps, rho_kgm3): 
        J  = TAS_mps / (max(1,RPS) * self.Diam_m)
        CP = np.interp(J, self.Jvec, self.CPvec)
        CQ = CP / (2*np.pi)
        Q  =  CQ * rho_kgm3 * RPS**2 * self.Diam_m**5
        return Q
            
    def step(self,Qext,TAS_mps = 0, rho_kgm3 = 1.225):
        
        self.cQ_Nm    = self.Torque(RPS = self.RPS, TAS_mps = TAS_mps, rho_kgm3 = rho_kgm3)
        NetTorque     = Qext - self.cQ_Nm
        wdot          = NetTorque / self.I_kgm2
        self.w       += wdot * self.dt
        self.RPS      = self.w / (2*np.pi)
        self.RPM      = self.RPS / 60
        self.Thrust_N = self.Thrust(self.RPS ,TAS_mps, rho_kgm3)
            
        return self.w, self.cQ_Nm
    
class MotorESC:
    def __init__(self, KP = 0.001, KI = 0, KD = 0, MaxV_V = 1000, ESC_dt = 0.001):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.MaxV_V = MaxV_V
        self.MinV_V = 0
        self.ESC_dt = ESC_dt
        
    def set_zero(self,Vini_V = 0, RPMtgt = 0):
        self.V_V    = Vini_V
        self.RPMint = 0
        self.RPMtgt = RPMtgt
        self.RPMerr = 0
        
    def step (self,RPMtgt, RPMtrue):
        lastRPMerr = self.RPMerr
        self.RPMerr  = (RPMtgt - RPMtrue)
        
        
        auxV_V = self.RPMerr * self.KP + self.RPMint * self.KI + (self.RPMerr - lastRPMerr) * self.KD
        
        if auxV_V > self.MaxV_V:
            self.V_V = self.MaxV_V
            # self.RPMint += self.RPMerr*self.ESC_dt
        elif auxV_V < self.MinV_V:
             self.V_V = self.MinV_V          
        else:
            self.V_V = auxV_V
            self.RPMint += self.RPMerr*self.ESC_dt
       
        return self.V_V
    
class MotorAssembly:
    
    def __init__(self, ESC, MOTOR, PROPELLER, Sim_dt, Asb_dt):
        self.ESC = ESC
        self.MOTOR = MOTOR
        self.PROPELLER = PROPELLER
        self.Tscale = int(np.round(Sim_dt / Asb_dt))
        
        MOTOR.dt     = Asb_dt
        PROPELLER.dt = Asb_dt

    def set_zero(self,RPM0 = 0):
        w0_radps = RPM0 * (2*np.pi)/60
        cQ_Nm         = PROP.set_zero(w0_radps = w0_radps)
        Vini_V,iini_A = MOT.set_zero(w0_radps = w0_radps,Qtgt_Nm = cQ_Nm)
        ESC.set_zero(Vini_V = Vini_V, RPMtgt = RPM0)
        
        self.RPM = RPM0
        
    def step (self,Throttle):
        
        for i in range(self.Tscale):
            self.V_ESC = Throttle**(1/2) * ESC.MaxV_V
            Qmot       = MOT.step(self.V_ESC,self.RPM)  
            w,Qprop    = PROP.step(Qext = Qmot)
            self.RPM   = w * 60 / (2*np.pi)
        
        return self.RPM
        
        
# %%  
dt_sim  = 0.001
t_sim = 10
Diam_m = 1.5
Jvec   = np.array([0.0  , 0.01  , 2.00])
CTvec  = np.array([0.140, 0.140 , 0.000])
CPvec  = np.array([0.060, 0.060 , 0.003])
M_kg   = 0.526
I_kgm2 = M_kg*Diam_m**2 / 12


# https://uav-en.tmotor.com/html/2019/MannedAircraft_0618/272.html
Kq_A_Nm  = 1/0.371
Kv_rpm_V = 29
i0_A     = 0
R_ohm    = 20*1e-3

RPM0 = 0
w0_radps = RPM0/60*2*3.1416

PROP = Propeller(Jvec=Jvec, CTvec=CTvec, CPvec=CPvec, I_kgm2=I_kgm2, Diam_m = Diam_m, Sim_dt = dt_sim)
MOT  = ElectricalMotor(Kq_A_Nm = Kq_A_Nm, Kv_rpm_V = Kv_rpm_V, i0_A = i0_A, R_ohm = R_ohm, imax_A = 500)
ESC  = MotorESC(KP = 0.5, KI = 50, KD = 0, MaxV_V = 100, ESC_dt = dt_sim)

# %% VOLTAGE INPUT
cQ_Nm  = PROP.set_zero(w0_radps = w0_radps)
Vini_V,iini_A = MOT.set_zero(w0_radps = w0_radps,Qtgt_Nm = cQ_Nm)
ESC.set_zero(Vini_V = Vini_V, RPMtgt = RPM0)

t_vec = np.arange(0,t_sim,step=dt_sim)

Thr_vec_t   = np.array([0 , 0.100 , 0.101 , 0.300 , 0.301 , 0.500 , 0.501 , 0.700 , 0.701 , 0.800 , 10.000])
Thr_vec_Thr = np.array([0 , 0     , 0.35  , 0.35  , 0.75  , 0.75  , 0     , 0     , 0.75  , 0.75  , 0.75  ])
Thr_vec     = np.interp(t_vec , Thr_vec_t , Thr_vec_Thr)

# Thr_vec_t   = np.array([0 , 0.100 , 0.101 , 10.000])
# Thr_vec_Thr = np.array([0 , 0     , 1.00  , 1.00  ])
# Thr_vec     = np.interp(t_vec , Thr_vec_t , Thr_vec_Thr)

Thr_vec_t   = np.array([0 , 0.100 , 5.0 , 10.000])
Thr_vec_Thr = np.array([0 , 0     , 1.0 , 1.00  ])
Thr_vec     = np.interp(t_vec , Thr_vec_t , Thr_vec_Thr)

RPM_vec      = np.zeros(np.shape(t_vec))
RPM_vec[0]   = RPM0
V_vec        = np.zeros(np.shape(t_vec))
Qmot_vec     = np.zeros(np.shape(t_vec))
Qmot_vec[0]  = cQ_Nm
Qprop_vec    = np.zeros(np.shape(t_vec))
Qprop_vec[0] = cQ_Nm
imot_vec     = np.zeros(np.shape(t_vec))
imot_vec[0]  = iini_A
Thrust_vec   = np.zeros(np.shape(t_vec))

for i in range(1,len(t_vec)):
    V_vec[i]    = Thr_vec[i]**(1/2) * ESC.MaxV_V
    Qmot_vec[i] = MOT.step(V_vec[i],RPM_vec[i-1])  
    imot_vec[i] = MOT.i_A
    w,Qprop_vec[i] = PROP.step(Qext = Qmot_vec[i])
    RPM_vec[i] = w*60/(2*np.pi)
    Thrust_vec[i] = PROP.Thrust_N
   
XLIM = (0,1)

fig = plt.figure()
plt.subplot(2,3,1)
plt.grid('on')
plt.plot(t_vec, Qmot_vec, 'b',label = 'V input - Eng')
plt.plot(t_vec, Qprop_vec, 'r',label = 'V input - Prop')
# plt.plot(t_vec, y_vec, 'r', label = 'actuator pos', markersize = 4)
plt.legend()
plt.ylabel('Torque [N.m]')
plt.xlim(XLIM)

plt.subplot(2,3,3)
plt.grid('on')
plt.plot(t_vec, imot_vec, 'b', label = 'V input - Motor')
plt.legend()
plt.ylabel('Current [A]')
plt.xlim(XLIM)

plt.subplot(2,3,4)
plt.grid('on')
plt.plot(t_vec, RPM_vec, 'b', label = 'V input')
plt.legend()
plt.ylabel('RPM')
plt.xlim(XLIM)

plt.subplot(2,3,2)
plt.grid('on')
plt.plot(t_vec, V_vec, 'b', label = 'V input - ESC')
plt.legend()
plt.ylabel('Voltage [V]')
plt.xlim(XLIM)

plt.subplot(2,3,6)
plt.grid('on')
plt.plot(t_vec, Thrust_vec, 'b', label = 'V input')
plt.legend()
plt.ylabel('Thrust [N]')
plt.xlim(XLIM)

fig.tight_layout() 
              
# %% VOLTAGE - ASSEMBLY 
Sim_dt = 0.01
MOT_ASBLY =  MotorAssembly(ESC = ESC, MOTOR = MOT, PROPELLER = PROP, Sim_dt = Sim_dt, Asb_dt = dt_sim)
MOT_ASBLY.set_zero(RPM0 = RPM0)

t_vec = np.arange(0,t_sim,step=Sim_dt)

# Thr_vec_t   = np.array([0 , 0.100 , 0.101 , 0.300 , 0.301 , 0.500 , 0.501 , 0.700 , 0.701 , 0.800 , 10.000])
# Thr_vec_Thr = np.array([0 , 0     , 0.35  , 0.35  , 0.75  , 0.75  , 0     , 0     , 0.75  , 0.75  , 0.75  ])
Thr_vec     = np.interp(t_vec , Thr_vec_t , Thr_vec_Thr)

RPM_vec      = np.zeros(np.shape(t_vec))
RPM_vec[0]   = RPM0
V_vec        = np.zeros(np.shape(t_vec))
Qmot_vec     = np.zeros(np.shape(t_vec))
Qmot_vec[0]  = cQ_Nm
Qprop_vec    = np.zeros(np.shape(t_vec))
Qprop_vec[0] = cQ_Nm
imot_vec     = np.zeros(np.shape(t_vec))
imot_vec[0]  = iini_A
Thrust_vec   = np.zeros(np.shape(t_vec))

for i in range(1,len(t_vec)):
    MOT_ASBLY.step(Thr_vec[i])  
    V_vec[i]      = MOT_ASBLY.V_ESC
    Qmot_vec[i]   = MOT_ASBLY.MOTOR.Q_Nm
    imot_vec[i]   = MOT_ASBLY.MOTOR.i_A
    Qprop_vec[i]  = MOT_ASBLY.PROPELLER.cQ_Nm
    RPM_vec[i]    = MOT_ASBLY.RPM
    Thrust_vec[i] = MOT_ASBLY.PROPELLER.Thrust_N
    
XLIM = (0,10)
# XLIM = (0.1,0.2)

# fig = plt.figure()
plt.subplot(2,3,1)
plt.grid('on')
plt.plot(t_vec, Qmot_vec, 'b--',label = 'Assembly - Eng')
plt.plot(t_vec, Qprop_vec, 'r--',label = 'Assembly - Prop')
# plt.plot(t_vec, y_vec, 'r', label = 'actuator pos', markersize = 4)
plt.legend()
plt.ylabel('Torque [N.m]')
plt.xlim(XLIM)

plt.subplot(2,3,3)
plt.grid('on')
plt.plot(t_vec, imot_vec, 'b--', label = 'Assembly - Motor')
plt.legend()
plt.ylabel('Current [A]')
plt.xlim(XLIM)

plt.subplot(2,3,4)
plt.grid('on')
plt.plot(t_vec, RPM_vec, 'b--', label = 'Assembly')
plt.legend()
plt.ylabel('RPM')
plt.xlim(XLIM)

plt.subplot(2,3,2)
plt.grid('on')
plt.plot(t_vec, V_vec, 'b--', label = 'Assembly - ESC')
plt.legend()
plt.ylabel('Voltage [V]')
plt.xlim(XLIM)

plt.subplot(2,3,6)
plt.grid('on')
plt.plot(t_vec, Thrust_vec, 'b--', label = 'Assembly')
plt.legend()
plt.ylabel('Thrust [N]')
plt.xlim(XLIM)

fig.tight_layout() 
    
# # %% RPM INPUT (with ESC)

# cQ_Nm  = PROP.set_zero(w0_radps = w0_radps)
# Vini_V,iini_A = MOT.set_zero(w0_radps = w0_radps,Qtgt_Nm = cQ_Nm)
# ESC.set_zero(Vini_V = Vini_V, RPMtgt = RPM0)

# RPM_w = 0
# RPM_d = 3000
# RPM_a = 0
# RPM_ts = 0.02
# RPM0   = w0_radps*60/(2*np.pi)

# t_vec = np.arange(0,t_sim,step=dt_sim)
# RPMtgt_vec = RPM0 + RPM_a*np.sin(RPM_w * t_vec)
# RPMtgt_vec[t_vec>RPM_ts] = RPMtgt_vec[t_vec>RPM_ts] + RPM_d

# RPM_vec_t   = np.array([0 , 0.100 , 0.101 , 0.300 , 0.301 , 0.500 , 0.501 , 0.700 , 0.701 , 0.800 , 10.000])
# RPM_vec_RPM = np.array([0 , 0     , 1000  , 1000  , 2000  , 2000  , 0     , 0     , 2000  , 2000  , 2000  ])
# RPMtgt_vec  = np.interp(t_vec , RPM_vec_t , RPM_vec_RPM)

# RPM_vec = np.zeros(np.shape(t_vec))
# RPM_vec[0] = RPM0
# Vesc_vec = np.zeros(np.shape(t_vec))
# Qmot_vec = np.zeros(np.shape(t_vec))
# Qmot_vec[0] = cQ_Nm
# Qprop_vec = np.zeros(np.shape(t_vec))
# Qprop_vec[0] = cQ_Nm
# imot_vec = np.zeros(np.shape(t_vec))
# imot_vec[0] = iini_A


# for i in range(1,len(t_vec)):
#     Vesc_vec[i] = ESC.step(RPMtgt = RPMtgt_vec[i], RPMtrue = RPM_vec[i-1])
#     Qmot_vec[i] = MOT.step(Vesc_vec[i],RPM_vec[i-1])  
#     imot_vec[i] = MOT.i_A
#     w,Qprop_vec[i] = PROP.step(Qext = Qmot_vec[i])
#     RPM_vec[i] = w*60/(2*np.pi)
    
# XLIM = (0,1)
# # XLIM = (0.3,0.4)

# # fig = plt.figure()
# plt.subplot(2,3,1)
# plt.grid('on')
# plt.plot(t_vec, Qmot_vec, 'b--',label = 'RPM input - Eng')
# plt.plot(t_vec, Qprop_vec, 'r--',label = 'RPM input - Prop')
# # plt.plot(t_vec, y_vec, 'r', label = 'actuator pos', markersize = 4)
# plt.legend()
# plt.ylabel('Torque [N.m]')
# plt.xlim(XLIM)

# plt.subplot(2,3,3)
# plt.grid('on')
# plt.plot(t_vec, imot_vec, 'b--', label = 'RPM input - Motor')
# plt.legend()
# plt.ylabel('Current [A]')
# plt.xlim(XLIM)

# plt.subplot(2,3,4)
# plt.grid('on')
# plt.plot(t_vec, RPM_vec, 'b--', label = 'RPM input')
# plt.legend()
# plt.ylabel('RPM')
# plt.xlim(XLIM)

# plt.subplot(2,3,2)
# plt.grid('on')
# plt.plot(t_vec, Vesc_vec, 'b--', label = 'RPM input - ESC')
# plt.legend()
# plt.ylabel('Voltage [V]')
# plt.xlim(XLIM)

# fig.tight_layout() 
              
                