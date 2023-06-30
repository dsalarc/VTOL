#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 21:02:13 2021

@author: dsalarc
"""

import gym
from gym import spaces
import numpy as np
from scipy.interpolate import RegularGridInterpolator
from scipy.interpolate import RectBivariateSpline
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
    
class Sensor:
    def __init__(self, CutFreq_radps = 40, Delay_s = 0, time_sample_sensor = 0.001, time_sample_sim = -1):
        
        if time_sample_sim == -1:
            time_sample_sim = time_sample_sensor

        self.DelaySteps = int(Delay_s / time_sample_sim)
        if self.DelaySteps == 0:
            self.BufferDelay = np.array([])
        else:
            self.BufferDelay = np.zeros(self.DelaySteps)
            
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
        elif self.DelaySteps == 1:
            self.y = self.BufferDelay[0]
            self.BufferDelay[0] = yaux
        else:
            self.y = self.BufferDelay[0]
            self.BufferDelay[0:-1] = self.BufferDelay[1:]
            self.BufferDelay[-1] = yaux
        
        return self.y

class ElectricalMotor:
    def __init__(self, Kq_A_Nm = 0, Kv_rpm_V = 0, i0_A = 0, R_ohm = 0, imax_A = 200, Sim_dt = 0.001):
        self.Kq     = Kq_A_Nm
        self.Kv     = Kv_rpm_V
        self.i0_A   = i0_A
        self.R_ohm  = R_ohm
        self.imax_A = imax_A
        self.dt     = Sim_dt
            
    def set_zero(self,Vinp_V,RPM):
        self.RPM     = RPM
        self.RPS     = self.RPM * 60
        self.w_radps = self.RPS * (2*np.pi)
        self.Vm_V    = self.RPM / self.Kv
        self.i_A     = min(self.imax_A, (Vinp_V - self.Vm_V) / self.R_ohm)
        self.Q_Nm    = (self.i_A - self.i0_A) / self.Kq      
        self.Energy_Ah   = 0
                
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
    
    def __init__(self, Tables, I_kgm2, Diam_m, Sim_dt):
        self.Tables = Tables
        self.I_kgm2 = I_kgm2
        self.Diam_m = Diam_m
        self.dt     = Sim_dt
       
    def set_zero(self, RPM0, TAS_mps, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0):
        self.RPM       = RPM0
        self.RPS       = RPM0/60
        self.Alpha_deg = Alpha_deg
        self.Pitch_deg = Pitch_deg
        self.J         = TAS_mps / (max(1,self.RPS) * self.Diam_m)    
        self.J_lin     = self.J * np.cos(np.deg2rad(self.Alpha_deg))
        self.w         = self.RPS * (2*np.pi)

        # Limit interpolation inputs
        self.inp_J         = min((max((self.J         , self.Tables['J'][0]))         ,  self.Tables['J'][-1]))
        self.inp_Alpha_deg = min((max((self.Alpha_deg , -self.Tables['Alpha_deg'][-1])) ,  self.Tables['Alpha_deg'][-1]))
        self.inp_Pitch_deg = min((max((self.Pitch_deg , self.Tables['Pitch_deg'][0])) ,  self.Tables['Pitch_deg'][-1]))

        self.cQ_Nm     = self.CalcTorque(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)
        self.Thrust_N  = self.CalcThrust(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)
        self.Normal_N  = -np.sign(self.inp_Alpha_deg)*self.CalcNormal(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)
        self.Torque_Nm = self.cQ_Nm
        self.Power_W   = self.CalcPower(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)
        
    def CalcThrust (self, RPS, J, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0): 
        self.CT = self.Tables['CTfcn'](Alpha_deg, J)[0][0]
        T  =  self.CT * rho_kgm3 * RPS**2 * self.Diam_m**4
        return T
        
    def CalcNormal (self, RPS, J, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0): 
        self.CN = self.Tables['CNfcn'](Alpha_deg, J)[0][0]
        N  =  self.CN * rho_kgm3 * RPS**2 * self.Diam_m**4
        return N
        
    def CalcPower (self, RPS, J, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0): 
        self.CP = self.Tables['CPfcn'](Alpha_deg, J)[0][0]
        P =  self.CP * rho_kgm3 * RPS**3 * self.Diam_m**5
        return P
        
    def CalcTorque (self, RPS, J, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0): 
        self.CP = self.Tables['CPfcn'](Alpha_deg, J)[0][0]
        self.CQ = self.CP / (2*np.pi)
        Q  =  self.CQ * rho_kgm3 * RPS**2 * self.Diam_m**5
        return Q
            
    def step(self,Qext,TAS_mps, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0):
        
        self.cQ_Nm     = self.CalcTorque(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)
        NetTorque      = Qext - self.cQ_Nm
        wdot           = NetTorque / self.I_kgm2
        self.w        += wdot * self.dt
        self.RPS       = self.w / (2*np.pi)
        self.RPM       = self.RPS / 60
        self.Alpha_deg = Alpha_deg
        self.J         = TAS_mps / (max(1,self.RPS) * self.Diam_m)    
        self.J_lin     = self.J * np.cos(np.deg2rad(self.Alpha_deg))

        # Limit interpolation inputs
        self.inp_J         = min((max((self.J         , self.Tables['J'][0]))         ,  self.Tables['J'][-1]))
        self.inp_Alpha_deg = min((max((self.Alpha_deg , -self.Tables['Alpha_deg'][-1])) ,  self.Tables['Alpha_deg'][-1]))
        self.inp_Pitch_deg = min((max((self.Pitch_deg , self.Tables['Pitch_deg'][0])) ,  self.Tables['Pitch_deg'][-1]))

        self.Thrust_N  = self.CalcThrust(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)
        self.Normal_N  = -np.sign(self.inp_Alpha_deg)*self.CalcNormal(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)
        self.Torque_Nm = self.cQ_Nm
        self.Power_W   = self.CalcPower(RPS = self.RPS, J = self.inp_J, rho_kgm3 = rho_kgm3, Alpha_deg = abs(self.inp_Alpha_deg), Pitch_deg = self.inp_Pitch_deg)

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

    def set_zero(self,Throttle, TAS_mps, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0):
        self.Throttle = Throttle
        self.V_V = self.calc_V(Throttle)

        RPM0 = 0       
        self.MOTOR.set_zero(self.V_V ,RPM0)
        self.PROPELLER.set_zero(RPM0, TAS_mps = TAS_mps, rho_kgm3 = rho_kgm3, Alpha_deg = Alpha_deg, Pitch_deg = Pitch_deg)
        Net0 = self.MOTOR.Q_Nm - self.PROPELLER.cQ_Nm
        
 
        RPM1 = 5000      
        self.MOTOR.set_zero(self.V_V ,RPM1)
        self.PROPELLER.set_zero(RPM1, TAS_mps = TAS_mps, rho_kgm3 = rho_kgm3, Alpha_deg = Alpha_deg, Pitch_deg = Pitch_deg)
        Net1 = self.MOTOR.Q_Nm - self.PROPELLER.cQ_Nm
       
        Net2 = np.inf
        it=0
        while abs(Net2) > 0.1:
            it += 1
            
            f = 3
            RPM2 = ((RPM0**f - RPM1**f) / (Net0 - Net1) * (0 - Net0) + RPM0**f)**(1/f)
            self.MOTOR.set_zero(self.V_V  , RPM2)
            self.PROPELLER.set_zero(RPM2, TAS_mps = TAS_mps, rho_kgm3 = rho_kgm3, Alpha_deg = Alpha_deg, Pitch_deg = Pitch_deg)
            Net2 = self.MOTOR.Q_Nm - self.PROPELLER.cQ_Nm
            
            if Net2 > 0:
                RPM0 = RPM2
                Net0 = Net2
            else:
                RPM1 = RPM2
                Net1 = Net2
       
        self.RPM = RPM2
        
    def calc_V(self,Throttle):
        return Throttle**(1/2) * self.ESC.MaxV_V
        
    def step (self,Throttle,TAS_mps, rho_kgm3, Alpha_deg = 0, Pitch_deg = 0):
        self.Throttle = Throttle
       
        for i in range(self.Tscale):
            self.V_V   = self.calc_V(Throttle)
            Qmot       = self.MOTOR.step(self.V_V,self.RPM)  
            w,Qprop    = self.PROPELLER.step(Qmot , TAS_mps , rho_kgm3, Alpha_deg = Alpha_deg, Pitch_deg = Pitch_deg)
            self.RPM   = w * 60 / (2*np.pi)
        
        return self.RPM
    
class Vahana_VertFlight(gym.Env):
    metadata = {'render.modes': ['console']}

    def __init__(self):
        
        #Define Constants     
        self.n_states = 12
        self.t_step = 0.01
        self.UseLateralActions = False
        
        # Define action and observation space
        '''
        sta: vetor de estados na ordem:
            XL_e: vetor posição lineares no eixo da terra (X, Y, Z)
            XR_e: vetor posição rotacional no eixo da terra (phi, theta psi)
            VL_b: vetor velocidades lineares no eixo do corpo (u,v,w)
            VR_b: vetor velocidade rotacional no eixo do corpo (p, q, r)
        '''
        # self.MaxState = np.array([np.inf,np.inf,np.inf,np.pi,np.pi,np.pi,np.inf,np.inf,np.inf,np.pi,np.pi,np.pi,
        #                           np.inf,np.inf,np.inf,np.pi,np.pi,np.pi,np.inf,np.inf,np.inf,np.pi,np.pi,np.pi])
        # Observation Space    = [Vx , dVx, Vy , dVy , Z  , Vz , dVz , Phi      , p        , Theta    , q        , Psi      , r        ]
        if self.UseLateralActions:
            self.adm_vec  = np.array([100 , 20 , 10 , 10  , 100, 10 , 10  , 3.1415/2 , 3.1415/2 , 3.1415/2 , 3.1415/2 , 3.1415/2 , 3.1415/2 , 100])
            self.MaxState = np.array([1   , 1  , 1  , 1   , 1  ,  1 , 1   , 1        , 1        , 1        , 1        , 1        ,1          , 1])
        else:
            self.adm_vec  = np.array([100 , 20 , 100, 10 , 10  , 3.1415/2 , 3.1415/2 , 100])
            self.MaxState = np.array([1  , 1  , 1  ,  1 , 1   , 1        , 1         , 1])
            
        
        '''
        ACTIONS:
            4 hover actions
            2 surface deflection (in % of Max Deflection)
            Front Wing Elevon
            Back Wing Elevon
            Aileron
        '''
        if self.UseLateralActions:
            self.action_names = ['Throttle','PitchThrottle',
                                 'RollThrottle','YawThrottle',
                                 'W1_Tilt','W2_Tilt',
                                 'W1_Elevator','W2_Elevator',
                                 'W1_Aileron','W2_Aileron']
        else:
            self.action_names = ['Throttle','PitchThrottle',
                                'W1_Tilt','W2_Tilt',
                                'W2_Elevator']
        
        self.action_space = spaces.Box(low=-1, 
                                       high=1,
                                       shape=(len(self.action_names),),
                                       dtype=np.float16)
        self.observation_space = spaces.Box(low=-self.MaxState,
                                            high=self.MaxState,
                                            dtype=np.float16)  

    
    def saveinfo(self):
      info = {}          
      info['ATM']  = self.ATM
      info['EQM']  = self.EQM
      info['MOT']  = self.MOT
      info['AERO'] = self.AERO
      info['CONT'] = self.CONT
      info['SENS'] = self.SENS
      info['MASS'] = self.MASS
      info['REW']  = self.LastReward
      info['Action'] = self.action
      self.info = info

      return info
      
    def OutputObs(self,sta,sta_dot,sta_int,cont):
        # obs = np.hstack((sta,sta_dot,cont))
        # Observation Space    = [Vx , dVx, Vy , dVy , Vz , dVz , Phi      , p        , Theta    , q        , Psi      , r        ]
        if self.UseLateralActions:
            if self.OPT['UseSensors']:
                obs_vec       = np.array([self.SENS['VX_mps']   , self.SENS['NX_mps2']  , 
                                          self.SENS['VY_mps']   , self.SENS['NY_mps2']  , 
                                          self.SENS['Z_m']      ,
                                          self.SENS['VZ_mps']   , self.SENS['NZ_mps2']  , 
                                          self.SENS['Phi_rad']  , self.SENS['P_radps']  , 
                                          self.SENS['Theta_rad'], self.SENS['Q_radps']  , 
                                          self.SENS['Psi_rad']  , self.SENS['R_radps']  , 
                                          self.SENS['CAS_mps']])
            else:
                obs_vec       = np.array([self.EQM['VelLin_EarthAx_mps'][0] , self.EQM['LoadFactor_mps2'][0] , 
                                          self.EQM['VelLin_EarthAx_mps'][1] , self.EQM['LoadFactor_mps2'][1] , 
                                          self.EQM['PosLin_EarthAx_m'][2]   ,
                                          self.EQM['VelLin_EarthAx_mps'][2] , self.EQM['LoadFactor_mps2'][2] , 
                                          self.EQM['EulerAngles_rad'][0]    , self.EQM['VelRot_BodyAx_radps'][0] , 
                                          self.EQM['EulerAngles_rad'][1]    , self.EQM['VelRot_BodyAx_radps'][1] , 
                                          self.EQM['EulerAngles_rad'][2]    , self.EQM['VelRot_BodyAx_radps'][2] , 
                                          self.ATM['CAS_mps']])

        else:
            if self.OPT['UseSensors']:
                obs_vec       = np.array([self.SENS['VX_mps']    , self.SENS['NX_mps2'], 
                                          self.SENS['Z_m']       ,
                                          self.SENS['VZ_mps']    , self.SENS['NZ_mps2'], 
                                          self.SENS['Theta_rad'] , self.SENS['Q_radps'],
                                          self.SENS['CAS_mps']])
            else:
                obs_vec       = np.array([self.EQM['VelLin_EarthAx_mps'][0] , self.EQM['LoadFactor_mps2'][0] , 
                                          self.EQM['PosLin_EarthAx_m'][2]   ,
                                          self.EQM['VelLin_EarthAx_mps'][2] , self.EQM['LoadFactor_mps2'][2] , 
                                          self.EQM['EulerAngles_rad'][1]    , self.EQM['VelRot_BodyAx_radps'][1] , 
                                          self.ATM['CAS_mps']])
           
        obs_adm = obs_vec / self.adm_vec
        
        obs_sat = np.min( np.vstack((obs_adm,np.ones(len(obs_vec)) )) , axis=0)
        obs     = np.max( np.vstack((obs_sat,-np.ones(len(obs_vec)))) , axis=0)

        # obs = np.array([sta[8],sta_dot[8]])
        return obs
    
    def reset(self,W = 0, Z = 0, THETA = 0,  PHI = 0,  PSI = 0, PaxIn = np.array([1,1]),
                   VX_mps = 0, VZ_mps = 0, DispMessages = False, Linearize = False, TermTheta_deg = 10, StaFreezeList = [],
                   UNC_seed = None , UNC_enable = 0):
      self.CurrentStep = 0
      self.trimming = 0

      # OPTIONS
      self.OPT  = {}
      self.OPT['UseAeroMoment'] = 1
      self.OPT['UseAeroForce']  = 1
      self.OPT['UsePropMoment'] = 1
      self.OPT['UsePropForce']  = 1
      self.OPT['UseSensors']    = 1
      self.OPT['EnableRoll']    = 1
      self.OPT['EnablePitch']   = 1
      self.OPT['EnableYaw']     = 1
      self.OPT['DispMessages']  = DispMessages
      self.OPT['StaFreezeList'] = StaFreezeList
      self.OPT['UNC_seed'] = UNC_seed
      self.OPT['UNC_enable'] = UNC_enable


      # Initialize Contants  
      self.Term = {}
      self.Term['Theta_deg'] = TermTheta_deg

      self.StartUp(PaxIn = PaxIn)
      
      self.EQM['sta']        = np.zeros(shape=12,dtype = np.float32)
      if W == 'rand':
          self.EQM['sta'][8] = np.random.randint(-1,2)*10
      else:
          self.EQM['sta'][8] = W    
          
      if THETA == 'rand':
          # self.EQM['sta'][4] = (2*np.random.random()-1)*10/57.3
          self.EQM['sta'][4] = np.random.randint(-1,2)*10/57.3
      else:
          self.EQM['sta'][4] = THETA  
          
      if PHI == 'rand':
          # self.EQM['sta'][3] = (2*np.random.random()-1)*10/57.3
          self.EQM['sta'][3] = np.random.randint(-1,2)*10/57.3
      else:
          self.EQM['sta'][3] = PHI  
          
      if PSI == 'rand':
          # self.EQM['sta'][5] = (2*np.random.random()-1)*10/57.3
          self.EQM['sta'][5] = np.random.randint(-1,2)*10/57.3
      else:
          self.EQM['sta'][5] = PSI  

      if Z == 'rand':
          self.EQM['sta'][2] = -np.random.random()*2000
      else:
          self.EQM['sta'][2]     = -Z
          
      
      self.EQM['sta_dot']    = np.zeros(shape=np.shape(self.EQM['sta']),dtype = np.float32)
      self.EQM['sta_dotdot'] = np.zeros(shape=np.shape(self.EQM['sta']),dtype = np.float32)
      self.EQM['sta_int']    = np.zeros(shape=np.shape(self.EQM['sta']),dtype = np.float32)
      
      TrimData = self.trim(TrimVX_mps = VX_mps, TrimVZ_mps = VZ_mps, TrimTheta_deg = THETA, 
                           PitchController = 'W2_Elevator', Linearize = Linearize)

      # If not trimmed with elevator only, or deflection abobe 10deg, trim with Pitch Throttle
      if (TrimData['Trimmed'] == 0) or (any(abs(TrimData['info']['CONT']['Elevon_deg'])>10)):
        Action_W2_Elevator = np.sign(TrimData['Action'][self.action_names.index('W2_Elevator')]) * 10/(self.CONT['ElevRange_deg'][2]/2)

        TrimData = self.trim(TrimVX_mps = VX_mps, TrimVZ_mps = VZ_mps, TrimTheta_deg = THETA, 
                             PitchController = 'PitchThrottle' , 
                             FixedAction = np.array(['W2_Elevator',Action_W2_Elevator]), Linearize = Linearize)

        # If signs of PitchThrottle and Elevator are different, invert elevator  
        if np.sign(TrimData['Action'][self.action_names.index('W2_Elevator')]) == np.sign(TrimData['Action'][self.action_names.index('PitchThrottle')]):
            TrimData = self.trim(TrimVX_mps = VX_mps, TrimVZ_mps = VZ_mps, TrimTheta_deg = THETA, 
                                PitchController = 'PitchThrottle' , 
                                FixedAction = np.array(['W2_Elevator',-Action_W2_Elevator]), Linearize = Linearize)


      self.TrimData = TrimData
      
      self.AllStates = self.EQM['sta']
      
      self.LastReward = self.CalcReward()

      obs = self.OutputObs(self.EQM['sta'],self.EQM['sta_dot'],self.EQM['sta_int'],self.CONT['Throttle_p'])
      
      self.saveinfo()

      return obs
  
    def trim(self, TrimVX_mps = 0, TrimVZ_mps = 0, TrimTheta_deg = 0, PitchController = 'PitchThrottle',FixedAction = np.array([]), Linearize = False):
        TrimData = {}
        TrimData['Trimmed'] = 0
       
        ''' Action Names:
        self.action_names = ['Throttle','PitchThrottle',
                             'RollThrottle','YawThrottle',
                             'W1_Tilt','W2_Tilt',
                             'W1_Elevator','W2_Elevator',
                             'W1_Aileron','W2_Aileron']
        '''
        IniAction = np.array([[0          ,  5         ,  10        ,  20        ,  30        ,  35         ,  40        ,  42        ,  45        ,  50        ,  55        ,  58         ,  60        ,  62        ,  65       ],
                              [-1.476e-01 , -1.463e-01 , -1.404e-01 , -1.436e-01 , -2.426e-01 , -2.997e-01  , -3.432e-01 , -3.869e-01 , -7.493e-01 , -7.054e-01 , -6.547e-01 , -6.217e-01  , -5.986e-01 , -5.744e-01 , -5.362e-01],
                              [-5.889e-02 , -5.795e-02 , -5.498e-02 , -4.025e-02 , -1.200e-02 ,  0.000e+00  ,  0.000e+00 ,  0.000e+00 ,  0.000e+00 ,  0.000e+00 ,  0.000e+00 ,  0.000e+00  ,  0.000e+00 ,  0.000e+00 ,  0.000e+00],
                              [ 1.000e+00 ,  9.618e-01 ,  8.481e-01 ,  4.325e-01 , -4.394e-02 , -2.334e-01  , -3.780e-01 , -4.387e-01 , -7.860e-01 , -8.266e-01 , -8.572e-01 , -8.717e-01  , -8.801e-01 , -8.878e-01 , -8.979e-01],
                              [ 1.000e+00 ,  9.618e-01 ,  8.481e-01 ,  4.325e-01 , -4.394e-02 , -2.334e-01  , -3.780e-01 , -4.387e-01 , -7.860e-01 , -8.266e-01 , -8.572e-01 , -8.717e-01  , -8.801e-01 , -8.878e-01 , -8.979e-01],
                              [ 6.667e-01 ,  6.667e-01 ,  6.667e-01 ,  6.667e-01 ,  3.025e-01 ,  6.959e-02  , -2.063e-02 , -2.047e-01 , -1.192e-01 , -1.149e-01 , -1.064e-01 , -1.027e-01  , -1.008e-01 , -0.987e-01 , -9.643e-02]])
        
        # Define function to get Outputs of interest
        def GetOutputFreeze(EQM, CONT, ATM, GetNames = False, DefaultOutputs = True):
            if DefaultOutputs:
                if GetNames:
                    return ['VX_mps' , 'VZ_mps', 'Theta_rad', 'TiltDiff_p']
                else:
                    return np.array([EQM['VelLin_EarthAx_mps'][0] ,
                                    EQM['VelLin_EarthAx_mps'][2] , 
                                    EQM['EulerAngles_rad'][1] , 
                                    CONT['TiltDiff_p'] ])
            else:
                if GetNames:
                    return ['VX_mps' , 'VZ_mps', 'Theta_deg', 'Q_degps', 'Nz_mps2','TiltDiff_p', 'Alpha_deg']
                else:
                    return np.array([EQM['VelLin_EarthAx_mps'][0] ,
                                    EQM['VelLin_EarthAx_mps'][2] , 
                                    np.rad2deg(EQM['EulerAngles_rad'][1]), 
                                    np.rad2deg(EQM['VelRot_BodyAx_radps'][1]), 
                                    EQM['LoadFactor_mps2'][2],
                                    CONT['TiltDiff_p'] ,
                                    ATM['Alpha_deg'] ])
                
        
       # Define function to calculate Hessian
        def HessianCalc (n_ActionFloat , n_StateFloat, n_StateDotFreeze, name_OutputFreeze, TrimAction, TrimState, pert, DefaultOutputs = True):
            H = np.zeros((len(n_StateDotFreeze) + len(name_OutputFreeze) , len(n_ActionFloat) + len(n_StateFloat)))
            for i in range(len(n_ActionFloat) + len(n_StateFloat)):
                
                TrimAction_Pert_p = TrimAction.copy()
                TrimAction_Pert_m = TrimAction.copy()
                TrimState_Pert_p  = TrimState.copy()
                TrimState_Pert_m  = TrimState.copy()
            
                if i < len(n_ActionFloat):  
                    
                    TrimAction_Pert_p[n_ActionFloat[i]] = TrimAction[n_ActionFloat[i]] + pert
                    TrimAction_Pert_m[n_ActionFloat[i]] = TrimAction[n_ActionFloat[i]] - pert
                    
                else:
                    j = i - len(n_ActionFloat) 
                    TrimState_Pert_p[n_StateFloat[j]]  = TrimState[n_StateFloat[j]] + pert
                    TrimState_Pert_m[n_StateFloat[j]]  = TrimState[n_StateFloat[j]] - pert
            
                self.EQM['sta'] = TrimState_Pert_p
                self.step(TrimAction_Pert_p)
                StateDot_p = self.EQM['sta_dot'][n_StateDotFreeze]
                Output_p  = GetOutputFreeze(self.EQM, self.CONT, self.ATM, DefaultOutputs = DefaultOutputs)
                
                self.EQM['sta'] = TrimState_Pert_m
                self.step(TrimAction_Pert_m)
                StateDot_m = self.EQM['sta_dot'][n_StateDotFreeze]
                Output_m  = GetOutputFreeze(self.EQM, self.CONT, self.ATM, DefaultOutputs = DefaultOutputs)

                H[:,i] = (np.hstack((StateDot_p,Output_p)) - np.hstack((StateDot_m,Output_m))) / (2*pert)
            
            return H

        #Trimming parameters
        TrimTol               = 1e-6
        TrimIter              = 50
        TrimIterNoImprovement = 5
        TrimPert              = 1e-5
        LambdaStep            = 1.0
        
        TrimTiltDiff_p = 0
        
        self.trimming = 1
        
        # Call EQM
        self.EQM_fcn(np.array([0,0,0]), np.array([0,0,0]), self.MASS['I_kgm'], self.MASS['Weight_kgf'])
                
        #Define Initial Trim Action
        
        TrimAction = np.zeros(np.shape(self.action_space))     
        for i in range(len(TrimAction)):
            TrimAction[i] = np.interp(TrimVX_mps,IniAction[0,:],IniAction[i+1,:])
        for i in range(int(len(FixedAction)/2)):
            TrimAction[self.action_names.index(FixedAction[i*2])] = FixedAction[i*2+1]
        
        TrimState = np.zeros(np.shape(self.EQM['sta_names']))
        TrimState[self.EQM['sta_names'].index('U_mps')] = TrimVX_mps
        
        # Define Freeze and Floats Indexes
        name_ActionFloat = ['Throttle', PitchController, 'W1_Tilt', 'W2_Tilt']
        n_ActionFloat    = np.zeros(len(name_ActionFloat), dtype = int)
        for i in range(len(n_ActionFloat)):
            n_ActionFloat[i] = int(self.action_names.index(name_ActionFloat[i]))

        name_StateFloat = ['Theta_rad', 'U_mps', 'W_mps']
        n_StateFloat    = np.zeros(len(name_StateFloat), dtype = int)
        for i in range(len(n_StateFloat)):
            n_StateFloat[i] = self.EQM['sta_names'].index(name_StateFloat[i]) 
 
        name_StateDotFreeze = ['U_mps', 'W_mps', 'Q_radps']
        n_StateDotFreeze    = np.zeros(len(name_StateFloat), dtype = int)
        for i in range(len(n_StateDotFreeze)):
            n_StateDotFreeze[i] = self.EQM['sta_names'].index(name_StateDotFreeze[i]) 

        name_OutputFreeze  = GetOutputFreeze(self.EQM, self.CONT, self.ATM, GetNames = True)
              
        TrimVars = np.hstack((TrimAction[n_ActionFloat],TrimState[n_StateFloat]))
        
        TrimVarsLim_p = np.ones(np.shape(TrimVars))
        TrimVarsLim_p[len(n_ActionFloat):] = np.inf
        TrimVarsLim_m = -np.ones(np.shape(TrimVars))
        TrimVarsLim_m[len(n_ActionFloat):] = -np.inf
        
        # Trim Target Vector
        TrimTarget = np.hstack((np.zeros(len(n_StateDotFreeze)) , np.array([TrimVX_mps, TrimVZ_mps, np.deg2rad(TrimTheta_deg), TrimTiltDiff_p])))
       
        # Perform One Step    
        self.EQM['sta'] = TrimState
        self.step(TrimAction)
        
        TrimStaDot = self.EQM['sta_dot'][n_StateDotFreeze]
        TrimOutput = GetOutputFreeze(self.EQM, self.CONT, self.ATM)
        TrimError  = np.hstack((TrimStaDot , TrimOutput)) - TrimTarget
        TrimErrorNorm = np.linalg.norm(TrimError)
        Min_TrimErrorNorm = TrimErrorNorm

        iter_n = 1
        n_NoImprovement = 0
        ContinueTrim = True


        while ContinueTrim:

            H = HessianCalc(n_ActionFloat , n_StateFloat, n_StateDotFreeze, name_OutputFreeze, TrimAction, TrimState, TrimPert)
            LastTrimVars = TrimVars
            DeltaTrimVars = np.matmul(np.linalg.pinv(H) , TrimError)

            for i in range(len(LastTrimVars)):
                if ((LastTrimVars[i] - LambdaStep*DeltaTrimVars[i]) > TrimVarsLim_p[i]):
                    TrimVars[i] = LastTrimVars[i] - LambdaStep/2*(TrimVarsLim_p[i]-LastTrimVars[i])
                elif((LastTrimVars[i] - LambdaStep*DeltaTrimVars[i]) < TrimVarsLim_m[i]):
                    TrimVars[i] = LastTrimVars[i] - LambdaStep/2*(TrimVarsLim_m[i]-LastTrimVars[i])
                else:
                    TrimVars[i] = LastTrimVars[i] - LambdaStep*DeltaTrimVars[i]

            TrimAction[n_ActionFloat] = TrimVars[0:len(n_ActionFloat)]
            TrimState[n_StateFloat] = TrimVars[len(n_ActionFloat):]
            
            
            # Perform One Step    
            self.EQM['sta'] = TrimState
            info = self.step(TrimAction)
            
            TrimStaDot = self.EQM['sta_dot'][n_StateDotFreeze]
            TrimOutput = GetOutputFreeze(self.EQM, self.CONT, self.ATM)           
            TrimError     = np.hstack((TrimStaDot , TrimOutput)) - TrimTarget
            TrimErrorNorm = np.linalg.norm(TrimError)

            if TrimErrorNorm < Min_TrimErrorNorm:
                Min_TrimErrorNorm = TrimErrorNorm
                n_NoImprovement = 0
            else:
                n_NoImprovement += 1

            iter_n = iter_n + 1
            
            if iter_n >=TrimIter:
                ContinueTrim = False
                if self.OPT['DispMessages']:
                    print('Trim Error - Max number of iterations')
            elif TrimErrorNorm <= TrimTol:
                ContinueTrim = False
                if self.OPT['DispMessages']:
                    print('Trim successful - error below tolerance')
                TrimData['Trimmed'] = 1
            elif n_NoImprovement >= TrimIterNoImprovement:
                ContinueTrim = False
                if self.OPT['DispMessages']:
                    print('Trim Error - No further convergence')
                TrimData['Trimmed'] = 0


        if Linearize:
            if self.OPT['DispMessages']:
                print('Linearizing ...')
            name_ActionFloat = self.action_names
            n_ActionFloat    = np.zeros(len(name_ActionFloat), dtype = int)
            for i in range(len(n_ActionFloat)):
                n_ActionFloat[i] = int(self.action_names.index(name_ActionFloat[i]))

            name_StateFloat = ['Q_radps', 'Theta_rad', 'U_mps', 'W_mps']
            n_StateFloat    = np.zeros(len(name_StateFloat), dtype = int)
            for i in range(len(n_StateFloat)):
                n_StateFloat[i] = self.EQM['sta_names'].index(name_StateFloat[i]) 
    
            name_StateDotFreeze = ['Q_radps', 'Theta_rad', 'U_mps', 'W_mps']
            n_StateDotFreeze    = np.zeros(len(name_StateFloat), dtype = int)
            for i in range(len(n_StateDotFreeze)):
                n_StateDotFreeze[i] = self.EQM['sta_names'].index(name_StateDotFreeze[i]) 
            
            name_OutputFreeze  = GetOutputFreeze(self.EQM, self.CONT, self.ATM, GetNames = True, DefaultOutputs = False)

            H = HessianCalc(n_ActionFloat , n_StateFloat, n_StateDotFreeze, name_OutputFreeze, TrimAction, TrimState, TrimPert, DefaultOutputs = False)
            TrimData['Linear']               = {}
            TrimData['Linear']['A']          = H[0:len(n_StateDotFreeze) , len(n_ActionFloat):]
            TrimData['Linear']['B']          = H[0:len(n_StateDotFreeze) , 0:len(n_ActionFloat) ]
            TrimData['Linear']['C']          = H[len(n_StateDotFreeze):  , len(n_ActionFloat):]
            TrimData['Linear']['D']          = H[len(n_StateDotFreeze):  , 0:len(n_ActionFloat) ]
            TrimData['Linear']['StaNames']   = name_StateDotFreeze
            TrimData['Linear']['InpNames']   = self.action_names 
            TrimData['Linear']['OutNames']   = name_OutputFreeze 

        self.trimming = 0
        TrimData['Action'] = TrimAction
        TrimData['info'] = info
        TrimData['iter_n'] = iter_n
    
        return TrimData
       

    def step(self, action):
      action = np.max((action,-np.ones(np.shape(action))),0)
      action = np.min((action,+np.ones(np.shape(action))),0)
      self.action = action

      if not(self.trimming):  
          self.CurrentStep += 1
      
      # If trimming, calculate the EQM outputs, but without 
      # calculating derivatives, just to calculate the outputs
      if self.trimming:  
          self.EQM_fcn(np.zeros(3),np.zeros(3),self.MASS['I_kgm'],self.MASS['Weight_kgf'], CalcStaDot = False)
      
      # Calculate Control, Atmosphere, Motor Forces and Aero Forces
      self.CONT_fcn(action)
      self.ATM_fcn()
      self.MOT_fcn()
      self.AERO_fcn()
      
      # Execute one time step within the environment
 
      self.EQM['TotalForce'] = (self.MOT['TotalForce_BodyAx_N'] +
                                self.AERO['TotalForce_BodyAx_N'])
      self.EQM['TotalMoment'] = (self.MOT['TotalMoment_BodyAx_Nm'] +
                                 self.AERO['TotalMoment_BodyAx_Nm'])  
 
      Last_Xdot    = self.EQM['sta_dot'].copy()
      self.EQM_fcn(self.EQM['TotalForce'],self.EQM['TotalMoment'],self.MASS['I_kgm'],self.MASS['Weight_kgf'])
      self.EQM['sta_dotdot'] = (self.EQM['sta_dot'] - Last_Xdot)/self.t_step
      
      if not(self.trimming):  
          self.EQM['sta'] = self.Euler_2nd(self.EQM['sta'],self.EQM['sta_dot'],self.EQM['sta_dotdot'],self.t_step)
          self.AllStates = np.vstack((self.AllStates,self.EQM['sta']))

          self.EQM['sta_int'] = self.EQM['sta_int'] + self.EQM['sta'] * self.t_step
      
              
      # Read all sensor data
      self.SENS_fcn()

      # Calculate Reward
      if not(self.trimming):
          self.LastReward = self.CalcReward()
      else:
          self.LastReward = 0
      
      # Terminal State = False   
      done = False
      if abs(np.rad2deg(self.EQM['sta'][4])) > self.Term['Theta_deg']:
          done = True

          
      # Export Model Oututs throught info
      info = self.saveinfo()

      obs = self.OutputObs(self.EQM['sta'],self.EQM['sta_dot'],self.EQM['sta_int'],self.CONT['Throttle_p'])

      if not(self.trimming):
          return obs, self.LastReward, done, info
      else:
          return info
          
    def init_REW(self):
        self.REW = {}

        self.REW['Target'] = {}
        self.REW['Target']['Vx']       = 60
        self.REW['Target']['Vz']       = 0
        self.REW['Target']['Z']        = 0
        self.REW['Target']['Theta']    = 0
        self.REW['Target']['Q']        = 0
        self.REW['Target']['Alpha_W1'] = 0
        self.REW['Target']['Alpha_W2'] = 0

        self.REW['Adm_n'] = {}
        self.REW['Adm_n']['Vx']       = 60
        self.REW['Adm_n']['Vz']       = 5
        self.REW['Adm_n']['Z']        = 5
        self.REW['Adm_n']['Theta']    = 5
        self.REW['Adm_n']['Q']        = 5
        self.REW['Adm_n']['Alpha_W1'] = 90
        self.REW['Adm_n']['Alpha_W2'] = 90

        self.REW['Adm_p'] = {}
        self.REW['Adm_p']['Vx']       = 10
        self.REW['Adm_p']['Vz']       = 5
        self.REW['Adm_p']['Z']        = 5
        self.REW['Adm_p']['Theta']    = 5
        self.REW['Adm_p']['Q']        = 5
        self.REW['Adm_p']['Alpha_W1'] = 90
        self.REW['Adm_p']['Alpha_W2'] = 90

        self.REW['Order'] = {}
        self.REW['Order']['Vx']       = 2
        self.REW['Order']['Vz']       = 1
        self.REW['Order']['Z']        = 1
        self.REW['Order']['Theta']    = 1
        self.REW['Order']['Q']        = 1
        self.REW['Order']['Alpha_W1'] = 1
        self.REW['Order']['Alpha_W2'] = 1

        self.REW['Weight'] = {}
        self.REW['Weight']['Vx']       = 0.40
        self.REW['Weight']['Vz']       = 0.15
        self.REW['Weight']['Z']        = 0.15
        self.REW['Weight']['Theta']    = 0.15
        self.REW['Weight']['Q']        = 0.15
        self.REW['Weight']['Alpha_W1'] = 0.10*0
        self.REW['Weight']['Alpha_W2'] = 0.10*0

        self.REW['DeadZone'] = {}
        self.REW['DeadZone']['Vx']       = 2
        self.REW['DeadZone']['Vz']       = 1
        self.REW['DeadZone']['Z']        = 1
        self.REW['DeadZone']['Theta']    = 1  
        self.REW['DeadZone']['Q']        = 1  
        self.REW['DeadZone']['Alpha_W1'] = 10
        self.REW['DeadZone']['Alpha_W2'] = 10

        self.REW['DeadZone']['Slope'] = 0.01

    def CalcReward(self):

        self.REW['Value'] = {}
        self.REW['Value']['Vx']        = self.EQM['VelLin_EarthAx_mps'][0]
        self.REW['Value']['Vz']        = self.EQM['VelLin_EarthAx_mps'][2]
        self.REW['Value']['Z']         = self.EQM['PosLin_EarthAx_m'][2]
        self.REW['Value']['Theta']     = np.rad2deg(self.EQM['EulerAngles_rad'][1])
        self.REW['Value']['Q']         = np.rad2deg(self.EQM['VelRot_BodyAx_radps'][1])
        self.REW['Value']['Alpha_W1']  = self.AERO['Wing1']['Alpha_deg']
        self.REW['Value']['Alpha_W2']  = self.AERO['Wing2']['Alpha_deg']

        Reward = 0

        for kk in self.REW['Target'].keys():
            Delta2Target = self.REW['Value'][kk] - self.REW['Target'][kk]
            abs_Delta2Target = np.abs(Delta2Target)
            
            if abs_Delta2Target < self.REW['DeadZone'][kk]:
                AuxReward = 1 - self.REW['DeadZone']['Slope']  / self.REW['DeadZone'][kk] * abs_Delta2Target
            elif (Delta2Target < -self.REW['Adm_n'][kk]) or (Delta2Target > self.REW['Adm_p'][kk]):
                AuxReward = 0
            elif self.REW['Value'][kk] < np.abs(self.REW['Target'][kk]):
                AuxReward = np.max((0 , (1 - self.REW['DeadZone']['Slope']) * 
                                    ((self.REW['Adm_n'][kk] - abs_Delta2Target) / 
                                    (self.REW['Adm_n'][kk] - self.REW['DeadZone'][kk]))**self.REW['Order'][kk] ))
            else:
                AuxReward = np.max((0 , (1 - self.REW['DeadZone']['Slope']) * 
                                    ((self.REW['Adm_p'][kk] - abs_Delta2Target) / 
                                    (self.REW['Adm_p'][kk] - self.REW['DeadZone'][kk]))**self.REW['Order'][kk] ))
            Reward += AuxReward * self.REW['Weight'][kk]

        return Reward    
    

     
      
    def render(self, mode='console', close=False):
        
        # Render the environment to the screen       
        plt.figure(1)
        plt.clf()
        plt.grid('on')

        if self.CurrentStep > 0:
            plt.plot(self.AllStates[:,2],self.AllStates[:,8],'tab:gray')
            plt.plot(self.AllStates[-1,2],self.AllStates[-1,8],'or')
        else:
            plt.plot(self.AllStates[2],self.AllStates[8],'ob')
        plt.xlabel('Position Z [m]')
        plt.ylabel('Velocity W [m/s]')
        plt.show()
        
    def close(self):
        pass
    
    
    # %% EULER INTEGRATOR
    def Euler_2nd(self,X,Xdot,Xdotdot,t_step):

        FreezeOff = np.ones(len(self.EQM['sta_names']))

        for i in range(len(self.OPT['StaFreezeList'])):
            FreezeOff[self.EQM['sta_names'].index(self.OPT['StaFreezeList'][i])] = 0

        X = X + np.multiply(FreezeOff , (Xdot*t_step + (Xdotdot*t_step**2)/2))
        return X
    
    def Euler_1st(self,X,Xdot,t_step):
        
        X = X + Xdot*t_step
        return X

    # %% SARTUP FUNCTION
    def StartUp (self,PaxIn):

      # INITIALIZE DICTS
      self.UNC  = {}
      self.EQM  = {}
      self.GEOM = {}
      self.ATM  = {}
      self.MASS = {}
      self.MOT  = {}
      self.AERO = {}
      self.CONS = {}
      self.CONT = {}
      self.SENS = {}
      
      # DEFINE CONSTANTS
      self.CONS['kt2mps'] = 0.514444
      self.CONS['mps2kt'] = 1 / self.CONS['kt2mps']     
      self.CONS['g_mps2'] = 9.806
       
      self.init_UNC()
      self.init_ATM()  
      self.init_GEOM()  
      self.init_MASS(PaxIn = PaxIn)  
      self.init_AERO()
      self.init_MOT()
      self.init_REW()
      self.init_CONT()
      self.init_SENS()
      
    def init_UNC (self):
      # 1) define the Std Deviation of the deviations and uncertanties
      
       self.UNC['StdDev'] = {}
       self.UNC['StdDev']['ATM'] = {}
       self.UNC['StdDev']['ATM']['Bias'] = {}
       self.UNC['StdDev']['ATM']['Bias']['WindX_kt'] = 10*0
       self.UNC['StdDev']['ATM']['Bias']['WindY_kt'] = 10*0
       self.UNC['StdDev']['ATM']['Bias']['WindZ_kt'] = 10*0
       self.UNC['StdDev']['ATM']['Bias']['dISA_C']   = 10*0
            
      
       self.UNC['StdDev']['AERO'] = {}
       self.UNC['StdDev']['AERO']['Gain'] = {}
       self.UNC['StdDev']['AERO']['Bias'] = {}
       self.UNC['StdDev']['AERO']['Gain']['CLa'] = 0.03
       self.UNC['StdDev']['AERO']['Gain']['ElevEff'] = 0.10
       self.UNC['StdDev']['AERO']['Bias']['CM0'] = 0.05
      
       self.UNC['StdDev']['MASS'] = {}
       self.UNC['StdDev']['MASS']['Gain'] = {}
       self.UNC['StdDev']['MASS']['Bias'] = {}
       self.UNC['StdDev']['MASS']['Bias']['CGX_cma'] = 0.01
       self.UNC['StdDev']['MASS']['Gain']['Weight'] = 0.05
      
       self.UNC['StdDev']['MOT'] = {}
       self.UNC['StdDev']['MOT']['Gain'] = {}
       self.UNC['StdDev']['MOT']['Bias'] = {}
       self.UNC['StdDev']['MOT']['Gain']['CT'] = 0.05
       self.UNC['StdDev']['MOT']['Gain']['CN'] = 0.10
       self.UNC['StdDev']['MOT']['Gain']['CP'] = 0.05
       self.UNC['StdDev']['MOT']['Gain']['Bandwidth'] = 0.05
       self.UNC['StdDev']['MOT']['Gain']['Kv'] = 0.05
       self.UNC['StdDev']['MOT']['Gain']['Kq'] = 0.05
      
       self.UNC['StdDev']['CONT'] = {}
       self.UNC['StdDev']['CONT']['Gain'] = {}
       self.UNC['StdDev']['CONT']['Bias'] = {}
       self.UNC['StdDev']['CONT']['Gain']['WingTilt_Bandwidth'] = 0.05
       self.UNC['StdDev']['CONT']['Gain']['WingTilt_Rate'] = 0.05
       self.UNC['StdDev']['CONT']['Gain']['Elevon_Bandwidth'] = 0.05
       self.UNC['StdDev']['CONT']['Gain']['Elevon_Rate'] = 0.05
    
       self.UNC['StdDev']['SENS'] = {}
       self.UNC['StdDev']['SENS']['Gain'] = {}
       self.UNC['StdDev']['SENS']['Bias'] = {}
       self.UNC['StdDev']['SENS']['Gain']['IMU_Bandwidth'] = 0.05
       self.UNC['StdDev']['SENS']['Gain']['ADS_Bandwidth'] = 0.05
       self.UNC['StdDev']['SENS']['Gain']['IMU_Delay'] = 0
       self.UNC['StdDev']['SENS']['Gain']['ADS_Delay'] = 0.05
       self.UNC['StdDev']['SENS']['Gain']['IMU_P']     = 0.03
       self.UNC['StdDev']['SENS']['Gain']['IMU_Q']     = 0.03
       self.UNC['StdDev']['SENS']['Gain']['IMU_R']     = 0.03
       self.UNC['StdDev']['SENS']['Bias']['IMU_P']     = 0.01
       self.UNC['StdDev']['SENS']['Bias']['IMU_Q']     = 0.01
       self.UNC['StdDev']['SENS']['Bias']['IMU_R']     = 0.01
       self.UNC['StdDev']['SENS']['Gain']['IMU_Phi']   = 0.03
       self.UNC['StdDev']['SENS']['Gain']['IMU_Theta'] = 0.03
       self.UNC['StdDev']['SENS']['Gain']['IMU_Psi']   = 0.03
       self.UNC['StdDev']['SENS']['Bias']['IMU_Phi']   = 0.01
       self.UNC['StdDev']['SENS']['Bias']['IMU_Theta'] = 0.01
       self.UNC['StdDev']['SENS']['Bias']['IMU_Psi']   = 0.01
       self.UNC['StdDev']['SENS']['Gain']['IMU_NX']    = 0.05
       self.UNC['StdDev']['SENS']['Gain']['IMU_NY']    = 0.05
       self.UNC['StdDev']['SENS']['Gain']['IMU_NZ']    = 0.05
       self.UNC['StdDev']['SENS']['Bias']['IMU_NX']    = 0.1
       self.UNC['StdDev']['SENS']['Bias']['IMU_NY']    = 0.1
       self.UNC['StdDev']['SENS']['Bias']['IMU_NZ']    = 0.1
       self.UNC['StdDev']['SENS']['Gain']['ADS_CAS']   = 0.05
      
      # 2) Calculate the real deviation, based on StdDeviation
       if self.OPT['UNC_seed'] is None:
           self.UNC['seed'] = np.random.randint(1,100000)
       else:
           self.UNC['seed'] = self.OPT['UNC_seed']
          
       self.UNC['Res'] = self.UNC['StdDev'].copy()
        
       def GetUncVal(InpDict, rdm, Enable):
           if type(InpDict) is dict:
               OutDict = {}
               for k in InpDict.keys():
                    OutDict[k] = GetUncVal(InpDict[k], rdm, Enable)
                    # For Debug
                    # if type(OutDict[k]) is not dict:
                    #     print(k + ": " + str(OutDict[k]))
               return OutDict
           else:
                if Enable:
                   return np.max((-3*InpDict, np.min((3*InpDict,rdm.normal() * InpDict)) ))
                else:
                   return 0
        
       rdm = np.random.default_rng(self.UNC['seed'])
       self.UNC['Res'] = GetUncVal(self.UNC['Res'], rdm, self.OPT['UNC_enable'])
     
    def init_ATM (self):
      # ATM
      self.ATM['dISA_C'] = self.UNC['Res']['ATM']['Bias']['dISA_C']
      
      self.ATM['WindX_kt'] = self.UNC['Res']['ATM']['Bias']['WindX_kt']
      self.ATM['WindY_kt'] = self.UNC['Res']['ATM']['Bias']['WindY_kt']
      self.ATM['WindZ_kt'] = self.UNC['Res']['ATM']['Bias']['WindZ_kt']
      
      self.ATM['Const'] = {}
      
      self.ATM['Const']['C2K']         = 273.15
      self.ATM['Const']['R']           = 287.04  
      self.ATM['Const']['P0_Pa']       = 101325
      self.ATM['Const']['T0_C']        = 15
      self.ATM['Const']['T0_K']        = self.ATM['Const']['T0_C'] + self.ATM['Const']['C2K']
      self.ATM['Const']['rho0_kgm3']   = 1.225
      self.ATM['Const']['Vsound0_mps'] = np.sqrt(1.4*self.ATM['Const']['R']*(self.ATM['Const']['T0_K']))
        
    def init_GEOM (self):
      # GEOM
      self.GEOM['Wing1']          = {}
      self.GEOM['Wing1']['cma_m'] = 0.65
      self.GEOM['Wing1']['b_m']   = 6.0
      self.GEOM['Wing1']['S_m2']  = 3.9
      self.GEOM['Wing1']['XYZ_m'] = np.array([0.30, 0.00, 0.00])

      self.GEOM['Wing2']          = {}
      self.GEOM['Wing2']['cma_m'] = 0.65
      self.GEOM['Wing2']['b_m']   = 6.0
      self.GEOM['Wing2']['S_m2']  = 3.9
      self.GEOM['Wing2']['XYZ_m'] = np.array([3.40, 0.00, 1.50])
      
      self.GEOM['Fus']          = {}
      self.GEOM['Fus']['cma_m'] = 4.10
      self.GEOM['Fus']['b_m']   = 0.90
      self.GEOM['Fus']['S_m2']  = np.pi * 0.45**2
      self.GEOM['Fus']['XYZ_m'] = np.array([2.05, 0.00, 0.00])
                                
    def init_MASS (self , PaxIn = 0):
      # MASS
      self.MASS['Pax']             = PaxIn
      self.MASS['PaxWeight_kgf']   = 100
      self.MASS['EmptyWeight_kgf'] = 616
      self.MASS['Weight_kgf'] = self.MASS['EmptyWeight_kgf'] + np.sum(self.MASS['Pax']) * self.MASS['PaxWeight_kgf']
      self.MASS['Weight_kgf'] = self.MASS['Weight_kgf'] * (1 + self.UNC['Res']['MASS']['Gain']['Weight'])
      self.MASS['Empty_CG_m'] = np.array([1.85 , 0.0 , 0.53])
      self.MASS['PaxPos_m'] = np.array([[0.9 , 0.0 , 1.0],
                                        [2.5 , 0.0 , 1.0]])
      self.MASS['CG_m'] = np.array(self.MASS['Empty_CG_m']    * self.MASS['EmptyWeight_kgf'] + 
                                   self.MASS['PaxPos_m'][0,:] * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                                   self.MASS['PaxPos_m'][1,:] * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf']) / self.MASS['Weight_kgf']
      self.MASS['CG_m'][0] = self.MASS['CG_m'][0] + self.UNC['Res']['MASS']['Bias']['CGX_cma']*self.GEOM['Wing1']['cma_m']
      
      Pax2CG_sq = (self.MASS['PaxPos_m'] - self.MASS['CG_m'])**2                  
      Emp2CG_sq = (self.MASS['Empty_CG_m'] - self.MASS['CG_m'])**2                  
      Ixx = 1987 + ((Pax2CG_sq[0][1] + Pax2CG_sq[0][2]) * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                    (Pax2CG_sq[1][1] + Pax2CG_sq[1][2]) * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf'] + 
                    (Emp2CG_sq[1]    + Emp2CG_sq[2])    * self.MASS['EmptyWeight_kgf'])
      Iyy = 1648 + ((Pax2CG_sq[0][0] + Pax2CG_sq[0][2]) * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                    (Pax2CG_sq[1][0] + Pax2CG_sq[1][2]) * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf'] +
                    (Emp2CG_sq[0]    + Emp2CG_sq[2])    * self.MASS['EmptyWeight_kgf'])
      Izz = 2967 + ((Pax2CG_sq[0][0] + Pax2CG_sq[0][1]) * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                    (Pax2CG_sq[1][0] + Pax2CG_sq[1][1]) * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf'] +
                    (Emp2CG_sq[0]    + Emp2CG_sq[1])    * self.MASS['EmptyWeight_kgf'])
      self.MASS['I_kgm'] = np.array([[Ixx,0,0],[0,Iyy,0],[0,0,Izz]])
        
    def init_AERO (self):
 
      # AERO
      self.AERO['Wing1'] = {}
      self.AERO['Wing2'] = {}
      self.AERO['Fus']  = {}
      self.AERO['Elevon']  = {}
      
      self.AERO['MRC_m']   = np.array([2.500 , 0.000 , 0.000])
      self.AERO['Sref_m2'] = 3.900
      self.AERO['cref_m']  = 0.650
      self.AERO['bref_m']  = 6.000
      
      self.AERO['Wing2']['EPS0']          = -0.2176
      self.AERO['Wing2']['dEPS_dCLW1']    = +1.4465
      self.AERO['Wing2']['dEPS_dAOAacft'] = -0.0026

      self.AERO['Wing1']['Coefs'] = {}
      self.AERO['Wing1']['Coefs']['Alpha_deg']   = np.array([0      , 1      , 2      , 3      , 4      , 5      , 6      , 7      , 8      , 9      , 10     , 11     , 12     , 13     , 14     , 15     , 16     , 17     , 18     , 19     , 20     , 21     , 22     , 23     , 24     , 25     , 26    ])
      self.AERO['Wing1']['Coefs']['CLS_25Local'] = np.array([0.0000 , 0.0891 , 0.1775 , 0.2652 , 0.3524 , 0.4390 , 0.5248 , 0.6097 , 0.6930 , 0.7772 , 0.8669 , 0.9616 , 1.0572 , 1.1379 , 1.2040 , 1.2676 , 1.3273 , 1.3838 , 1.4330 , 1.4768 , 1.4705 , 1.3991 , 1.2325 , 1.0344 , 0.8687 , 0.7970 , 0.7888])
      self.AERO['Wing1']['Coefs']['CDS_25Local'] = np.array([0.0057 , 0.0061 , 0.0070 , 0.0086 , 0.0108 , 0.0137 , 0.0172 , 0.0213 , 0.0260 , 0.0313 , 0.0374 , 0.0446 , 0.0526 , 0.0603 , 0.0677 , 0.0753 , 0.0830 , 0.0908 , 0.0992 , 0.1080 , 0.1247 , 0.1542 , 0.2027 , 0.2591 , 0.3128 , 0.3530 , 0.3843])
      self.AERO['Wing1']['Coefs']['CMS_25Local'] = np.array([0.0000 , 0.0001 , 0.0003 , 0.0006 , 0.0010 , 0.0015 , 0.0023 , 0.0032 , 0.0040 , 0.0048 , 0.0055 , 0.0063 , 0.0071 , 0.0078 , 0.0090 , 0.0112 , 0.0147 , 0.0192 , 0.0234 , 0.0269 , 0.0271 , 0.0246 , 0.0184 , 0.0108 , 0.0043 , 0.0011 , 0.0000])
      # Dynamic derivatives from AVL, for AOA = 5
      self.AERO['Wing1']['Coefs']['CRp'] = -0.547338
      self.AERO['Wing1']['Coefs']['CRr'] = +0.081196
      self.AERO['Wing1']['Coefs']['CNp'] = -0.004602
      self.AERO['Wing1']['Coefs']['CNr'] = -0.002512
      self.AERO['Wing1']['Coefs']['CYr'] = -0.005922
      self.AERO['Wing1']['Coefs']['CMq'] = -106.2060
      self.AERO['Wing1']['Coefs']['CLq'] = -29.56319

      self.AERO['Wing2']['Coefs'] = {}
      self.AERO['Wing2']['Coefs']['Alpha_deg']   = np.array([0      , 1      , 2      , 3      , 4      , 5      , 6      , 7      , 8      , 9      , 10     , 11     , 12     , 13     , 14     , 15     , 16     , 17     , 18     , 19     , 20     , 21     , 22     , 23     , 24     , 25     , 26    ])
      self.AERO['Wing2']['Coefs']['CLS_25Local'] = np.array([0.0000 , 0.1024 , 0.2039 , 0.3048 , 0.4050 , 0.5045 , 0.6031 , 0.7006 , 0.7963 , 0.8932 , 0.9963 , 1.1050 , 1.2149 , 1.3077 , 1.3836 , 1.4567 , 1.5253 , 1.5903 , 1.6468 , 1.6972 , 1.6858 , 1.5915 , 1.3753 , 1.1185 , 0.9024 , 0.8055 , 0.7888])
      self.AERO['Wing2']['Coefs']['CDS_25Local'] = np.array([0.0084 , 0.0087 , 0.0096 , 0.0110 , 0.0130 , 0.0155 , 0.0187 , 0.0223 , 0.0264 , 0.0311 , 0.0366 , 0.0429 , 0.0500 , 0.0569 , 0.0635 , 0.0702 , 0.0771 , 0.0842 , 0.0918 , 0.0999 , 0.1165 , 0.1467 , 0.1970 , 0.2558 , 0.3114 , 0.3527 , 0.3843])
      self.AERO['Wing2']['Coefs']['CMS_25Local'] = np.array([0.0000 , 0.0001 , 0.0003 , 0.0006 , 0.0010 , 0.0015 , 0.0023 , 0.0032 , 0.0040 , 0.0048 , 0.0055 , 0.0063 , 0.0071 , 0.0078 , 0.0090 , 0.0112 , 0.0147 , 0.0192 , 0.0234 , 0.0269 , 0.0271 , 0.0246 , 0.0184 , 0.0108 , 0.0043 , 0.0011 , 0.0000])
      # Dynamic derivatives from AVL, for AOA = 5
      self.AERO['Wing2']['Coefs']['CRp'] = -1.021109
      self.AERO['Wing2']['Coefs']['CRr'] = +0.112894
      self.AERO['Wing2']['Coefs']['CNp'] = -0.066839
      self.AERO['Wing2']['Coefs']['CNr'] = -0.358400
      self.AERO['Wing2']['Coefs']['CYr'] = -0.942011
      self.AERO['Wing2']['Coefs']['CMq'] = -129.7503
      self.AERO['Wing2']['Coefs']['CLq'] = -36.30297

      self.AERO['Fus']['Coefs'] = {}
      self.AERO['Fus']['Coefs']['CD0'] = 0.1
      self.AERO['Fus']['Coefs']['CYbeta'] = -0.4
      self.AERO['Fus']['Coefs']['CRp'] = 0
      self.AERO['Fus']['Coefs']['CRr'] = 0
      self.AERO['Fus']['Coefs']['CNp'] = 0
      self.AERO['Fus']['Coefs']['CNr'] = 0
      self.AERO['Fus']['Coefs']['CMq'] = 0

      self.AERO['Elevon']['dCDSde_MRC']  = np.array([+0.000000 , +0.000000 , +0.000000 , +0.000000])
      self.AERO['Elevon']['dCYSde_MRC']  = np.array([+0.000000 , +0.000000 , +0.000000 , +0.000000])
      self.AERO['Elevon']['dCLSde_MRC']  = np.array([+0.009907 , +0.009907 , +0.014602 , +0.014602]) * (1 + self.UNC['Res']['AERO']['Gain']['ElevEff'])
      self.AERO['Elevon']['dCRSde_MRC']  = np.array([+0.002925 , -0.002925 , +0.004620 , -0.004620]) * (1 + self.UNC['Res']['AERO']['Gain']['ElevEff'])
      self.AERO['Elevon']['dCMSde_MRC']  = np.array([+0.042829 , +0.042829 , -0.055021 , -0.055021]) * (1 + self.UNC['Res']['AERO']['Gain']['ElevEff'])
      self.AERO['Elevon']['dCNSde_MRC']  = np.array([+0.000000 , +0.000000 , +0.000000 , +0.000000])

      self.AERO['Elevon']['AOAeff']  = {}
      self.AERO['Elevon']['AOAeff']['Alpha_deg']  = np.array([-180 , -45 , -20 ,  20 ,  45 , 180])
      self.AERO['Elevon']['AOAeff']['Gain']       = np.array([ 0.0 , 0.0 , 1.0 , 1.0 , 0.0 , 0.0])

    
    def init_MOT (self): 
      self.MOT['ESC'] = {}
      self.MOT['MOTOR'] = {}
      self.MOT['ASSEMBLY'] = {}
      
      # MOTOR
      x1 = 0.05
      x2 = 3.15
      y1 = 1.3
      y2 = 3.0
      z1  = 0
      z2 = 1.5
    
      self.MOT['Position_m'] = np.array([[x1,-y2,z1],
                                         [x1,-y1,z1],
                                         [x1,+y1,z1],
                                         [x1,+y2,z1],
                                         [x2,-y2,z2],
                                         [x2,-y1,z2],
                                         [x2,+y1,z2],
                                         [x2,+y2,z2]])
      self.MOT['n_motor'] = np.shape(self.MOT['Position_m'])[0]
      
      self.MOT['PROPELLER'] = {}
      self.MOT['PROPELLER']['MaxRPM']        = np.ones(self.MOT['n_motor']) * 3000
      self.MOT['PROPELLER']['MinRPM']        = np.ones(self.MOT['n_motor']) * 0.01
      self.MOT['PROPELLER']['RPMRange']      = self.MOT['PROPELLER']['MaxRPM'] - self.MOT['PROPELLER']['MinRPM'] 
      self.MOT['PROPELLER']['Diameter_m']    = np.ones(self.MOT['n_motor']) * 1.5
      self.MOT['RotationSense'] = np.array([+1,-1,+1,-1,
                                                         -1,+1,-1,+1])  
      
      # CT e CP - Vide planilha
      self.MOT['PROPELLER']['Tables'] = {}
      self.MOT['PROPELLER']['Tables']['J']         = np.array([0.0 , 0.1 , 0.2 , 0.3 , 0.4 , 0.5 , 0.6 , 0.7 , 0.8 , 0.9 , 1.0 , 1.1 , 1.2 , 1.3 , 1.4 , 1.5 , 1.6 , 1.7 , 1.8 , 1.9 ,  2.0])
      self.MOT['PROPELLER']['Tables']['Alpha_deg'] = np.array([0.0 , 15.0 , 30.0 , 45.0 , 60.0 , 75.0 , 80.0 ,  85.0])
      self.MOT['PROPELLER']['Tables']['Pitch_deg'] = np.array([0.0 , 30.0])
      

      self.MOT['PROPELLER']['Tables']['CT'] = np.array([[0.1592 , 0.1497 , 0.1417 , 0.1348 , 0.1281 , 0.1211 , 0.1143 , 0.1074 , 0.1008 , 0.0946 , 0.0877 , 0.0804 , 0.0729 , 0.0666 , 0.0587 , 0.0492 , 0.0380 , 0.0278 , 0.0187 , 0.0107 , 0.0034 ],
                                                        [0.1592 , 0.1588 , 0.1562 , 0.1508 , 0.1435 , 0.1352 , 0.1266 , 0.1187 , 0.1127 , 0.1095 , 0.1055 , 0.1004 , 0.0940 , 0.0890 , 0.0833 , 0.0765 , 0.0676 , 0.0579 , 0.0468 , 0.0343 , 0.0209 ],
                                                        [0.1592 , 0.1592 , 0.1576 , 0.1536 , 0.1490 , 0.1449 , 0.1421 , 0.1394 , 0.1373 , 0.1364 , 0.1355 , 0.1345 , 0.1331 , 0.1333 , 0.1325 , 0.1304 , 0.1268 , 0.1238 , 0.1209 , 0.1180 , 0.1151 ],
                                                        [0.1592 , 0.1606 , 0.1614 , 0.1612 , 0.1606 , 0.1603 , 0.1611 , 0.1642 , 0.1698 , 0.1781 , 0.1856 , 0.1920 , 0.1976 , 0.2052 , 0.2109 , 0.2142 , 0.2145 , 0.2150 , 0.2157 , 0.2164 , 0.2172 ],
                                                        [0.1592 , 0.1652 , 0.1709 , 0.1760 , 0.1815 , 0.1876 , 0.1951 , 0.2035 , 0.2135 , 0.2255 , 0.2370 , 0.2477 , 0.2573 , 0.2682 , 0.2791 , 0.2897 , 0.2995 , 0.3095 , 0.3196 , 0.3298 , 0.3400 ],
                                                        [0.1592 , 0.1712 , 0.1826 , 0.1934 , 0.2053 , 0.2188 , 0.2342 , 0.2476 , 0.2596 , 0.2705 , 0.2830 , 0.2966 , 0.3108 , 0.3253 , 0.3390 , 0.3519 , 0.3636 , 0.3746 , 0.3844 , 0.3927 , 0.3998 ],
                                                        [0.1592 , 0.1736 , 0.1872 , 0.2000 , 0.2140 , 0.2297 , 0.2472 , 0.2632 , 0.2783 , 0.2931 , 0.3096 , 0.3272 , 0.3454 , 0.3637 , 0.3808 , 0.3970 , 0.4127 , 0.4294 , 0.3096 , 0.3272 , 0.3454 ],
                                                        [0.1592 , 0.1762 , 0.1930 , 0.2096 , 0.2256 , 0.2409 , 0.2558 , 0.2717 , 0.2891 , 0.3086 , 0.3284 , 0.3488 , 0.3693 , 0.3913 , 0.4140 , 0.4371 , 0.4607 , 0.4849 , 0.5099 , 0.5356 , 0.5618 ]])
      self.MOT['PROPELLER']['Tables']['CT'] = self.MOT['PROPELLER']['Tables']['CT'] * (1+self.UNC['Res']['MOT']['Gain']['CT'])
      self.MOT['PROPELLER']['Tables']['CTfcn'] = RectBivariateSpline(self.MOT['PROPELLER']['Tables']['Alpha_deg'],
                                                                     self.MOT['PROPELLER']['Tables']['J'],
                                                                     self.MOT['PROPELLER']['Tables']['CT'],
                                                                     kx=1, ky=1)

      self.MOT['PROPELLER']['Tables']['CN'] = np.array([[0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 ],
                                                        [0.0000 , 0.0007 , 0.0017 , 0.0029 , 0.0042 , 0.0056 , 0.0074 , 0.0096 , 0.0123 , 0.0152 , 0.0185 , 0.0221 , 0.0261 , 0.0309 , 0.0365 , 0.0429 , 0.0501 , 0.0580 , 0.0664 , 0.0749 , 0.0835 ],
                                                        [0.0000 , 0.0008 , 0.0022 , 0.0044 , 0.0071 , 0.0103 , 0.0139 , 0.0180 , 0.0229 , 0.0285 , 0.0348 , 0.0419 , 0.0499 , 0.0588 , 0.0686 , 0.0795 , 0.0915 , 0.1045 , 0.1184 , 0.1332 , 0.1484 ],
                                                        [0.0000 , 0.0021 , 0.0045 , 0.0076 , 0.0115 , 0.0160 , 0.0214 , 0.0277 , 0.0353 , 0.0443 , 0.0540 , 0.0645 , 0.0757 , 0.0878 , 0.1007 , 0.1143 , 0.1287 , 0.1441 , 0.1603 , 0.1774 , 0.1950 ],
                                                        [0.0000 , 0.0034 , 0.0068 , 0.0105 , 0.0148 , 0.0202 , 0.0266 , 0.0346 , 0.0442 , 0.0555 , 0.0677 , 0.0805 , 0.0941 , 0.1086 , 0.1238 , 0.1396 , 0.1561 , 0.1736 , 0.1922 , 0.2118 , 0.2320 ],
                                                        [0.0000 , 0.0023 , 0.0053 , 0.0093 , 0.0145 , 0.0208 , 0.0284 , 0.0375 , 0.0483 , 0.0605 , 0.0736 , 0.0876 , 0.1028 , 0.1195 , 0.1364 , 0.1532 , 0.1693 , 0.1856 , 0.2020 , 0.2181 , 0.2342 ],
                                                        [0.0000 , 0.0033 , 0.0073 , 0.0122 , 0.0179 , 0.0242 , 0.0313 , 0.0399 , 0.0502 , 0.0620 , 0.0745 , 0.0876 , 0.1014 , 0.1162 , 0.1311 , 0.1460 , 0.1605 , 0.1753 , 0.0745 , 0.0876 , 0.1014 ],
                                                        [0.0000 , 0.0029 , 0.0067 , 0.0113 , 0.0164 , 0.0218 , 0.0277 , 0.0349 , 0.0435 , 0.0533 , 0.0640 , 0.0757 , 0.0886 , 0.1027 , 0.1166 , 0.1298 , 0.1417 , 0.1534 , 0.1648 , 0.1760 , 0.1870 ]])
      self.MOT['PROPELLER']['Tables']['CN'] = self.MOT['PROPELLER']['Tables']['CN'] * (1+self.UNC['Res']['MOT']['Gain']['CN'])
      self.MOT['PROPELLER']['Tables']['CNfcn'] = RectBivariateSpline(self.MOT['PROPELLER']['Tables']['Alpha_deg'],
                                                                     self.MOT['PROPELLER']['Tables']['J'],
                                                                     self.MOT['PROPELLER']['Tables']['CN'],
                                                                     kx=1, ky=1)
      

      self.MOT['PROPELLER']['Tables']['CP'] = np.array([[0.0439 , 0.0584 , 0.0706 , 0.0790 , 0.0846 , 0.0895 , 0.0936 , 0.0956 , 0.0958 , 0.0952 , 0.0934 , 0.0907 , 0.0875 , 0.0831 , 0.0775 , 0.0709 , 0.0640 , 0.0569 , 0.0495 , 0.0419 , 0.0340 ],
                                                        [0.0439 , 0.0555 , 0.0664 , 0.0753 , 0.0831 , 0.0890 , 0.0932 , 0.0969 , 0.1004 , 0.1042 , 0.1061 , 0.1067 , 0.1064 , 0.1056 , 0.1040 , 0.1017 , 0.0994 , 0.0974 , 0.0958 , 0.0947 , 0.0939 ],
                                                        [0.0439 , 0.0545 , 0.0657 , 0.0760 , 0.0845 , 0.0928 , 0.1013 , 0.1096 , 0.1179 , 0.1269 , 0.1340 , 0.1391 , 0.1426 , 0.1457 , 0.1480 , 0.1499 , 0.1514 , 0.1532 , 0.1555 , 0.1584 , 0.1617 ],
                                                        [0.0439 , 0.0570 , 0.0694 , 0.0803 , 0.0910 , 0.1014 , 0.1119 , 0.1244 , 0.1389 , 0.1554 , 0.1710 , 0.1862 , 0.2012 , 0.2164 , 0.2295 , 0.2407 , 0.2499 , 0.2593 , 0.2694 , 0.2803 , 0.2919 ],
                                                        [0.0439 , 0.0572 , 0.0687 , 0.0784 , 0.0901 , 0.1038 , 0.1194 , 0.1380 , 0.1599 , 0.1857 , 0.2107 , 0.2346 , 0.2570 , 0.2798 , 0.3027 , 0.3257 , 0.3493 , 0.3743 , 0.4010 , 0.4296 , 0.4594 ],
                                                        [0.0439 , 0.0550 , 0.0657 , 0.0762 , 0.0908 , 0.1096 , 0.1324 , 0.1577 , 0.1851 , 0.2145 , 0.2433 , 0.2720 , 0.3006 , 0.3308 , 0.3622 , 0.3947 , 0.4278 , 0.4613 , 0.4949 , 0.5282 , 0.5613 ],
                                                        [0.0439 , 0.0556 , 0.0673 , 0.0793 , 0.0960 , 0.1173 , 0.1429 , 0.1712 , 0.2019 , 0.2347 , 0.2680 , 0.3022 , 0.3375 , 0.3738 , 0.4073 , 0.4379 , 0.4653 , 0.4932 , 0.2680 , 0.3022 , 0.3375 ],
                                                        [0.0439 , 0.0555 , 0.0681 , 0.0821 , 0.0998 , 0.1215 , 0.1473 , 0.1780 , 0.2120 , 0.2485 , 0.2831 , 0.3169 , 0.3510 , 0.3868 , 0.4226 , 0.4580 , 0.4927 , 0.5284 , 0.5652 , 0.6030 , 0.6414 ]])
      self.MOT['PROPELLER']['Tables']['CP'] = self.MOT['PROPELLER']['Tables']['CP'] * (1+self.UNC['Res']['MOT']['Gain']['CP'])
      self.MOT['PROPELLER']['Tables']['CPfcn'] = RectBivariateSpline(self.MOT['PROPELLER']['Tables']['Alpha_deg'],
                                                                     self.MOT['PROPELLER']['Tables']['J'],
                                                                     self.MOT['PROPELLER']['Tables']['CP'],
                                                                     kx=1, ky=1)
      self.MOT['PROPELLER']['M_kg']       = np.ones(self.MOT['n_motor']) * 0.526
      self.MOT['PROPELLER']['I_kgm2']     = self.MOT['PROPELLER']['M_kg']  * self.MOT['PROPELLER']['Diameter_m']**2 / 12

      self.MOT['TiltSurf_link']  = np.array([0,0,0,0,1,1,1,1])                 #ID of surface which the motor is linked. Every motor will rotate the same amount
      
      self.MOT['Bandwidth_radps'] = 40 * (1+self.UNC['Res']['MOT']['Gain']['Bandwidth'])
      self.MOT['Beta'] = np.exp(-self.MOT['Bandwidth_radps']*self.t_step)
      

      self.MOT['MaxV_V']      = 450
      self.MOT['imax_A']      = 500 #maximum constant is 250
      self.MOT['i0_A']        = 0
      self.MOT['R_ohm']       = 15*1e-3
      self.MOT['Kq_A_Nm']     = 1/0.75  * (1+self.UNC['Res']['MOT']['Gain']['Kq'])
      self.MOT['Kv_rpm_V']    = 6.53    * (1+self.UNC['Res']['MOT']['Gain']['Kv'])
      self.MOT['dt']          = 0.001 
      
      # Init Objects        
      self.MOT['ESC']['obj'] = []
      self.MOT['MOTOR']['obj'] = []
      self.MOT['PROPELLER']['obj'] = []
      self.MOT['ASSEMBLY']['obj'] = []
      for i in range(self.MOT['n_motor']):
        self.MOT['ESC']['obj'].append(MotorESC(KP = 0.5, KI = 50, KD = 0, MaxV_V = self.MOT['MaxV_V'], ESC_dt = self.MOT['dt']))
        self.MOT['MOTOR']['obj'].append(ElectricalMotor(Kq_A_Nm = self.MOT['Kq_A_Nm'], Kv_rpm_V = self.MOT['Kv_rpm_V'], i0_A = self.MOT['i0_A'], R_ohm = self.MOT['R_ohm'], imax_A = self.MOT['imax_A']))
        self.MOT['PROPELLER']['obj'].append(Propeller(Tables = self.MOT['PROPELLER']['Tables'],
                                                      I_kgm2 = self.MOT['PROPELLER']['I_kgm2'][i],
                                                      Diam_m = self.MOT['PROPELLER']['Diameter_m'][i],
                                                      Sim_dt = self.MOT['dt']))
        self.MOT['ASSEMBLY']['obj'].append(MotorAssembly(self.MOT['ESC']['obj'][i],
                                                         self.MOT['MOTOR']['obj'][i], 
                                                         self.MOT['PROPELLER']['obj'][i],
                                                         self.t_step, self.MOT['dt']))
      
    def init_CONT (self):
      
      self.CONT['Actuators'] = {}
      
      # Wing Tilt
      self.CONT['n_TiltSurf']    = 2
      self.CONT['MinTilt_deg']   = np.ones(self.CONT['n_TiltSurf']) * 0
      self.CONT['MaxTilt_deg']   = np.ones(self.CONT['n_TiltSurf']) * 90
      self.CONT['TiltRange_deg'] = self.CONT['MaxTilt_deg'] - self.CONT['MinTilt_deg'] 
      
      self.CONT['Actuators']['WingTilt'] = {}
      self.CONT['Actuators']['WingTilt']['CutFreq_radps'] = np.ones(self.CONT['n_TiltSurf']) * 20 * (1+self.UNC['Res']['CONT']['Gain']['WingTilt_Bandwidth'])
      self.CONT['Actuators']['WingTilt']['MaxRate']       = np.ones(self.CONT['n_TiltSurf']) * 10 * (1+self.UNC['Res']['CONT']['Gain']['WingTilt_Rate'])
      self.CONT['Actuators']['WingTilt']['t_act']         = np.ones(self.CONT['n_TiltSurf']) * 0.001
      self.CONT['Actuators']['WingTilt']['Actuators'] = []
     
      for i in range(self.CONT['n_TiltSurf']):
          self.CONT['Actuators']['WingTilt']['Actuators'].append(Actuator(CutFreq_radps = self.CONT['Actuators']['WingTilt']['CutFreq_radps'][i],
                                                                 MaxRate = self.CONT['Actuators']['WingTilt']['MaxRate'][i], 
                                                                 time_sample_actuator = self.CONT['Actuators']['WingTilt']['t_act'][i], 
                                                                 time_sample_sim = self.t_step))
      
      # Elevons
      self.CONT['n_elev'] = 4
      self.CONT['MinElev_deg']   = np.ones(self.CONT['n_elev']) * -15
      self.CONT['MaxElev_deg']   = np.ones(self.CONT['n_elev']) * +15
      self.CONT['ElevRange_deg'] = self.CONT['MaxElev_deg'] - self.CONT['MinElev_deg'] 
      self.CONT['ElevCenter_deg'] = (self.CONT['MinElev_deg'] + self.CONT['MaxElev_deg']) / 2

      self.CONT['Actuators']['Elevon'] = {}
      self.CONT['Actuators']['Elevon']['CutFreq_radps'] = np.ones(self.CONT['n_elev']) * 40 * (1+self.UNC['Res']['CONT']['Gain']['Elevon_Bandwidth'])
      self.CONT['Actuators']['Elevon']['MaxRate']       = np.ones(self.CONT['n_elev']) * 20 * (1+self.UNC['Res']['CONT']['Gain']['Elevon_Rate'])
      self.CONT['Actuators']['Elevon']['t_act']         = np.ones(self.CONT['n_elev']) * 0.001
      self.CONT['Actuators']['Elevon']['Actuators'] = []
     
      for i in range(self.CONT['n_elev']):
          self.CONT['Actuators']['Elevon']['Actuators'].append(Actuator(CutFreq_radps = self.CONT['Actuators']['Elevon']['CutFreq_radps'][i],
                                                                      MaxRate = self.CONT['Actuators']['Elevon']['MaxRate'][i], 
                                                                      time_sample_actuator = self.CONT['Actuators']['Elevon']['t_act'][i], 
                                                                      time_sample_sim = self.t_step))
      
    def init_SENS (self):
        
        self.SENS['Data'] = {}
        
        self.SENS['Data']['IMU'] = {}      
        self.SENS['Data']['IMU']['Delay_s'] = 0.01 * (1+self.UNC['Res']['SENS']['Gain']['IMU_Delay'])
        self.SENS['Data']['IMU']['CutFreq_radps'] = 40 * (1+self.UNC['Res']['SENS']['Gain']['IMU_Bandwidth'])
        
        self.SENS['Data']['ADS'] = {}      
        self.SENS['Data']['ADS']['Delay_s'] = 0.100 * (1+self.UNC['Res']['SENS']['Gain']['ADS_Delay'])
        self.SENS['Data']['ADS']['CutFreq_radps'] = 20 * (1+self.UNC['Res']['SENS']['Gain']['ADS_Bandwidth'])
        
        
        self.SENS['Sensors'] = {}
        self.SENS['Sensors']['IMU'] = {}
        self.SENS['Sensors']['ADS'] = {}
        
        self.SENS['Sensors']['IMU']['P_radps'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                  Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                  time_sample_sensor = 0.001,
                                                  time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['Q_radps'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                  Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                  time_sample_sensor = 0.001,
                                                  time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['R_radps'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                  Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                  time_sample_sensor = 0.001,
                                                  time_sample_sim = self.t_step)
        
        self.SENS['Sensors']['IMU']['Phi_rad'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                    Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                    time_sample_sensor = 0.001,
                                                    time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['Theta_rad'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                      Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                      time_sample_sensor = 0.001,
                                                      time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['Psi_rad'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                    Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                    time_sample_sensor = 0.001,
                                                    time_sample_sim = self.t_step)
        
        self.SENS['Sensors']['IMU']['VX_mps'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['VY_mps'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['VZ_mps'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)

        self.SENS['Sensors']['IMU']['X_m'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['Y_m'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['Z_m'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        
        self.SENS['Sensors']['IMU']['NX_mps2'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['NY_mps2'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        self.SENS['Sensors']['IMU']['NZ_mps2'] = Sensor(CutFreq_radps = self.SENS['Data']['IMU']['CutFreq_radps'],
                                                   Delay_s = self.SENS['Data']['IMU']['Delay_s'],
                                                   time_sample_sensor = 0.001,
                                                   time_sample_sim = self.t_step)
        
        self.SENS['Sensors']['ADS']['CAS_mps'] = Sensor(CutFreq_radps = self.SENS['Data']['ADS']['CutFreq_radps'],
                                                    Delay_s = self.SENS['Data']['ADS']['Delay_s'],
                                                    time_sample_sensor = 0.001,
                                                    time_sample_sim = self.t_step)
        
    # %% GENERAL FUNCTIONS
    def RotationMatrix(self,Phi_rad,Theta_rad,Psi_rad):
        
        LE2B = np.array([
            [+np.cos(Theta_rad)*np.cos(Psi_rad)                                                   , +np.cos(Theta_rad)*np.sin(Psi_rad)                                                   , -np.sin(Theta_rad)],
            [+np.sin(Phi_rad)*np.sin(Theta_rad)*np.cos(Psi_rad) - np.cos(Phi_rad)*np.sin(Psi_rad) , +np.cos(Phi_rad)*np.cos(Psi_rad) + np.sin(Phi_rad)*np.sin(Theta_rad)*np.sin(Psi_rad) , +np.sin(Phi_rad)*np.cos(Theta_rad)],
            [+np.cos(Phi_rad)*np.sin(Theta_rad)*np.cos(Psi_rad) + np.sin(Phi_rad)*np.sin(Psi_rad) , -np.sin(Phi_rad)*np.cos(Psi_rad) + np.cos(Phi_rad)*np.sin(Theta_rad)*np.sin(Psi_rad) , +np.cos(Phi_rad)*np.cos(Theta_rad)]])
        
        LB2E = np.transpose(LE2B)
        
        return LE2B,LB2E

    def EulerRotation(self,Phi_rad,Theta_rad,Psi_rad):
        
        LR2E = np.array([
                  [1 , np.sin(Phi_rad)*np.tan(Theta_rad) , np.cos(Phi_rad)*np.tan(Theta_rad)],
                  [0 , np.cos(Phi_rad)                   , -np.sin(Phi_rad)],
                  [0 , np.sin(Phi_rad)/np.cos(Theta_rad) , np.cos(Phi_rad)/np.cos(Theta_rad)]])
        
        return LR2E
    
    def cosd(self,theta):
        return np.cos(np.deg2rad(theta))
    
    def sind(self,theta):
        return np.sin(np.deg2rad(theta))   
    
    def NearZeroArray(self,Array,Saturation):
        NewArray = Array.copy()
        for i in range(np.size(NewArray)):
            if abs(NewArray[i]) < Saturation:
                NewArray[i] = 0
        
        return NewArray
    
    # %% EQM FUNCTION
    def EQM_fcn(self,F_b,M_b,I,m, CalcStaDot = True):
        """
        Status da função:
            Completa - ok
            Funcionando - ok
            Testada - nok
        Inputs:
            F_b: vetor de forças no eixo do corpo
            M_b: vetor de momentos no eixo do corpo
            I: tensor de inercias
            m: massa do corpo
            sta: vetor de estados na ordem:
                XL_e: vetor posição lineares no eixo da terra (X, Y, Z)
                XR_e: vetor posição rotacional no eixo da terra (phi, theta psi)
                VL_b: vetor velocidades lineares no eixo do corpo (u,v,w)
                VR_b: vetor velocidade rotacional no eixo do corpo (p, q, r)
                
        Outputs:
            sta_dot: vetor de derivadas de estados na ordem:
                dXL_e: vetor velocidades lineares no eixo da terra (X_dot, Y_dot, Z_dot)
                dXR_e: vetor velocidade rotacional no eixo da terra   (phi_dot, theta_dot psi_dot)      
                dVL_b: aceleração linear no eixo do corpo (u_dot,v_dot,w_dot)
                dVR_b: aceleração rotacional no eixo do corpo (p_dot, q_dot, r_dot)
            
        Descrição
            Função geral das equações de movimento do corpo rígido qualquer.
            Com massas, inercias, forças e momentos, calcula as derivadas doos estados
            É uma função genérica para qualquer corpo rídido no espaço
        """
        self.EQM['sta_names'] = ['X_m'    , 'Y_m'      , 'Z_m'    ,
                                 'Phi_rad', 'Theta_rad', 'Psi_rad',
                                 'U_mps'  , 'V_mps'    , 'W_mps'  ,
                                 'P_radps', 'Q_radps'  , 'R_radps']
        
        # LEITURA DOS VETORES NOS ESTADOS
        VL_b = self.EQM['sta'][6:9]
        VR_b = self.EQM['sta'][9:12]
        XL_e = self.EQM['sta'][0:3]
        XR_e = self.EQM['sta'][3:6]
        
        # ROTATION MATRIX
        self.EQM['LE2B'],self.EQM['LB2E'] = self.RotationMatrix(XR_e[0],XR_e[1],XR_e[2])
        self.EQM['LR2E']                  = self.EulerRotation(XR_e[0],XR_e[1],XR_e[2])
        
        
        # OUTPUT
        self.EQM['VelLin_BodyAx_mps']   = VL_b
        self.EQM['VelRot_BodyAx_radps'] = VR_b
        self.EQM['PosLin_EarthAx_m']    = XL_e 
        self.EQM['EulerAngles_rad']     = XR_e 
        
        if CalcStaDot:
            # Vetor Peso no Eixo Corpo
            W_b = np.dot(self.EQM['LE2B'],np.array([0,0,m*self.CONS['g_mps2']]))
            
            # CALCULO DAS DERIVADAS
            dVL_b = (F_b+W_b)/m - np.cross(VR_b,VL_b)
                    
            dVR_b = np.dot(np.linalg.inv(I),(M_b - np.cross(VR_b,np.dot(I,VR_b))))
            
            dXL_e = np.dot(self.EQM['LB2E'] , VL_b)
            
            dXR_e = np.dot(self.EQM['LR2E'] , VR_b)    

            # OUTPUT
            self.EQM['AccLin_BodyAx_mps2']   = dVL_b
            self.EQM['AccRot_BodyAx_radps2'] = dVR_b
            self.EQM['VelLin_EarthAx_mps']   = dXL_e   
            self.EQM['AccLin_EarthAx_mps2']  = np.dot(self.EQM['LB2E'] , dVL_b)
            self.EQM['EulerAnglesDot_radps'] = dXR_e 
    
            self.EQM['LoadFactor_mps2'] = (F_b)/m 

            # VETOR SAIDA COM OS ESTADOS
            self.EQM['sta_dot'][6:9]  = dVL_b
            self.EQM['sta_dot'][9:12] = dVR_b
            self.EQM['sta_dot'][0:3]  = dXL_e
            self.EQM['sta_dot'][3:6]  = np.array([self.OPT['EnableRoll'],self.OPT['EnablePitch'] ,self.OPT['EnableYaw'] ]) * dXR_e
            
    # %% ATMOSPHERE FUNCTION
    def ATM_fcn(self):
        """
        Status da função:
            Completa - ok
            Funcionando - ok
            Testada - nok
            
        Input:
            Altitude_m: Flight Geomtric Altitude
            dISA: delta ISA (Celsius)
            UVW_mps: body speeds
            TAS_mps: True Airspeed (em m/s)
            
        Description: function to calculate Atmosphere variables using the Standard
                     Atmosphere model.
        """
        # CONSTANTS
        Altitude_m     = -self.EQM['sta'][2]
        self.ATM['PresAlt_ft'] = Altitude_m / 0.3048
        
        # WIND VECTOR
        WindVec_kt = np.array([self.ATM['WindX_kt'],self.ATM['WindY_kt'] ,self.ATM['WindZ_kt'] ])
        WindVec_mps = WindVec_kt * self.CONS['kt2mps']
        # INITIAL CALCULATIIONS
        
        if Altitude_m < 11000:
            self.ATM['TStd_C'] = self.ATM['Const']['T0_C'] - 0.0065*Altitude_m
            self.ATM['T_C']    = self.ATM['TStd_C'] + self.ATM['dISA_C']
            self.ATM['P_Pa']   = self.ATM['Const']['P0_Pa']*(1-0.0065*Altitude_m/self.ATM['Const']['T0_K'])**5.2561
        else:
            self.ATM['TStd_C'] = -56.5
            self.ATM['T_C']    = self.ATM['TStd_C'] + self.ATM['dISA_C']
            self.ATM['P_Pa']   = 22632 * np.exp(-self.CONS['g_mps2']/(self.ATM['Const']['R']*216.65)*(Altitude_m-11000))
        
        self.ATM['rho_kgm3']   = self.ATM['P_Pa'] / (self.ATM['Const']['R']*(self.ATM['T_C'] + self.ATM['Const']['C2K']))
        self.ATM['Vsound_mps'] = np.sqrt(1.4*self.ATM['Const']['R']*(self.ATM['T_C']+self.ATM['Const']['C2K']))
        self.ATM['TAS2EAS']    = np.sqrt(self.ATM['rho_kgm3']/self.ATM['Const']['rho0_kgm3'])
                    
        self.ATM['Vaero']      = np.dot(self.EQM['LE2B'] , WindVec_mps) + self.EQM['VelLin_BodyAx_mps']
        self.ATM['TAS_mps']    = np.linalg.norm(self.ATM['Vaero'])

        self.ATM['EAS_mps']    = self.ATM['TAS_mps'] * self.ATM['TAS2EAS']
        self.ATM['Mach']       = self.ATM['TAS_mps'] / self.ATM['Vsound_mps']

        self.ATM['DynPres_Pa']    = self.ATM['EAS_mps'] **2 * self.ATM['rho_kgm3'] / 2
        
        self.ATM['qc']         = self.ATM['P_Pa'] *((1+0.2*self.ATM['Mach']**2)**(7/2)-1)
        self.ATM['CAS_mps']    = self.ATM['Const']['Vsound0_mps']*np.sqrt(5*((self.ATM['qc'] /self.ATM['Const']['P0_Pa']+1)**(2/7)-1))

        
        if (abs(self.ATM['Vaero'][0]) < 1e-2 and not(self.trimming)):
            u = 0
        else:
            u = self.ATM['Vaero'][0]
            
        if (abs(self.ATM['Vaero'][1]) < 1e-2 and not(self.trimming)):
            v = 0
        else:
            v = self.ATM['Vaero'][1]
            
        if (abs(self.ATM['Vaero'][2]) < 1e-2 and not(self.trimming)):
            w = 0
        else:
            w = self.ATM['Vaero'][2]
            
        self.ATM['Alpha_rad'] = np.arctan2(w,u)
        self.ATM['Alpha_deg'] = np.rad2deg(self.ATM['Alpha_rad'])
       
        Beta_aux  = np.rad2deg(
                                np.arctan2(v*np.cos(np.deg2rad(self.ATM['Alpha_deg'] )),u))      
        
        self.ATM['Beta_deg'] = Beta_aux * np.sign(self.cosd(Beta_aux))         #sign correction to consider backward flight (AOA = 180)
       
    # %% MOTOR MODEL
    def MOT_fcn(self):            
        # Calcular Tracao/Torque de Cada Helice
        # Calcular Fp de cada hélice
        # Calcular Torque devido a inercia (conservacao momento angular)
        
        # Calculate induced angles and speeds
        self.MOT['Tilt_deg']   = self.CONT['Tilt_deg'][self.MOT['TiltSurf_link']]
        self.MOT['Tilt_rad'] = np.deg2rad(self.MOT['Tilt_deg'])
        
        # Calculate Induced and Total Airflow Velocities (body axis) due to 
        # Aircraft Rotation (p,q,r)
        MOT_Vind      = np.zeros([self.MOT['n_motor'],3])
        MOT_VTotal_b  = np.zeros([self.MOT['n_motor'],3])
        MOT_Vind      = np.cross(self.EQM['VelRot_BodyAx_radps'],(self.MOT['Position_m'] - self.MASS['CG_m']))
        MOT_VTotal_b  = np.add(self.ATM['Vaero'],MOT_Vind)
        
        
        # Calculate Total Airflow Velocities in propeller axis   
        LM2B = np.zeros([3,3,self.MOT['n_motor']])
        LB2M = np.zeros([3,3,self.MOT['n_motor']])
        MOT_VTotal_p  = np.zeros([self.MOT['n_motor'],3])
        
        for i in range(self.MOT['n_motor']):
            LM2B[:,:,i] = np.array([[+np.cos(-self.MOT['Tilt_rad'][i])*np.cos(0) , -np.cos(-self.MOT['Tilt_rad'][i])*np.sin(0) , -np.sin(-self.MOT['Tilt_rad'][i]) ],
                                    [+np.sin(0)                                  , +np.cos(0)                                 , 0                                ],
                                    [+np.sin(-self.MOT['Tilt_rad'][i])*np.cos(0)  , -np.sin(-self.MOT['Tilt_rad'][i])*np.sin(0) , +np.cos(-self.MOT['Tilt_rad'][i]) ]])
            LB2M[:,:,i] = np.transpose(LM2B[:,:,i]) 
            
            MOT_VTotal_p[i,:] = np.dot(LB2M[:,:,i] , MOT_VTotal_b[i,:])
        
        # Calculate Angle of attack of propeller axis 
        Denominator = self.NearZeroArray(MOT_VTotal_p[:,0],1e-4)
        self.MOT['Alpha_deg'] = np.rad2deg(np.arctan2(MOT_VTotal_p[:,2],Denominator))
        self.MOT['Beta_deg']  = np.rad2deg(np.arctan2(MOT_VTotal_p[:,1]*np.cos(np.deg2rad(self.MOT['Alpha_deg'])),
                                                          Denominator))
        
        # Calculate RPM and Rotation of each Propeller
        if self.trimming:   
            # Initialize arrays
            self.MOT['RPM'] = np.zeros(self.MOT['n_motor'])
            self.MOT['Thrust_N'] = np.zeros(self.MOT['n_motor'])
            self.MOT['Normal_N'] = np.zeros(self.MOT['n_motor'])
            self.MOT['Torque_Nm'] = np.zeros(self.MOT['n_motor'])
            
            # Calculate RPM
            for i in range(self.MOT['n_motor']):
                self.MOT['ASSEMBLY']['obj'][i].set_zero(self.CONT['Throttle_p'][i] , MOT_VTotal_p[i,0] , self.ATM['rho_kgm3'])
                self.MOT['RPM'][i] = self.MOT['ASSEMBLY']['obj'][i].RPM
        else:
            for i in range(self.MOT['n_motor']):
                self.MOT['ASSEMBLY']['obj'][i].step(self.CONT['Throttle_p'][i] , MOT_VTotal_p[i,0], self.ATM['rho_kgm3'])
                self.MOT['RPM'][i] = self.MOT['ASSEMBLY']['obj'][i].RPM
                
        self.MOT['RPS'] = self.MOT['RPM'] / 60
        
        
        # Calculate Thrust and Torque
        for i in range(self.MOT['n_motor']):
            self.MOT['Thrust_N'][i]  = self.MOT['ASSEMBLY']['obj'][i].PROPELLER.Thrust_N
            self.MOT['Normal_N'][i]  = self.MOT['ASSEMBLY']['obj'][i].PROPELLER.Normal_N
            self.MOT['Torque_Nm'][i] = self.MOT['ASSEMBLY']['obj'][i].PROPELLER.Torque_Nm
        
        self.MOT['Force_BodyAx_N'] = np.zeros([self.MOT['n_motor'],3])
        self.MOT['Moment_BodyAx_N'] = np.zeros([self.MOT['n_motor'],3])
        
        for i in range(self.MOT['n_motor']):
            self.MOT['Force_BodyAx_N'][i,:]  = np.dot(LM2B[:,:,i],np.array([self.MOT['Thrust_N'][i],0,self.MOT['Normal_N'][i]]))
            r    = self.MOT['Position_m'][i,:] - self.MASS['CG_m']
            r[0] = -r[0]
            r[2] = -r[2]
            self.MOT['Moment_BodyAx_N'][i,:] = np.cross(r,self.MOT['Force_BodyAx_N'][i,:]) + np.dot(LM2B[:,:,i],np.array([self.MOT['Torque_Nm'][i],0,0]))*self.MOT['RotationSense'][i]
        
        self.MOT['TotalForce_BodyAx_N'] = np.sum(self.MOT['Force_BodyAx_N'] , axis = 0) * self.OPT['UsePropForce']
        self.MOT['TotalMoment_BodyAx_Nm'] = np.sum(self.MOT['Moment_BodyAx_N'] , axis = 0) * self.OPT['UsePropMoment']

        
    def AERO_fcn(self):
        
        def CalcInducedAOA(Xw_m,XCG_m,q_radps,TAS_mps,Inc_deg,AOA_Acft_deg,EPS_deg):
            '''
            Function to calculate the final Angle of Attack of surface (SurfaceAoA), considering
            Aircraft AOA (AOA_Acft_deg), surface incidence (Inc_deg), Downwash (EPS_deg),
            and induced angle due to pitch rate (q_degps)
            '''
            
            if abs(TAS_mps) < 1e-4:
                SurfaceAoA = (+AOA_Acft_deg
                              +Inc_deg
                              -EPS_deg
                              + np.rad2deg( np.arctan2(
                                            (Xw_m - XCG_m) * q_radps,
                                            0)))
            else:
                 SurfaceAoA = (+AOA_Acft_deg
                              +Inc_deg
                              -EPS_deg
                              + np.rad2deg( np.arctan2(
                                            (Xw_m - XCG_m) * q_radps,
                                            TAS_mps)))                   
            
            SurfaceAoA = np.mod(SurfaceAoA+180,360)-180
            return SurfaceAoA

        def CalcInducedBETA(Xw_m,XCG_m,r_radps,TAS_mps,Inc_deg,BETA_Acft_deg,Sidewash_deg):
            '''
            Function to calculate the final Angle of Sideslip of surface (SurfaceBETA), considering
            Aircraft Beta (BETA_Acft_deg), surface incidence (Inc_deg), Sidewash (Sidewash_deg),
            and induced angle due to yaw rate (r_radps)
            '''
            if abs(TAS_mps) < 1e-4:
                 SurfaceBETA = (+BETA_Acft_deg
                              -Inc_deg
                              +Sidewash_deg
                              - np.rad2deg( np.arctan2(
                                            (Xw_m - XCG_m) * r_radps,
                                            0)))
            else:
                 SurfaceBETA = (+BETA_Acft_deg
                              -Inc_deg
                              +Sidewash_deg
                              - np.rad2deg( np.arctan2(
                                            (Xw_m - XCG_m) * r_radps,
                                            TAS_mps)))
            
            SurfaceBETA = np.mod(SurfaceBETA+180,360)-180
            return SurfaceBETA

        def AuxAOA(Alpha_deg):
            if abs(Alpha_deg) < 90:
                Alpha_deg_aux = abs(Alpha_deg)
                sign_aux = np.sign(Alpha_deg)
            else:
                Alpha_deg_aux = abs(180-abs(Alpha_deg))
                sign_aux = -np.sign(Alpha_deg)
            
            return Alpha_deg_aux , sign_aux

        def FlatPlate_CL(Alpha_deg, Beta_deg):
            return self.sind(2*Alpha_deg)*abs(self.cosd(Beta_deg))

        def FlatPlate_CD(Alpha_deg, Beta_deg):
            return 2*(self.sind(Alpha_deg)**2)*abs(self.cosd(Beta_deg))

        def STAB2BODY (Alpha_rad,CDS,CYS,CLS,CRS,CMS,CNS):
            CDB = np.cos(Alpha_rad) * CDS - np.sin(Alpha_rad) * CLS
            CYB = CYS
            CLB = np.sin(Alpha_rad) * CDS + np.cos(Alpha_rad) * CLS

            CRB = np.cos(Alpha_rad) * CRS - np.sin(Alpha_rad) * CNS
            CMB = CMS
            CNB = np.sin(Alpha_rad) * CRS + np.cos(Alpha_rad) * CNS

            return CDB, CYB, CLB, CRB, CMB, CNB

        def BODYMRC2CG(XYZ_MRC, XYZ_CG, bref_m, cref_m, CXB_MRC, CYB_MRC, CZB_MRC, CDB_MRC, CLB_MRC, CRB_MRC, CMB_MRC, CNB_MRC):
            CXB_CG = CXB_MRC 
            CYB_CG = CYB_MRC 
            CZB_CG = CZB_MRC 
            CDB_CG = CDB_MRC 
            CLB_CG = CLB_MRC 
            CRB_CG = ( + CZB_CG * (XYZ_MRC[1] - XYZ_CG[1]) / bref_m + CYB_CG * (XYZ_MRC[2] - XYZ_CG[2]) / bref_m + CRB_MRC )
            CMB_CG = ( + CZB_CG * (XYZ_MRC[0] - XYZ_CG[0]) / cref_m - CXB_CG * (XYZ_MRC[2] - XYZ_CG[2]) / cref_m + CMB_MRC )
            CNB_CG = ( - CYB_CG * (XYZ_MRC[0] - XYZ_CG[0]) / bref_m - CXB_CG * (XYZ_MRC[1] - XYZ_CG[1]) / bref_m + CNB_MRC )

            return CXB_CG, CYB_CG, CZB_CG, CDB_CG, CLB_CG, CRB_CG, CMB_CG, CNB_CG
        
        LimitedTAS = max(self.ATM['TAS_mps'] , 10)
        # Calculate W1 local stability coefs
        self.AERO['Wing1']['Incidence_deg'] = self.CONT['Tilt_deg'][0]
        self.AERO['Wing1']['EPS_deg'] = 0.0

        self.AERO['Wing1']['Alpha_deg'] = CalcInducedAOA(self.GEOM['Wing1']['XYZ_m'][0],
                                                         self.MASS['CG_m'][0],
                                                         self.EQM['VelRot_BodyAx_radps'][1],
                                                         self.ATM['TAS_mps'],
                                                         self.AERO['Wing1']['Incidence_deg'],
                                                         self.ATM['Alpha_deg'],
                                                         self.AERO['Wing1']['EPS_deg'])
        
        self.AERO['Wing1']['Beta_deg'] = CalcInducedBETA(self.GEOM['Wing1']['XYZ_m'][0],
                                                         self.MASS['CG_m'][0],
                                                         self.EQM['VelRot_BodyAx_radps'][2],
                                                         self.ATM['TAS_mps'],
                                                         0,
                                                         self.ATM['Beta_deg'],
                                                         0)
    
        
        W1_Alpha_deg_aux, W1_sign_aux = AuxAOA(self.AERO['Wing1']['Alpha_deg'])
        W1_Beta_deg_aux = self.AERO['Wing1']['Beta_deg']

        if W1_Alpha_deg_aux <= self.AERO['Wing1']['Coefs']['Alpha_deg'][-1] and W1_Alpha_deg_aux >= self.AERO['Wing1']['Coefs']['Alpha_deg'][0]:
            self.AERO['Wing1']['CDS_25Local'] = np.interp(W1_Alpha_deg_aux, self.AERO['Wing1']['Coefs']['Alpha_deg'], self.AERO['Wing1']['Coefs']['CDS_25Local'])
            self.AERO['Wing1']['CLS_25Local'] = W1_sign_aux*np.interp(W1_Alpha_deg_aux, self.AERO['Wing1']['Coefs']['Alpha_deg'], self.AERO['Wing1']['Coefs']['CLS_25Local'])
            self.AERO['Wing1']['CMS_25Local'] = W1_sign_aux*np.interp(W1_Alpha_deg_aux, self.AERO['Wing1']['Coefs']['Alpha_deg'], self.AERO['Wing1']['Coefs']['CMS_25Local'])
        else:
            self.AERO['Wing1']['CDS_25Local'] = FlatPlate_CD(W1_Alpha_deg_aux, W1_Beta_deg_aux)
            self.AERO['Wing1']['CLS_25Local'] = W1_sign_aux*FlatPlate_CL(W1_Alpha_deg_aux, W1_Beta_deg_aux)
            self.AERO['Wing1']['CMS_25Local'] = 0
        
        self.AERO['Wing1']['CLS_25Local'] = (1+self.UNC['Res']['AERO']['Gain']['CLa']) * self.AERO['Wing1']['CLS_25Local']
        self.AERO['Wing1']['CMS_25Local'] = (self.UNC['Res']['AERO']['Bias']['CM0'] + self.AERO['Wing1']['CMS_25Local']
                                           + self.AERO['Wing1']['Coefs']['CMq'] * self.EQM['VelRot_BodyAx_radps'][1] * self.AERO['cref_m'] / (2* LimitedTAS))
        self.AERO['Wing1']['CYS_25Local'] = 0
        self.AERO['Wing1']['CRS_25Local'] = (self.AERO['Wing1']['Coefs']['CRp'] * self.EQM['VelRot_BodyAx_radps'][0] * self.AERO['bref_m'] / (2* LimitedTAS)
                                           + self.AERO['Wing1']['Coefs']['CRr'] * self.EQM['VelRot_BodyAx_radps'][2] * self.AERO['bref_m'] / (2* LimitedTAS))
        self.AERO['Wing1']['CNS_25Local'] = (self.AERO['Wing1']['Coefs']['CNp'] * self.EQM['VelRot_BodyAx_radps'][0] * self.AERO['bref_m'] / (2* LimitedTAS)
                                           + self.AERO['Wing1']['Coefs']['CNr'] * self.EQM['VelRot_BodyAx_radps'][2] * self.AERO['bref_m'] / (2* LimitedTAS))


        # Calculate W2 local stability coefs
        self.AERO['Wing2']['Incidence_deg'] = self.CONT['Tilt_deg'][1]
        self.AERO['Wing2']['EPS_deg'] = (self.AERO['Wing2']['EPS0']
                                      + self.AERO['Wing2']['dEPS_dCLW1'] * self.AERO['Wing1']['CLS_25Local']
                                      + self.AERO['Wing2']['dEPS_dAOAacft'] * self.ATM['Alpha_deg'])

        self.AERO['Wing2']['Alpha_deg'] = CalcInducedAOA(self.GEOM['Wing2']['XYZ_m'][0],
                                                         self.MASS['CG_m'][0],
                                                         self.EQM['VelRot_BodyAx_radps'][1],
                                                         self.ATM['TAS_mps'],
                                                         self.AERO['Wing2']['Incidence_deg'],
                                                         self.ATM['Alpha_deg'],
                                                         self.AERO['Wing2']['EPS_deg'])

        self.AERO['Wing2']['Beta_deg'] = CalcInducedBETA(self.GEOM['Wing2']['XYZ_m'][0],
                                                         self.MASS['CG_m'][0],
                                                         self.EQM['VelRot_BodyAx_radps'][2],
                                                         self.ATM['TAS_mps'],
                                                         0,
                                                         self.ATM['Beta_deg'],
                                                         0)

        W2_Alpha_deg_aux, W2_sign_aux = AuxAOA(self.AERO['Wing2']['Alpha_deg'])
        W2_Beta_deg_aux = self.AERO['Wing2']['Beta_deg']


        if W2_Alpha_deg_aux <= self.AERO['Wing2']['Coefs']['Alpha_deg'][-1] and W2_Alpha_deg_aux >= self.AERO['Wing2']['Coefs']['Alpha_deg'][0]:
            self.AERO['Wing2']['CDS_25Local'] = np.interp(W2_Alpha_deg_aux, self.AERO['Wing2']['Coefs']['Alpha_deg'], self.AERO['Wing2']['Coefs']['CDS_25Local'])
            self.AERO['Wing2']['CLS_25Local'] = W2_sign_aux*np.interp(W2_Alpha_deg_aux, self.AERO['Wing2']['Coefs']['Alpha_deg'], self.AERO['Wing2']['Coefs']['CLS_25Local'])
            self.AERO['Wing2']['CMS_25Local'] = W2_sign_aux*np.interp(W2_Alpha_deg_aux, self.AERO['Wing2']['Coefs']['Alpha_deg'], self.AERO['Wing2']['Coefs']['CMS_25Local'])
        else:
            self.AERO['Wing2']['CDS_25Local'] = FlatPlate_CD(W2_Alpha_deg_aux, W2_Beta_deg_aux)
            self.AERO['Wing2']['CLS_25Local'] = W2_sign_aux*FlatPlate_CL(W2_Alpha_deg_aux, W2_Beta_deg_aux)
            self.AERO['Wing2']['CMS_25Local'] = 0

        self.AERO['Wing2']['CLS_25Local'] = (1+self.UNC['Res']['AERO']['Gain']['CLa']) * self.AERO['Wing2']['CLS_25Local']
        self.AERO['Wing2']['CMS_25Local'] = (self.UNC['Res']['AERO']['Bias']['CM0'] + self.AERO['Wing2']['CMS_25Local']
                                           + self.AERO['Wing2']['Coefs']['CMq'] * self.EQM['VelRot_BodyAx_radps'][1] * self.AERO['cref_m'] / (2* LimitedTAS))
        self.AERO['Wing2']['CYS_25Local'] = 0
        self.AERO['Wing2']['CRS_25Local'] = (self.AERO['Wing2']['Coefs']['CRp'] * self.EQM['VelRot_BodyAx_radps'][0] * self.AERO['bref_m'] / (2* LimitedTAS)
                                           + self.AERO['Wing2']['Coefs']['CRr'] * self.EQM['VelRot_BodyAx_radps'][2] * self.AERO['bref_m'] / (2* LimitedTAS))
        self.AERO['Wing2']['CNS_25Local'] = (self.AERO['Wing2']['Coefs']['CNp'] * self.EQM['VelRot_BodyAx_radps'][0] * self.AERO['bref_m'] / (2* LimitedTAS)
                                           + self.AERO['Wing2']['Coefs']['CNr'] * self.EQM['VelRot_BodyAx_radps'][2] * self.AERO['bref_m'] / (2* LimitedTAS))

        # Calculate Fuselage local stability coefs
        self.AERO['Fus']['Beta_deg'] = self.ATM['Beta_deg']

        self.AERO['Fus']['CDS_25Local'] = self.AERO['Fus']['Coefs']['CD0']
        self.AERO['Fus']['CLS_25Local'] = 0
        self.AERO['Fus']['CMS_25Local'] = self.AERO['Fus']['Coefs']['CMq'] * self.EQM['VelRot_BodyAx_radps'][1] * self.AERO['cref_m'] / (2* LimitedTAS)
        self.AERO['Fus']['CYS_25Local'] = self.AERO['Fus']['Coefs']['CYbeta'] * self.sind(self.AERO['Fus']['Beta_deg'])
        self.AERO['Fus']['CRS_25Local'] = 0
        self.AERO['Fus']['CNS_25Local'] = 0
        self.AERO['Fus']['CRS_25Local'] = (self.AERO['Fus']['Coefs']['CRp'] * self.EQM['VelRot_BodyAx_radps'][0] * self.AERO['bref_m'] / (2* LimitedTAS)
                                         + self.AERO['Fus']['Coefs']['CRr'] * self.EQM['VelRot_BodyAx_radps'][2] * self.AERO['bref_m'] / (2* LimitedTAS))
        self.AERO['Fus']['CNS_25Local'] = (self.AERO['Fus']['Coefs']['CNp'] * self.EQM['VelRot_BodyAx_radps'][0] * self.AERO['bref_m'] / (2* LimitedTAS)
                                         + self.AERO['Fus']['Coefs']['CNr'] * self.EQM['VelRot_BodyAx_radps'][2] * self.AERO['bref_m'] / (2* LimitedTAS))

        ElevonGain_1 = np.interp(self.AERO['Wing1']['Alpha_deg'],
                                 self.AERO['Elevon']['AOAeff']['Alpha_deg'],
                                 self.AERO['Elevon']['AOAeff']['Gain'])
        ElevonGain_2 = np.interp(self.AERO['Wing2']['Alpha_deg'],
                                 self.AERO['Elevon']['AOAeff']['Alpha_deg'],
                                 self.AERO['Elevon']['AOAeff']['Gain'])
        ElevonGain = np.array([ElevonGain_1 , ElevonGain_1 , ElevonGain_2 , ElevonGain_2])

        self.AERO['Elevon']['CDS_MRC']  = self.AERO['Elevon']['dCDSde_MRC'] * self.CONT['Elevon_deg'] * ElevonGain
        self.AERO['Elevon']['CYS_MRC']  = self.AERO['Elevon']['dCYSde_MRC'] * self.CONT['Elevon_deg'] * ElevonGain
        self.AERO['Elevon']['CLS_MRC']  = self.AERO['Elevon']['dCLSde_MRC'] * self.CONT['Elevon_deg'] * ElevonGain
        self.AERO['Elevon']['CRS_MRC']  = self.AERO['Elevon']['dCRSde_MRC'] * self.CONT['Elevon_deg'] * ElevonGain
        self.AERO['Elevon']['CMS_MRC']  = self.AERO['Elevon']['dCMSde_MRC'] * self.CONT['Elevon_deg'] * ElevonGain
        self.AERO['Elevon']['CNS_MRC']  = self.AERO['Elevon']['dCNSde_MRC'] * self.CONT['Elevon_deg'] * ElevonGain
        
        # Calculate Coefficcient in Body Local Axis
        (CDB, CYB, CLB, CRB, CMB, CNB) = STAB2BODY (self.ATM['Alpha_rad'],
                                                    self.AERO['Wing1']['CDS_25Local'],
                                                    self.AERO['Wing1']['CYS_25Local'],
                                                    self.AERO['Wing1']['CLS_25Local'],
                                                    self.AERO['Wing1']['CRS_25Local'],
                                                    self.AERO['Wing1']['CMS_25Local'],
                                                    self.AERO['Wing1']['CNS_25Local'])

        self.AERO['Wing1']['CDB_25Local'] = CDB
        self.AERO['Wing1']['CYB_25Local'] = CYB
        self.AERO['Wing1']['CLB_25Local'] = CLB
        self.AERO['Wing1']['CXB_25Local'] = -CDB
        self.AERO['Wing1']['CZB_25Local'] = -CLB
        self.AERO['Wing1']['CRB_25Local'] = CRB
        self.AERO['Wing1']['CMB_25Local'] = CMB
        self.AERO['Wing1']['CNB_25Local'] = CNB

        (CDB, CYB, CLB, CRB, CMB, CNB) = STAB2BODY (self.ATM['Alpha_rad'],
                                                    self.AERO['Wing2']['CDS_25Local'],
                                                    self.AERO['Wing2']['CYS_25Local'],
                                                    self.AERO['Wing2']['CLS_25Local'],
                                                    self.AERO['Wing2']['CRS_25Local'],
                                                    self.AERO['Wing2']['CMS_25Local'],
                                                    self.AERO['Wing2']['CNS_25Local'])

        self.AERO['Wing2']['CDB_25Local'] = CDB
        self.AERO['Wing2']['CYB_25Local'] = CYB
        self.AERO['Wing2']['CLB_25Local'] = CLB
        self.AERO['Wing2']['CXB_25Local'] = -CDB
        self.AERO['Wing2']['CZB_25Local'] = -CLB
        self.AERO['Wing2']['CRB_25Local'] = CRB
        self.AERO['Wing2']['CMB_25Local'] = CMB
        self.AERO['Wing2']['CNB_25Local'] = CNB

        (CDB, CYB, CLB, CRB, CMB, CNB) = STAB2BODY (self.ATM['Alpha_rad'],
                                                    self.AERO['Fus']['CDS_25Local'],
                                                    self.AERO['Fus']['CYS_25Local'],
                                                    self.AERO['Fus']['CLS_25Local'],
                                                    self.AERO['Fus']['CRS_25Local'],
                                                    self.AERO['Fus']['CMS_25Local'],
                                                    self.AERO['Fus']['CNS_25Local'])

        self.AERO['Fus']['CDB_25Local'] = CDB
        self.AERO['Fus']['CYB_25Local'] = CYB
        self.AERO['Fus']['CLB_25Local'] = CLB
        self.AERO['Fus']['CXB_25Local'] = -CDB
        self.AERO['Fus']['CZB_25Local'] = -CLB
        self.AERO['Fus']['CRB_25Local'] = CRB
        self.AERO['Fus']['CMB_25Local'] = CMB
        self.AERO['Fus']['CNB_25Local'] = CNB

        (CDB, CYB, CLB, CRB, CMB, CNB) = STAB2BODY (self.ATM['Alpha_rad'],
                                                    self.AERO['Elevon']['CDS_MRC'],
                                                    self.AERO['Elevon']['CYS_MRC'],
                                                    self.AERO['Elevon']['CLS_MRC'],
                                                    self.AERO['Elevon']['CRS_MRC'],
                                                    self.AERO['Elevon']['CMS_MRC'],
                                                    self.AERO['Elevon']['CNS_MRC'])

        self.AERO['Elevon']['CDB_MRC'] = CDB
        self.AERO['Elevon']['CYB_MRC'] = CYB
        self.AERO['Elevon']['CLB_MRC'] = CLB
        self.AERO['Elevon']['CXB_MRC'] = -CDB
        self.AERO['Elevon']['CZB_MRC'] = -CLB
        self.AERO['Elevon']['CRB_MRC'] = CRB
        self.AERO['Elevon']['CMB_MRC'] = CMB
        self.AERO['Elevon']['CNB_MRC'] = CNB

        # Calculate Coefficcient in Body CG Axis

        (CXB_CG, CYB_CG, CZB_CG, CDB_CG, CLB_CG, CRB_CG, CMB_CG, CNB_CG) = BODYMRC2CG(self.GEOM['Wing1']['XYZ_m'], self.MASS['CG_m'], self.AERO['bref_m'], self.AERO['cref_m'], 
                                                                                      self.AERO['Wing1']['CXB_25Local'], 
                                                                                      self.AERO['Wing1']['CYB_25Local'],
                                                                                      self.AERO['Wing1']['CZB_25Local'],
                                                                                      self.AERO['Wing1']['CDB_25Local'],
                                                                                      self.AERO['Wing1']['CLB_25Local'],
                                                                                      self.AERO['Wing1']['CRB_25Local'],
                                                                                      self.AERO['Wing1']['CMB_25Local'],
                                                                                      self.AERO['Wing1']['CNB_25Local'])
        self.AERO['Wing1']['CXB_CG'] = CXB_CG
        self.AERO['Wing1']['CYB_CG'] = CYB_CG
        self.AERO['Wing1']['CZB_CG'] = CZB_CG
        self.AERO['Wing1']['CDB_CG'] = CDB_CG
        self.AERO['Wing1']['CLB_CG'] = CLB_CG
        self.AERO['Wing1']['CRB_CG'] = CRB_CG
        self.AERO['Wing1']['CMB_CG'] = CMB_CG
        self.AERO['Wing1']['CNB_CG'] = CNB_CG

        (CXB_CG, CYB_CG, CZB_CG, CDB_CG, CLB_CG, CRB_CG, CMB_CG, CNB_CG) = BODYMRC2CG(self.GEOM['Wing2']['XYZ_m'], self.MASS['CG_m'], self.AERO['bref_m'], self.AERO['cref_m'], 
                                                                                      self.AERO['Wing2']['CXB_25Local'], 
                                                                                      self.AERO['Wing2']['CYB_25Local'],
                                                                                      self.AERO['Wing2']['CZB_25Local'],
                                                                                      self.AERO['Wing2']['CDB_25Local'],
                                                                                      self.AERO['Wing2']['CLB_25Local'],
                                                                                      self.AERO['Wing2']['CRB_25Local'],
                                                                                      self.AERO['Wing2']['CMB_25Local'],
                                                                                      self.AERO['Wing2']['CNB_25Local'])
        self.AERO['Wing2']['CXB_CG'] = CXB_CG
        self.AERO['Wing2']['CYB_CG'] = CYB_CG
        self.AERO['Wing2']['CZB_CG'] = CZB_CG
        self.AERO['Wing2']['CDB_CG'] = CDB_CG
        self.AERO['Wing2']['CLB_CG'] = CLB_CG
        self.AERO['Wing2']['CRB_CG'] = CRB_CG
        self.AERO['Wing2']['CMB_CG'] = CMB_CG
        self.AERO['Wing2']['CNB_CG'] = CNB_CG

        (CXB_CG, CYB_CG, CZB_CG, CDB_CG, CLB_CG, CRB_CG, CMB_CG, CNB_CG) = BODYMRC2CG(self.GEOM['Fus']['XYZ_m'], self.MASS['CG_m'], self.AERO['bref_m'], self.AERO['cref_m'], 
                                                                                      self.AERO['Fus']['CXB_25Local'], 
                                                                                      self.AERO['Fus']['CYB_25Local'],
                                                                                      self.AERO['Fus']['CZB_25Local'],
                                                                                      self.AERO['Fus']['CDB_25Local'],
                                                                                      self.AERO['Fus']['CLB_25Local'],
                                                                                      self.AERO['Fus']['CRB_25Local'],
                                                                                      self.AERO['Fus']['CMB_25Local'],
                                                                                      self.AERO['Fus']['CNB_25Local'])
        self.AERO['Fus']['CXB_CG'] = CXB_CG
        self.AERO['Fus']['CYB_CG'] = CYB_CG
        self.AERO['Fus']['CZB_CG'] = CZB_CG
        self.AERO['Fus']['CDB_CG'] = CDB_CG
        self.AERO['Fus']['CLB_CG'] = CLB_CG
        self.AERO['Fus']['CRB_CG'] = CRB_CG
        self.AERO['Fus']['CMB_CG'] = CMB_CG
        self.AERO['Fus']['CNB_CG'] = CNB_CG

        (CXB_CG, CYB_CG, CZB_CG, CDB_CG, CLB_CG, CRB_CG, CMB_CG, CNB_CG) = BODYMRC2CG(self.AERO['MRC_m'], self.MASS['CG_m'], self.AERO['bref_m'], self.AERO['cref_m'], 
                                                                                      self.AERO['Elevon']['CXB_MRC'], 
                                                                                      self.AERO['Elevon']['CYB_MRC'],
                                                                                      self.AERO['Elevon']['CZB_MRC'],
                                                                                      self.AERO['Elevon']['CDB_MRC'],
                                                                                      self.AERO['Elevon']['CLB_MRC'],
                                                                                      self.AERO['Elevon']['CRB_MRC'],
                                                                                      self.AERO['Elevon']['CMB_MRC'],
                                                                                      self.AERO['Elevon']['CNB_MRC'])
        self.AERO['Elevon']['CXB_CG'] = CXB_CG
        self.AERO['Elevon']['CYB_CG'] = CYB_CG
        self.AERO['Elevon']['CZB_CG'] = CZB_CG
        self.AERO['Elevon']['CDB_CG'] = CDB_CG
        self.AERO['Elevon']['CLB_CG'] = CLB_CG
        self.AERO['Elevon']['CRB_CG'] = CRB_CG
        self.AERO['Elevon']['CMB_CG'] = CMB_CG
        self.AERO['Elevon']['CNB_CG'] = CNB_CG

        # Calculate Surfaces Forces and Moments
        self.AERO['Wing1']['FXB_N']   = self.AERO['Wing1']['CXB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing1']['S_m2']
        self.AERO['Wing1']['FYB_N']   = self.AERO['Wing1']['CYB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing1']['S_m2']
        self.AERO['Wing1']['FZB_N']   = self.AERO['Wing1']['CZB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing1']['S_m2']
        self.AERO['Wing1']['MXB_Nm']  = self.AERO['Wing1']['CRB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing1']['S_m2'] * self.GEOM['Wing1']['b_m']
        self.AERO['Wing1']['MYB_Nm']  = self.AERO['Wing1']['CMB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing1']['S_m2'] * self.GEOM['Wing1']['cma_m']
        self.AERO['Wing1']['MZB_Nm']  = self.AERO['Wing1']['CNB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing1']['S_m2'] * self.GEOM['Wing1']['b_m']

        self.AERO['Wing2']['FXB_N']   = self.AERO['Wing2']['CXB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing2']['S_m2']
        self.AERO['Wing2']['FYB_N']   = self.AERO['Wing2']['CYB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing2']['S_m2']
        self.AERO['Wing2']['FZB_N']   = self.AERO['Wing2']['CZB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing2']['S_m2']
        self.AERO['Wing2']['MXB_Nm']  = self.AERO['Wing2']['CRB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing2']['S_m2'] * self.GEOM['Wing2']['b_m']
        self.AERO['Wing2']['MYB_Nm']  = self.AERO['Wing2']['CMB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing2']['S_m2'] * self.GEOM['Wing2']['cma_m']
        self.AERO['Wing2']['MZB_Nm']  = self.AERO['Wing2']['CNB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing2']['S_m2'] * self.GEOM['Wing2']['b_m']

        self.AERO['Fus']['FXB_N']   = self.AERO['Fus']['CXB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2']
        self.AERO['Fus']['FYB_N']   = self.AERO['Fus']['CYB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2']
        self.AERO['Fus']['FZB_N']   = self.AERO['Fus']['CZB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2']
        self.AERO['Fus']['MXB_Nm']  = self.AERO['Fus']['CRB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2'] * self.GEOM['Fus']['b_m']
        self.AERO['Fus']['MYB_Nm']  = self.AERO['Fus']['CMB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2'] * self.GEOM['Fus']['cma_m']
        self.AERO['Fus']['MZB_Nm']  = self.AERO['Fus']['CNB_CG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2'] * self.GEOM['Fus']['b_m']

        self.AERO['Elevon']['FXB_N']   = self.AERO['Elevon']['CXB_CG'] * self.ATM['DynPres_Pa'] * self.AERO['Sref_m2']
        self.AERO['Elevon']['FYB_N']   = self.AERO['Elevon']['CYB_CG'] * self.ATM['DynPres_Pa'] * self.AERO['Sref_m2']
        self.AERO['Elevon']['FZB_N']   = self.AERO['Elevon']['CZB_CG'] * self.ATM['DynPres_Pa'] * self.AERO['Sref_m2']
        self.AERO['Elevon']['MXB_Nm']  = self.AERO['Elevon']['CRB_CG'] * self.ATM['DynPres_Pa'] * self.AERO['Sref_m2'] * self.AERO['bref_m']
        self.AERO['Elevon']['MYB_Nm']  = self.AERO['Elevon']['CMB_CG'] * self.ATM['DynPres_Pa'] * self.AERO['Sref_m2'] * self.AERO['cref_m']
        self.AERO['Elevon']['MZB_Nm']  = self.AERO['Elevon']['CNB_CG'] * self.ATM['DynPres_Pa'] * self.AERO['Sref_m2'] * self.AERO['bref_m']

        # Calculate Total Forces and Moments
        self.AERO['TotalForce_BodyAx_N']  = self.OPT['UseAeroForce'] * (
                                            np.array([np.sum( self.AERO['Wing1']['FXB_N'] ),
                                                      np.sum( self.AERO['Wing1']['FYB_N'] ),
                                                      np.sum( self.AERO['Wing1']['FZB_N'] )]) +
                                            np.array([np.sum( self.AERO['Wing2']['FXB_N'] ),
                                                      np.sum( self.AERO['Wing2']['FYB_N'] ),
                                                      np.sum( self.AERO['Wing2']['FZB_N'] )]) +
                                            np.array([np.sum( self.AERO['Fus']['FXB_N'] ),
                                                      np.sum( self.AERO['Fus']['FYB_N'] ),
                                                      np.sum( self.AERO['Fus']['FZB_N'] )]) +
                                            np.array([np.sum( self.AERO['Elevon']['FXB_N'] ),
                                                      np.sum( self.AERO['Elevon']['FYB_N'] ),
                                                      np.sum( self.AERO['Elevon']['FZB_N'] )]))
                                            
        self.AERO['TotalMoment_BodyAx_Nm'] = self.OPT['UseAeroMoment'] * (
                                             np.array([np.sum( self.AERO['Wing1']['MXB_Nm'] ),
                                                       np.sum( self.AERO['Wing1']['MYB_Nm'] ),
                                                       np.sum( self.AERO['Wing1']['MZB_Nm'] )]) + 
                                             np.array([np.sum( self.AERO['Wing2']['MXB_Nm'] ),
                                                       np.sum( self.AERO['Wing2']['MYB_Nm'] ),
                                                       np.sum( self.AERO['Wing2']['MZB_Nm'] )]) + 
                                             np.array([np.sum( self.AERO['Fus']['MXB_Nm'] ),
                                                       np.sum( self.AERO['Fus']['MYB_Nm'] ),
                                                       np.sum( self.AERO['Fus']['MZB_Nm'] )]) +
                                             np.array([np.sum( self.AERO['Elevon']['MXB_Nm'] ),
                                                       np.sum( self.AERO['Elevon']['MYB_Nm'] ),
                                                       np.sum( self.AERO['Elevon']['MZB_Nm'] )]))

    def CONT_fcn(self,action_vec):
        def VerticalControlAllocation(u):    
          return np.array([+1,+1,+1,+1,+1,+1,+1,+1]) * (u+1)/2
      
        def PitchControlAllocation(u):    
            return np.array([+1,+1,+1,+1,-1,-1,-1,-1]) * u
        
        def RollControlAllocation(u):    
            return np.array([+1,+1,-1,-1,+1,+1,-1,-1]) * u
        
        def YawControlAllocation(u):    
            return np.array([-1,+1,-1,+1,+1,-1,+1,-1]) * u
        
        def ControlMixer(u_Vert,u_Pitch,u_Roll,u_Yaw):
            SUM_INP = np.sum(np.array([u_Vert,u_Pitch,u_Roll,u_Yaw]),axis=0)
            
            INP_SAT = np.min( np.vstack((SUM_INP,np.ones(8) )) , axis=0)
            INP_SAT = np.max( np.vstack((INP_SAT,np.zeros(8))) , axis=0)
            
            return INP_SAT
                        
        u_Vert = action_vec[self.action_names.index('Throttle')]
        u_Pitc = action_vec[self.action_names.index('PitchThrottle')]
        if self.UseLateralActions:
            u_Roll = action_vec[self.action_names.index('RollThrottle')]
            u_Yaw  = action_vec[self.action_names.index('YawThrottle')]
        else:
            u_Roll = 0
            u_Yaw  = 0
        
        if not(self.trimming):
            self.CONT['LastThrottle_p'] = self.CONT['Throttle_p'].copy()

        self.CONT['Throttle_p'] = ControlMixer(VerticalControlAllocation(u_Vert),PitchControlAllocation(u_Pitc),RollControlAllocation(u_Roll),YawControlAllocation(u_Yaw))
        
        TILT_vec = (np.array([action_vec[self.action_names.index('W1_Tilt')],
                              action_vec[self.action_names.index('W2_Tilt')]])+1)/2        
        self.CONT['Tilt_p']   = TILT_vec
        TiltCmd_deg = (self.CONT['MinTilt_deg'] + self.CONT['TiltRange_deg'] * self.CONT['Tilt_p'])  
       
        if self.trimming:
            self.CONT['Tilt_deg'] = TiltCmd_deg
            for i in range(self.CONT['n_TiltSurf']):
                self.CONT['Actuators']['WingTilt']['Actuators'][i].set_zero(TiltCmd_deg[i])
            
        else:
            for i in range(self.CONT['n_TiltSurf']):
                self.CONT['Tilt_deg'][i],v,a = self.CONT['Actuators']['WingTilt']['Actuators'][i].step(TiltCmd_deg[i])

        self.CONT['Tilt_rad'] = np.deg2rad(self.CONT['Tilt_deg'])
        
        
        self.CONT['TiltDiff_p']   = TILT_vec[0] - TILT_vec[1]
        
        if self.UseLateralActions:
            self.CONT['Elevon_p'] = np.array([action_vec[self.action_names.index('W1_Elevator')] - 0.5*action_vec[self.action_names.index('W1_Aileron')],
                                            action_vec[self.action_names.index('W1_Elevator')] + 0.5*action_vec[self.action_names.index('W1_Aileron')],
                                            action_vec[self.action_names.index('W2_Elevator')] - 0.5*action_vec[self.action_names.index('W2_Aileron')],
                                            action_vec[self.action_names.index('W2_Elevator')] + 0.5*action_vec[self.action_names.index('W2_Aileron')]])
        else:
            self.CONT['Elevon_p'] = np.array([0                                                  - 0.5*0,
                                            0                                                  + 0.5*0,
                                            action_vec[self.action_names.index('W2_Elevator')] - 0.5*0,
                                            action_vec[self.action_names.index('W2_Elevator')] + 0.5*0])
        
        ElevCmd_deg = (self.CONT['ElevCenter_deg'] + self.CONT['ElevRange_deg']/2 * self.CONT['Elevon_p'])
        if self.trimming:
            self.CONT['Elevon_deg'] = ElevCmd_deg
            for i in range(self.CONT['n_elev']):
                self.CONT['Actuators']['Elevon']['Actuators'][i].set_zero(ElevCmd_deg[i])
            
        else:
            for i in range(self.CONT['n_elev']):
                self.CONT['Elevon_deg'][i],v,a = self.CONT['Actuators']['Elevon']['Actuators'][i].step(ElevCmd_deg[i])
                
    def SENS_fcn(self):
        if self.trimming:
            self.SENS['Sensors']['IMU']['P_radps'].set_zero(self.EQM['VelRot_BodyAx_radps'][0])
            self.SENS['Sensors']['IMU']['Q_radps'].set_zero(self.EQM['VelRot_BodyAx_radps'][1])
            self.SENS['Sensors']['IMU']['R_radps'].set_zero(self.EQM['VelRot_BodyAx_radps'][2])
            
            self.SENS['Sensors']['IMU']['Phi_rad'].set_zero(self.EQM['EulerAngles_rad'][0])
            self.SENS['Sensors']['IMU']['Theta_rad'].set_zero(self.EQM['EulerAngles_rad'][1])
            self.SENS['Sensors']['IMU']['Psi_rad'].set_zero(self.EQM['EulerAngles_rad'][2])
            
            self.SENS['Sensors']['IMU']['VX_mps'].set_zero(self.EQM['VelLin_EarthAx_mps'][0])
            self.SENS['Sensors']['IMU']['VY_mps'].set_zero(self.EQM['VelLin_EarthAx_mps'][1])
            self.SENS['Sensors']['IMU']['VZ_mps'].set_zero(self.EQM['VelLin_EarthAx_mps'][2])
            
            self.SENS['Sensors']['IMU']['X_m'].set_zero(self.EQM['PosLin_EarthAx_m'][0])
            self.SENS['Sensors']['IMU']['Y_m'].set_zero(self.EQM['PosLin_EarthAx_m'][1])
            self.SENS['Sensors']['IMU']['Z_m'].set_zero(self.EQM['PosLin_EarthAx_m'][2])
            
            self.SENS['Sensors']['IMU']['NX_mps2'].set_zero(self.EQM['LoadFactor_mps2'][0])
            self.SENS['Sensors']['IMU']['NY_mps2'].set_zero(self.EQM['LoadFactor_mps2'][1])
            self.SENS['Sensors']['IMU']['NZ_mps2'].set_zero(self.EQM['LoadFactor_mps2'][2])

            self.SENS['Sensors']['ADS']['CAS_mps'].set_zero(self.ATM['CAS_mps'])
            
        else:
            self.SENS['Sensors']['IMU']['P_radps'].step(self.EQM['VelRot_BodyAx_radps'][0])
            self.SENS['Sensors']['IMU']['Q_radps'].step(self.EQM['VelRot_BodyAx_radps'][1])
            self.SENS['Sensors']['IMU']['R_radps'].step(self.EQM['VelRot_BodyAx_radps'][2])
                    
            self.SENS['Sensors']['IMU']['Phi_rad'].step(self.EQM['EulerAngles_rad'][0])
            self.SENS['Sensors']['IMU']['Theta_rad'].step(self.EQM['EulerAngles_rad'][1])
            self.SENS['Sensors']['IMU']['Psi_rad'].step(self.EQM['EulerAngles_rad'][2])
                    
            self.SENS['Sensors']['IMU']['VX_mps'].step(self.EQM['VelLin_EarthAx_mps'][0])
            self.SENS['Sensors']['IMU']['VY_mps'].step(self.EQM['VelLin_EarthAx_mps'][1])
            self.SENS['Sensors']['IMU']['VZ_mps'].step(self.EQM['VelLin_EarthAx_mps'][2])
            
            self.SENS['Sensors']['IMU']['X_m'].step(self.EQM['PosLin_EarthAx_m'][0])
            self.SENS['Sensors']['IMU']['Y_m'].step(self.EQM['PosLin_EarthAx_m'][1])
            self.SENS['Sensors']['IMU']['Z_m'].step(self.EQM['PosLin_EarthAx_m'][2])
            
            self.SENS['Sensors']['IMU']['NX_mps2'].step(self.EQM['LoadFactor_mps2'][0])
            self.SENS['Sensors']['IMU']['NY_mps2'].step(self.EQM['LoadFactor_mps2'][1])
            self.SENS['Sensors']['IMU']['NZ_mps2'].step(self.EQM['LoadFactor_mps2'][2])
            
            self.SENS['Sensors']['ADS']['CAS_mps'].step(self.ATM['CAS_mps'])
                            
        self.SENS['P_radps']   = self.SENS['Sensors']['IMU']['P_radps'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_P']) + (self.UNC['Res']['SENS']['Bias']['IMU_P'])
        self.SENS['Q_radps']   = self.SENS['Sensors']['IMU']['Q_radps'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_Q']) + (self.UNC['Res']['SENS']['Bias']['IMU_Q'])
        self.SENS['R_radps']   = self.SENS['Sensors']['IMU']['R_radps'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_R']) + (self.UNC['Res']['SENS']['Bias']['IMU_R'])
        self.SENS['Phi_rad']   = self.SENS['Sensors']['IMU']['Phi_rad'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_Phi']) + (self.UNC['Res']['SENS']['Bias']['IMU_Phi'])
        self.SENS['Theta_rad'] = self.SENS['Sensors']['IMU']['Theta_rad'].y  * (1+self.UNC['Res']['SENS']['Gain']['IMU_Theta']) + (self.UNC['Res']['SENS']['Bias']['IMU_Theta'])
        self.SENS['Psi_rad']   = self.SENS['Sensors']['IMU']['Psi_rad'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_Psi']) + (self.UNC['Res']['SENS']['Bias']['IMU_Psi'])
        self.SENS['VX_mps']    = self.SENS['Sensors']['IMU']['VX_mps'].y 
        self.SENS['VY_mps']    = self.SENS['Sensors']['IMU']['VY_mps'].y 
        self.SENS['VZ_mps']    = self.SENS['Sensors']['IMU']['VZ_mps'].y 
        self.SENS['X_m']       = self.SENS['Sensors']['IMU']['X_m'].y 
        self.SENS['Y_m']       = self.SENS['Sensors']['IMU']['Y_m'].y 
        self.SENS['Z_m']       = self.SENS['Sensors']['IMU']['Z_m'].y 
        self.SENS['NX_mps2']   = self.SENS['Sensors']['IMU']['NX_mps2'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_NX']) + (self.UNC['Res']['SENS']['Bias']['IMU_NX'])
        self.SENS['NY_mps2']   = self.SENS['Sensors']['IMU']['NY_mps2'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_NY']) + (self.UNC['Res']['SENS']['Bias']['IMU_NY'])
        self.SENS['NZ_mps2']   = self.SENS['Sensors']['IMU']['NZ_mps2'].y * (1+self.UNC['Res']['SENS']['Gain']['IMU_NZ']) + (self.UNC['Res']['SENS']['Bias']['IMU_NZ'])
        self.SENS['CAS_mps']   = self.SENS['Sensors']['ADS']['CAS_mps'].y * (1+self.UNC['Res']['SENS']['Gain']['ADS_CAS'])


                
                

    
    
