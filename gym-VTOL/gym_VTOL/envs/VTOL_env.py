#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 21:02:13 2021

@author: dsalarc
"""

import gym
from gym import spaces
import numpy as np
import matplotlib.pyplot as plt

class Vahana_VertFlight(gym.Env):
    """Custom Environment that follows gym interface"""
    #metadata = {'render.modes': ['human']}
    metadata = {'render.modes': ['console']}

    def __init__(self, K=1, M=1, C=0, g = 0, MaxForce = 3):
        # super(Vahana_VertFlight, self).__init__() 
        
        #Define Constants
        self.K = K
        self.M = M
        self.C = C
        self.g = g
        self.I = np.array([[1,0,0],[0,1,0],[0,0,1]])
        
        self.MaxForce = MaxForce        
        self.n_states = 12
        self.t_step = 0.05
        
        # Define action and observation space
        '''
        sta: vetor de estados na ordem:
        VL_b: vetor velocidades lineares no eixo do corpo (u,v,w)
        VR_b: vetor velocidade rotacional no eixo do corpo (p, q, r)
        XL_e: vetor posição lineares no eixo da terra (X, Y, Z)
        XR_e: vetor posição rotacional no eixo da terra (phi, theta psi)  
        '''
        self.MaxState = np.array([20,20,20,np.pi,np.pi,np.pi,20,20,20,np.pi,np.pi,np.pi])
        
        self.action_space = spaces.Box(low=-self.MaxForce, 
                                       high=self.MaxForce,
                                       shape=(1,),
                                       dtype=np.float16)
        self.observation_space = spaces.Box(low=-self.MaxState,
                                            high=self.MaxState,
                                            dtype=np.float16)  
    
    def Euler_2nd(self,X,Xdot,Xdotdot,t_step):
        
        X = X + Xdot*t_step + (Xdotdot*t_step**2)/2
        return X

    def step(self, action):
      self.CurrentStep += 1
      
      self.StdAtm_fcn()
      # Execute one time step within the environment
      
      ExtForce    = np.array([0],dtype = np.float16)
      ExtForce[0] = min(max(action[0], -self.MaxForce), self.MaxForce)
      SysForce    = np.array([0],dtype = np.float16)
      SysForce[0] = (-self.EQM['sta'][2]*self.K - self.EQM['sta'][8]*self.C)
      
      TotalForce  = np.array([0,0,ExtForce[0]+SysForce[0]])
      TotalMoment = np.array([0,0,0])   
      
      Last_Xdot    = self.EQM['sta_dot'].copy()
      self.EQM_fcn(TotalForce,TotalMoment,self.I,self.M)
      self.EQM['sta_dotdot'] = (self.EQM['sta_dot'] - Last_Xdot)/self.t_step
      
      self.EQM['sta'] = self.Euler_2nd(self.EQM['sta'],self.EQM['sta_dot'],self.EQM['sta_dotdot'],self.t_step)
      self.AllStates = np.vstack((self.AllStates,self.EQM['sta']))

      
      # Calculate Reward
      self.LastReward = self.CalcReward()
      
      # Terminal State
      if self.LastReward > 0.99:
          done = True
      else:
          done = False
      
      done = False
      # Optionally we can pass additional info, we are not using that for now
      info = {}          
      info['ATM'] = self.ATM
      info['EQM'] = self.EQM
      
      obs = np.take(self.EQM['sta'],np.array([0,5]))
      return obs, self.LastReward, done, info
      
    def CalcReward(self):
        # Reward = 1 - np.sqrt(np.sum((self.EQM['sta'] / self.MaxState)**2))
        Reward = 10 - (self.EQM['sta'][2]**2 + self.EQM['sta'][8]**2)/10
        return Reward    
    
    def reset(self,XV=0):

      self.StartUp()
      # Reset the state of the environment to an initial state
      if type(XV) == np.ndarray:
          X = XV[0]
          V = XV[1]
      else:
          Energy = 100
          Theta = np.random.random(1) * 2 * np.pi
          X = Energy**0.5 * np.cos(Theta[0])
          V = Energy**0.5 * np.sin(Theta[0])

      self.EQM['sta']        = np.zeros(shape=12,dtype = np.float32)
      self.EQM['sta'][2]     = X
      self.EQM['sta'][8]     = V
      self.EQM['sta_dot']    = np.zeros(shape=np.shape(self.EQM['sta']),dtype = np.float32)
      self.EQM['sta_dotdot'] = np.zeros(shape=np.shape(self.EQM['sta']),dtype = np.float32)
      
      self.CurrentStep = 0
      
      self.AllStates = self.EQM['sta']
      
      return self.EQM['sta']
     
      
    def render(self, mode='console', close=False):
        
        # Render the environment to the screen       
        plt.figure(1)
        plt.clf()
        plt.grid('on')
        plt.xlim((self.observation_space.low[0],self.observation_space.high[0]))
        plt.ylim((self.observation_space.low[1],self.observation_space.high[1]))
        if self.CurrentStep > 0:
            plt.plot(self.AllStates[:,2],self.AllStates[:,8],'tab:gray')
            plt.plot(self.AllStates[-1,2],self.AllStates[-1,8],'ob')
        else:
            plt.plot(self.AllStates[2],self.AllStates[8],'ob')
        plt.xlabel('Position Z [m]')
        plt.ylabel('Velocity W [m/s]')
        plt.show()
        
    def close(self):
        pass
    
    # %% SARTUP FUNCTION
    def StartUp (self):
 
      # INITIALIZE DICTS
      self.EQM  = {}
      self.ATM  = {}
      self.MASS = {}
      self.MOT  = {}
      self.AERO = {}
      
      # DEFINE CONSTANTS
      
      # ATM
      self.ATM['dISA_C'] = 0
      self.ATM['Const'] = {}
      
      self.ATM['Const']['C2K'] = 273.15
      self.ATM['Const']['R']   = 287.04  
      self.ATM['Const']['P0_Pa']       = 101325
      self.ATM['Const']['T0_C']        = 15
      self.ATM['Const']['T0_K']        = self.ATM['Const']['T0_C'] + self.ATM['Const']['C2K']
      self.ATM['Const']['rho0_kgm3']   = 1.225
      self.ATM['Const']['Vsound0_mps'] = np.sqrt(1.4*self.ATM['Const']['R']*(self.ATM['Const']['T0_K']))
        
      # EQM  
      self.EQM['g_mps2'] = -9.806
      
      # MASS
      self.MASS['Weight_kgf'] = self.M
      self.MASS['I_kgm'] = self.I
    
    # %% EQM FUNCTION
    def EQM_fcn(self,F_b,M_b,I,m):
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
                VL_b: vetor velocidades lineares no eixo do corpo (u,v,w)
                VR_b: vetor velocidade rotacional no eixo do corpo (p, q, r)
                XL_e: vetor posição lineares no eixo da terra (X, Y, Z)
                XR_e: vetor posição rotacional no eixo da terra (phi, theta psi)
                
        Outputs:
            sta_dot: vetor de derivadas de estados na ordem:
                VLd_b: aceleração linear no eixo do corpo (u_dot,v_dot,w_dot)
                VRd_b: aceleração rotacional no eixo do corpo (p_dot, q_dot, r_dot)
                XLd_e: vetor velocidades lineares no eixo da terra (X_dot, Y_dot, Z_dot)
                XRd_e: vetor velocidade rotacional no eixo da terra   (phi_dot, theta_dot psi_dot)      
            
        Descrição
            Função geral das equações de movimento do corpo rígido qualquer.
            Com massas, inercias, forças e momentos, calcula as derivadas doos estados
            É uma função genérica para qualquer corpo rídido no espaço
        """
        g0 = -9.806
        # LEITURA DOS VETORES NOS ESTADOS
        VL_b = self.EQM['sta'][6:9]
        VR_b = self.EQM['sta'][9:12]
        XL_e = self.EQM['sta'][0:3]
        XR_e = self.EQM['sta'][3:6]
        
        # MATRIZ DE ROTACAO
    
        cos_phi = np.cos(XR_e[0]); sin_phi = np.sin(XR_e[0]); 
        cos_the = np.cos(XR_e[1]); sin_the = np.sin(XR_e[1]); tan_the = np.tan(XR_e[1])
        cos_psi = np.cos(XR_e[2]); sin_psi = np.sin(XR_e[2]); 
        
        # Matriz de Rotação do Eixo Terra para Eixo Corpo    
        Lbt = np.array([[cos_the*cos_psi                           , cos_the*sin_psi                           , -sin_the],
                        [sin_phi*sin_the*cos_psi - cos_phi*sin_psi , sin_phi*sin_the*sin_psi + cos_phi*cos_psi , sin_phi*cos_the],
                        [cos_phi*sin_the*cos_psi + sin_phi*sin_psi , cos_phi*sin_the*sin_psi - sin_phi*cos_psi , cos_phi*cos_the]])
        
        # Matriz de Rotação do Eixo Corpo para Eixo Terra
        Ltb = np.transpose(Lbt)
        
        # Vetor Peso no Eixo Corpo
        W_b = np.dot(Lbt,np.array([0,0,m*g0]))
        
        # CALCULO DAS DERIVADAS
        VLd_b = (F_b+W_b)/m - np.cross(VR_b,VL_b)
        
        VRd_b = np.dot(np.linalg.inv(I),(M_b - np.cross(VR_b,np.dot(I,VR_b))))
    
        
        XLd_e = np.dot(Ltb,VL_b)
        
        XRd_e = np.dot(np.array([
                    [1 , sin_phi*tan_the , cos_phi*tan_the],
                    [0 , cos_phi         , -sin_phi],
                    [0 , sin_phi/cos_the , cos_phi/cos_the]]),VR_b)    
        
        # VETOR SAIDA COM OS ESTADOS
        self.EQM['sta_dot'][6:9]  = VLd_b
        self.EQM['sta_dot'][9:12] = VRd_b
        self.EQM['sta_dot'][0:3]  = XLd_e
        self.EQM['sta_dot'][3:6]  = XRd_e

    # %% ATOSPHERE FUNCTION
    def StdAtm_fcn(self):
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
        BodySpeeds_mps = self.EQM['sta_dot'][6:9].copy()
        Altitude_m     = -self.EQM['sta_dot'][2]
        # INITIAL CALCULATIIONS
        self.ATM['TAS_mps'] = np.linalg.norm(BodySpeeds_mps)
        
        if Altitude_m < 11000:
            self.ATM['TStd_C'] = self.ATM['Const']['T0_C'] - 0.0065*Altitude_m
            self.ATM['T_C']    = self.ATM['TStd_C'] + self.ATM['dISA_C']
            self.ATM['P_Pa']   = self.ATM['Const']['P0_Pa']*(1-0.0065*Altitude_m/self.ATM['Const']['T0_K'])**5.2561
        else:
            self.ATM['TStd_C'] = -56.5
            self.ATM['T_C']    = self.ATM['TStd_C'] + self.ATM['dISA_C']
            self.ATM['P_Pa']   = 22632 * np.exp(-g0/(self.ATM['Const']['R']*216.65)*(Altitude_m-11000))
        
        self.ATM['rho_kgm3']   = self.ATM['P_Pa'] / (self.ATM['Const']['R']*(self.ATM['T_C'] + self.ATM['Const']['C2K']))
        self.ATM['Vsound_mps'] = np.sqrt(1.4*self.ATM['Const']['R']*(self.ATM['T_C']+self.ATM['Const']['C2K']))
        self.ATM['TAS2EAS']    = np.sqrt(self.ATM['rho_kgm3']/self.ATM['Const']['rho0_kgm3'])
        
        self.ATM['EAS_mps'] = self.ATM['TAS_mps'] * self.ATM['TAS2EAS']
        self.ATM['Mach']    = self.ATM['TAS_mps'] / self.ATM['Vsound_mps']
        
        self.ATM['PDyn']    = self.ATM['EAS_mps'] **2 * self.ATM['rho_kgm3'] / 2
        
        self.ATM['qc'] = self.ATM['P_Pa'] *((1+0.2*self.ATM['Mach']**2)**(7/2)-1)
        self.ATM['CAS_mps']  = self.ATM['Const']['Vsound0_mps']*np.sqrt(5*((self.ATM['qc'] /self.ATM['Const']['P0_Pa']+1)**(2/7)-1))
        
