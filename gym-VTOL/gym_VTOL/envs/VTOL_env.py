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
    metadata = {'render.modes': ['console']}

    def __init__(self):
        # super(Vahana_VertFlight, self).__init__() 
        
        #Define Constants
        
        self.n_states = 12
        self.t_step = 0.05
        
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
        # Observation Space    = [Vx , dVx, Vy , dVy , Vz , dVz , Phi      , p        , Theta    , q        , Psi      , r        ]
        self.adm_vec  = np.array([20 , 20 , 10 , 10  , 10 , 10  , 3.1415/2 , 3.1415/2 , 3.1415/2 , 3.1415/2 , 3.1415/2 , 3.1415/2 ])
        self.MaxState = np.array([1  , 1  , 1  , 1   ,  1 , 1   , 1        , 1        , 1        , 1        , 1        ,1         ])
        # self.MaxState = np.array([np.inf,np.inf])
        
        '''
        ACTIONS:
            4 hover actions
            2 surface deflection (in % of Max Deflection)
            Front Wing Elevon
            Back Wing Elevon
            Aileron
        '''
        self.action_names = ['Throttle','PitchThrottle',
                             'RollThrottle','YawThrottle',
                             'W1_Tilt','W2_Tilt',
                             'W1_Elevator','W2_Elevator',
                             'W1_Aileron','W2_Aileron']
        
        self.action_space = spaces.Box(low=-1, 
                                       high=1,
                                       shape=(len(self.action_names),),
                                       dtype=np.float16)
        self.observation_space = spaces.Box(low=-self.MaxState,
                                            high=self.MaxState,
                                            dtype=np.float16)  

    
    def OutputObs(self,sta,sta_dot,sta_int,cont):
        # obs = np.hstack((sta,sta_dot,cont))
        # Observation Space    = [Vx , dVx, Vy , dVy , Vz , dVz , Phi      , p        , Theta    , q        , Psi      , r        ]
        obs_vec       = np.array([self.EQM['VelLin_EarthAx_mps'][0] , self.EQM['AccLin_EarthAx_mps2'][0] , 
                                  self.EQM['VelLin_EarthAx_mps'][1] , self.EQM['AccLin_EarthAx_mps2'][1] , 
                                  self.EQM['VelLin_EarthAx_mps'][2] , self.EQM['AccLin_EarthAx_mps2'][2] , 
                                  self.EQM['EulerAngles_rad'][0]    , self.EQM['VelRot_BodyAx_radps'][0] , 
                                  self.EQM['EulerAngles_rad'][1]    , self.EQM['VelRot_BodyAx_radps'][1] , 
                                  self.EQM['EulerAngles_rad'][2]    , self.EQM['VelRot_BodyAx_radps'][2]])
        obs_adm = obs_vec / self.adm_vec
        
        
        
        
        obs_sat = np.min( np.vstack((obs_adm,np.ones(len(obs_vec)) )) , axis=0)
        obs     = np.max( np.vstack((obs_sat,-np.ones(len(obs_vec)))) , axis=0)

        # obs = np.array([sta[8],sta_dot[8]])
        return obs
    
    def reset(self,W = 'rand', Z = 0, THETA = 'rand',  PHI = 'rand',  PSI = 'rand', PaxIn = np.array([1,1]),
                   VX_mps = 0, VZ_mps = 0, W1_Tilt_p = 1, W2_Tilt_p = 1):
      self.CurrentStep = 0
      self.trimming = 0

      # Initialize Contants  

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
      
      print('Goto trim')
      TrimAction = self.trim(TrimVX_mps = VX_mps, TrimVZ_mps = VZ_mps, TrimW1_Tilt_p = W1_Tilt_p, TrimW2_Tilt_p = W2_Tilt_p)
      
      self.EQM_fcn(np.array([0,0,0]), np.array([0,0,0]), self.MASS['I_kgm'], self.MASS['Weight_kgf'])
      
      # # Set Initial Control
      # self.CONT['RPM_vec'] = np.ones(self.MOT['n_motor']) * 0.8
      # self.CONT['RPM_p']  = self.CONT['RPM_vec'] ** (1/2) 
      # self.CONT['Tilt_p'] = np.ones(self.CONT['n_TiltSurf']) * 1
      
      # # Set Initial RPM (necessary due to first order filter)
      # self.MOT['RPM'] = np.multiply(self.CONT['RPM_p'],self.MOT['RPMRange']) + self.MOT['MinRPM']
      
      self.AllStates = self.EQM['sta']

      obs = self.OutputObs(self.EQM['sta'],self.EQM['sta_dot'],self.EQM['sta_int'],self.CONT['RPM_p'])
      
      self.obs_ini = [max(np.deg2rad(10) , abs(self.EQM['sta'][4])) , 
                      max(np.deg2rad(10) , abs(self.EQM['sta'][3])) , 
                      max(np.deg2rad(10) , abs(self.EQM['sta'][5])) , 
                      max(           10  , abs(self.EQM['sta'][8])) ,
                      np.deg2rad(10) ,
                      np.deg2rad(10) ,
                      np.deg2rad(10) ,
                      10             ]
     
      return obs, TrimAction
  
    def trim(self, TrimVX_mps = 0, TrimVZ_mps = 0, TrimW1_Tilt_p = 1, TrimW2_Tilt_p = 1):
        
        # Define function to get Outputs of interest
        def GetOutputFloat(EQM):
            return np.array([EQM['VelLin_EarthAx_mps'][0] ,
                             EQM['VelLin_EarthAx_mps'][2]])
        
        #Trimming parameters
        TrimTol  = 1e-6
        TrimIter = 20
        TrimPert = 1e-5
        
        self.trimming = 1
        
        # Call EQM
        self.EQM_fcn(np.array([0,0,0]), np.array([0,0,0]), self.MASS['I_kgm'], self.MASS['Weight_kgf'])
        

        
        #Define Initial Trim Action
        TrimAction = np.zeros(np.shape(self.action_space))
        TrimAction[self.action_names.index('W1_Tilt')] = TrimW1_Tilt_p
        TrimAction[self.action_names.index('W2_Tilt')] = TrimW2_Tilt_p
        
        TrimState = np.zeros(np.shape(self.EQM['sta_names']))
        
        # Define Freeze and Floats Indexes
        n_ActionFloat    = np.array([self.action_names.index('Throttle') ,
                                     self.action_names.index('PitchThrottle')])
        
        n_StateFloat     = np.array([self.EQM['sta_names'].index('Theta_rad'),
                                     self.EQM['sta_names'].index('U_mps'), 
                                     self.EQM['sta_names'].index('W_mps')])
        
        n_StateDotFreeze = np.array([self.EQM['sta_names'].index('U_mps') , 
                                     self.EQM['sta_names'].index('W_mps') , 
                                     self.EQM['sta_names'].index('Q_radps')])
        
        TrimVars = np.hstack((TrimAction[n_ActionFloat],TrimState[n_StateFloat]))

        # Trim Target Vector
        TrimTarget = np.hstack((np.zeros(len(n_StateDotFreeze)) , np.array([TrimVX_mps, TrimVZ_mps])))
       
        # Perform One Step    
        self.EQM['sta'] = TrimState
        self.step(TrimAction)
        
        TrimStaDot = self.EQM['sta_dot'][n_StateDotFreeze]
        TrimOutput = GetOutputFloat(self.EQM)
        TrimError  = np.hstack((TrimStaDot , TrimOutput)) - TrimTarget
        TrimErrorNorm = np.linalg.norm(TrimError)
        
        iter_n = 1
        ContinueTrim = True
        
        while ContinueTrim:
            # Initialize Hessian
            H = np.zeros((len(n_ActionFloat) + len(n_StateFloat) , len(TrimTarget)))
            
            # Calulate Hessian
            for i in range(len(n_ActionFloat) + len(n_StateFloat)):
                
                TrimAction_Pert_p = TrimAction.copy()
                TrimAction_Pert_m = TrimAction.copy()
                TrimState_Pert_p  = TrimState.copy()
                TrimState_Pert_m  = TrimState.copy()
               
                if i < len(n_ActionFloat):  
                    
                    TrimAction_Pert_p[n_ActionFloat[i]] = TrimAction[n_ActionFloat[i]] + TrimPert
                    TrimAction_Pert_m[n_ActionFloat[i]] = TrimAction[n_ActionFloat[i]] - TrimPert
                    
                else:
                    j = i - len(n_ActionFloat) 
                    TrimState_Pert_p[n_StateFloat[j]]  = TrimState[n_StateFloat[j]] + TrimPert
                    TrimState_Pert_m[n_StateFloat[j]]  = TrimState[n_StateFloat[j]] - TrimPert
              
                self.EQM['sta'] = TrimState_Pert_p
                self.step(TrimAction_Pert_p)
                StateDot_p = self.EQM['sta_dot'][n_StateDotFreeze]
                Output_p  = GetOutputFloat(self.EQM)
                
                self.EQM['sta'] = TrimState_Pert_m
                self.step(TrimAction_Pert_m)
                StateDot_m = self.EQM['sta_dot'][n_StateDotFreeze]
                Output_m  = GetOutputFloat(self.EQM)
               
                H[:,i] = (np.hstack((StateDot_p,Output_p)) - np.hstack((StateDot_m,Output_m))) / (2*TrimPert)
                
                
            TrimVars = TrimVars - np.matmul(np.linalg.pinv(H) , TrimError)
            TrimAction[n_ActionFloat] = TrimVars[0:len(n_ActionFloat)]
            TrimState[n_StateFloat] = TrimVars[len(n_ActionFloat):]
            
            
            # Perform One Step    
            self.EQM['sta'] = TrimState
            self.step(TrimAction)
            
            TrimStaDot = self.EQM['sta_dot'][n_StateDotFreeze]
            TrimOutput = GetOutputFloat(self.EQM)           
            TrimError     = np.hstack((TrimStaDot , TrimOutput)) - TrimTarget
            TrimErrorNorm = np.linalg.norm(TrimError)
        
            iter_n = iter_n + 1
            
            if iter_n >=TrimIter:
                ContinueTrim = False
                print('Trim Error - Max number of iterations')
            elif TrimErrorNorm <= TrimTol:
                ContinueTrim = False
                print('Trim successful - error below tolerance')
                   
                    
        self.trimming = 0
        
        return TrimAction
       
        
    def step(self, action):
      if not(self.trimming):  
          self.CurrentStep += 1
      
      # Calculate Control, Atmosphere, Motor Forces and Aero Forces
      
      self.CONT_fcn(action)
      self.StdAtm_fcn()
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
      
      # Calculate Reward
      if not(self.trimming):
          self.LastReward = self.CalcReward()
      
      # Terminal State = False   
      done = False
      # if abs(np.rad2deg(self.EQM['sta'][4])) > 45:
      # if self.sta[4] > 90:
          # self.LastReward = (300-self.CurrentStep) * self.LastReward
          # self.LastReward = 0
          # self.LastReward = -10000
          # done = True
          
          
      # if abs(np.rad2deg(self.EQM['sta'][4])) < 0.1 and abs(np.rad2deg(self.EQM['sta_dot'][4])) < 0.1 and (
      #    abs(np.rad2deg(self.EQM['sta'][0])) < 0.1 and abs(np.rad2deg(self.EQM['sta_dot'][0])) < 0.1):
      #     self.LastReward = 0
      #     done = True   
          
      # Export Model Oututs throught info
      info = {}          
      info['ATM']  = self.ATM
      info['EQM']  = self.EQM
      info['MOT']  = self.MOT
      info['AERO'] = self.AERO
      info['CONT'] = self.CONT
      info['MASS'] = self.MASS
      
      obs = self.OutputObs(self.EQM['sta'],self.EQM['sta_dot'],self.EQM['sta_int'],self.CONT['RPM_p'])

      if not(self.trimming):
          return obs, self.LastReward, done, info
      else:
          return info
          
      
    def CalcReward(self):
        
        # SECOND REWARD - Sigmoide_Gain3 / Penalty Speed / Euler Dot / Squared Sum
        Reward = ( 10*(1 - np.sqrt(min(1,abs(self.EQM['sta'][4]) / self.obs_ini[0])**2 +
                                    min(1,abs(self.EQM['sta'][10]) / self.obs_ini[4])**2) / np.sqrt(2)) +
                   10*(1 - np.sqrt(min(1,abs(self.EQM['sta'][3]) / self.obs_ini[1])**2 +
                                    min(1,abs(self.EQM['sta'][9]) / self.obs_ini[5])**2) / np.sqrt(2)) +
                   10*(1 - np.sqrt(min(1,abs(self.EQM['sta'][5]) / self.obs_ini[2])**2 +
                                    min(1,abs(self.EQM['sta'][11]) / self.obs_ini[6])**2) / np.sqrt(2)) +
                   10*(1 - np.sqrt(min(1,abs(self.EQM['sta'][8]) / self.obs_ini[3])**2 +
                                    min(1,abs(self.EQM['sta_dot'][8]) / self.obs_ini[7])**2) / np.sqrt(2)) )
  
        return Reward    
    

     
      
    def render(self, mode='console', close=False):
        
        # Render the environment to the screen       
        plt.figure(1)
        plt.clf()
        plt.grid('on')
        # plt.xlim((self.observation_space.low[0],self.observation_space.high[0]))
        # plt.ylim((self.observation_space.low[1],self.observation_space.high[1]))
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
        
        X = X + Xdot*t_step + (Xdotdot*t_step**2)/2
        return X
    
    def Euler_1st(self,X,Xdot,t_step):
        
        X = X + Xdot*t_step
        return X

    # %% SARTUP FUNCTION
    def StartUp (self,PaxIn):
      # OPTIONS
      self.OPT  = {}
      self.OPT['UseAeroMoment'] = 1
      self.OPT['UseAeroForce']  = 1
      self.OPT['EnableRoll']    = 1
      self.OPT['EnablePitch']   = 1
      self.OPT['EnableYaw']     = 1

      # INITIALIZE DICTS
      self.EQM  = {}
      self.GEOM = {}
      self.ATM  = {}
      self.MASS = {}
      self.MOT  = {}
      self.AERO = {}
      self.CONS = {}
      self.CONT = {}
      
      # DEFINE CONSTANTS
      self.CONS['kt2mps'] = 0.514444
      self.CONS['mps2kt'] = 1 / self.CONS['kt2mps']     
      
      # ATM
      self.ATM['dISA_C'] = 0
      
      self.ATM['WindX_kt'] = 0
      self.ATM['WindY_kt'] = 0
      self.ATM['WindZ_kt'] = 0
      
      self.ATM['Const'] = {}
      
      self.ATM['Const']['C2K']         = 273.15
      self.ATM['Const']['R']           = 287.04  
      self.ATM['Const']['P0_Pa']       = 101325
      self.ATM['Const']['T0_C']        = 15
      self.ATM['Const']['T0_K']        = self.ATM['Const']['T0_C'] + self.ATM['Const']['C2K']
      self.ATM['Const']['rho0_kgm3']   = 1.225
      self.ATM['Const']['Vsound0_mps'] = np.sqrt(1.4*self.ATM['Const']['R']*(self.ATM['Const']['T0_K']))
        
      # EQM  
      self.EQM['g_mps2'] = 9.806
      
      # GEOM
      self.GEOM['Wing']          = {}
      self.GEOM['Wing']['cma_m'] = np.array([0.65,0.65])
      self.GEOM['Wing']['b_m']   = np.array([6,6])
      self.GEOM['Wing']['S_m2']  = np.array([3.9,3.9])
      self.GEOM['Wing']['X_m']   = np.array([0.3,3.4])
      self.GEOM['Wing']['Y_m']   = np.array([0,0])
      self.GEOM['Wing']['Z_m']   = np.array([0,1.5])
      
      self.GEOM['Fus']          = {}
      self.GEOM['Fus']['cma_m'] = np.array([4.1])
      self.GEOM['Fus']['b_m']   = np.array([.9])
      self.GEOM['Fus']['S_m2']  = np.array([np.pi * 0.45**2])
      self.GEOM['Fus']['X_m']   = np.array([2.05])
      self.GEOM['Fus']['Y_m']   = np.array([0])
      self.GEOM['Fus']['Z_m']   = np.array([0])
 
      # MASS
      self.MASS['Pax']             = PaxIn
      self.MASS['PaxWeight_kgf']   = 100
      self.MASS['EmptyWeight_kgf'] = 616
      self.MASS['Weight_kgf'] = self.MASS['EmptyWeight_kgf'] + np.sum(self.MASS['Pax']) * self.MASS['PaxWeight_kgf']
      self.MASS['Empty_CG_m'] = np.array([1.85 , 0.0 , 0.53])
      self.MASS['PaxPos_m'] = np.array([[0.9 , 0.0 , 1.0],
                                        [2.5 , 0.0 , 1.0]])
      self.MASS['CG_m'] = np.array(self.MASS['Empty_CG_m']    * self.MASS['EmptyWeight_kgf'] + 
                                   self.MASS['PaxPos_m'][0,:] * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                                   self.MASS['PaxPos_m'][1,:] * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf']) / self.MASS['Weight_kgf']
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
                                   
      
      # AERO
      self.AERO['Wing'] = {}
      self.AERO['Fus']  = {}

      # MOTOR
      x1 = 0.04
      x2 = 2*self.MASS['CG_m'][0] - x1
      y1 = 1.3
      y2 = 3.0
      z1  = 0
      z2 = 1.5
      
      self.MOT['n_motor'] = 8
      self.MOT['Position_m'] = np.array([[x1,-y2,z1],
                                         [x1,-y1,z1],
                                         [x1,+y1,z1],
                                         [x1,+y2,z1],
                                         [x2,-y2,z2],
                                         [x2,-y1,z2],
                                         [x2,+y1,z2],
                                         [x2,+y2,z2]])

      self.MOT['MaxRPM']        = np.ones(self.MOT['n_motor']) * 2600
      self.MOT['MinRPM']        = np.ones(self.MOT['n_motor']) * 0.01
      self.MOT['RPMRange']      = self.MOT['MaxRPM'] - self.MOT['MinRPM'] 
      self.MOT['Diameter_m']    = np.ones(self.MOT['n_motor']) * 1.5
      self.MOT['RotationSense'] = np.array([+1,-1,+1,-1,
                                            -1,+1,-1,+1])  
      
      # CT e CP - Vide planilha
      self.MOT['CT_J']       = np.array([[0.0 , 0.01 , 0.80] , 
                                         [0.14, 0.14 , 0.00]])
      self.MOT['CP_J']       = np.array([[0.0 , 0.01 , 0.80] , 
                                         [0.06, 0.06 , 0.01]])

      self.MOT['TiltSurf_link']  = np.array([0,0,0,0,1,1,1,1])                 #ID of surface which the motor is linked. Every motor will rotate the same amount
      
      self.MOT['Bandwidth_radps'] = 40
      self.MOT['Beta'] = np.exp(-self.MOT['Bandwidth_radps']*self.t_step)

      # CONTROL
      self.CONT['n_TiltSurf']    = 2
      self.CONT['MinTilt_deg']   = np.ones(self.CONT['n_TiltSurf']) * 0
      self.CONT['MaxTilt_deg']   = np.ones(self.CONT['n_TiltSurf']) * 90
      self.CONT['TiltRange_deg'] = self.CONT['MaxTilt_deg'] - self.CONT['MinTilt_deg'] 
           
      # OTHER CONSTANTS
      
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
        
        # LE2R = np.linalg.inv (LR2E)
        
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
        
        # Vetor Peso no Eixo Corpo
        W_b = np.dot(self.EQM['LE2B'],np.array([0,0,m*self.EQM['g_mps2']]))
        
        # CALCULO DAS DERIVADAS
        dVL_b = (F_b+W_b)/m - np.cross(VR_b,VL_b)
        
        dVR_b = np.dot(np.linalg.inv(I),(M_b - np.cross(VR_b,np.dot(I,VR_b))))
        
        dXL_e = np.dot(self.EQM['LB2E'] , VL_b)
        
        dXR_e = np.dot(self.EQM['LR2E'] , VR_b)    
        
        # OUTPUt
        self.EQM['VelLin_BodyAx_mps']   = VL_b
        self.EQM['VelRot_BodyAx_radps'] = VR_b
        self.EQM['PosLin_EarthAx_m']    = XL_e 
        self.EQM['EulerAngles_rad']     = XR_e 
        
        self.EQM['AccLin_BodyAx_mps2']   = dVL_b
        self.EQM['AccRot_BodyAx_radps2'] = dVR_b
        self.EQM['VelLin_EarthAx_mps']   = dXL_e   
        self.EQM['AccLin_EarthAx_mps2']  = np.dot(self.EQM['LB2E'] , dVL_b)
        self.EQM['EulerAnglesDot_radps'] = dXR_e 

        self.EQM['LoadFactor_g'] = (F_b)/m 

        # VETOR SAIDA COM OS ESTADOS
        self.EQM['sta_dot'][6:9]  = dVL_b
        self.EQM['sta_dot'][9:12] = dVR_b
        self.EQM['sta_dot'][0:3]  = dXL_e
        self.EQM['sta_dot'][3:6]  = np.array([self.OPT['EnableRoll'],self.OPT['EnablePitch'] ,self.OPT['EnableYaw'] ]) * dXR_e

    # %% ATMOSPHERE FUNCTION
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
            self.ATM['P_Pa']   = 22632 * np.exp(-self.EQM['g_mps2']/(self.ATM['Const']['R']*216.65)*(Altitude_m-11000))
        
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

        
        if abs(self.ATM['Vaero'][0]) < 1e-2:
            u = 0
        else:
            u = self.ATM['Vaero'][0]
            
        if abs(self.ATM['Vaero'][1]) < 1e-2:
            v = 0
        else:
            v = self.ATM['Vaero'][1]
            
        if abs(self.ATM['Vaero'][2]) < 1e-2:
            w = 0
        else:
            w = self.ATM['Vaero'][2]
            
        self.ATM['Alpha_deg'] = np.rad2deg(
                               np.arctan2(w,u))
       
        Beta_aux  = np.rad2deg(
                                np.arctan2(v*np.cos(np.deg2rad(self.ATM['Alpha_deg'] )),u))      
        
        self.ATM['Beta_deg'] = Beta_aux * np.sign(self.cosd(Beta_aux))         #sign correction to consider backward flight (AOA = 180)
       
    # %% MOTOR MODEL
    def MOT_fcn(self):            
        # Calcular Tracao/Torque de Cada Helice
        # Calcular Fp de cada hélice
        # Calcular Torque devido a inercia (conservacao momento angular)
        
        # Calculate RPM and Rotation of each Propeller
        RPM_tgt = np.multiply(self.CONT['RPM_p'],self.MOT['RPMRange']) + self.MOT['MinRPM']
        
        if self.CurrentStep == 0:   
            self.MOT['RPM'] = RPM_tgt
        else:
            old_RPM = self.MOT['RPM']
            self.MOT['RPM'] = (self.MOT['Beta']) * old_RPM + (1-self.MOT['Beta']) * RPM_tgt
        self.MOT['RPS'] = self.MOT['RPM'] / 60
        
        

        self.MOT['Tilt_p']   = self.CONT['Tilt_p'][self.MOT['TiltSurf_link']]
        
        self.MOT['Tilt_deg'] = (self.CONT['MinTilt_deg'][self.MOT['TiltSurf_link']] 
                                + self.CONT['TiltRange_deg'][self.MOT['TiltSurf_link']] * self.MOT['Tilt_p'])  
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
            
        # Calculate Advance Ratio (J) CT (Thrust Coef) and CP (Power Coef)
        # Source: Diss. Mestrado Daud Filho
        MOT_J = MOT_VTotal_p[:,0] / (self.MOT['RPS'] * self.MOT['Diameter_m'])
        MOT_CT = np.interp(MOT_J,self.MOT['CT_J'][0,:],self.MOT['CT_J'][1,:])
        MOT_CP = np.interp(MOT_J,self.MOT['CP_J'][0,:],self.MOT['CP_J'][1,:])
        MOT_CQ = MOT_CP / (2*np.pi)
        
        # Calculate Thrust and Torque
        self.MOT['Thrust_N']  = self.ATM['rho_kgm3'] * MOT_CT * self.MOT['RPS']**2 * self.MOT['Diameter_m']**4
        self.MOT['Torque_Nm'] = self.ATM['rho_kgm3'] * MOT_CQ * self.MOT['RPS']**2 * self.MOT['Diameter_m']**5
        
        self.MOT['Force_BodyAx_N'] = np.zeros([self.MOT['n_motor'],3])
        self.MOT['Moment_BodyAx_N'] = np.zeros([self.MOT['n_motor'],3])
        
        for i in range(self.MOT['n_motor']):
            self.MOT['Force_BodyAx_N'][i,:]  = np.dot(LM2B[:,:,i],np.array([self.MOT['Thrust_N'][i],0,0]))
            r    = self.MOT['Position_m'][i,:] - self.MASS['CG_m']
            r[0] = -r[0]
            r[2] = -r[2]
            self.MOT['Moment_BodyAx_N'][i,:] = np.cross(r,self.MOT['Force_BodyAx_N'][i,:]) + np.dot(LM2B[:,:,i],np.array([self.MOT['Torque_Nm'][i],0,0]))*self.MOT['RotationSense'][i]
        
        self.MOT['TotalForce_BodyAx_N'] = np.sum(self.MOT['Force_BodyAx_N'] , axis = 0)
        self.MOT['TotalMoment_BodyAx_Nm'] = np.sum(self.MOT['Moment_BodyAx_N'] , axis = 0)
        
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
            
            return SurfaceBETA
        
        
        self.AERO['Wing']['Incidence_deg'] = (self.CONT['MinTilt_deg'][0] 
                                            + self.CONT['TiltRange_deg'][0] * self.CONT['Tilt_p'])
        
        self.AERO['Wing']['EPS_deg'] = np.array([0,0])

        self.AERO['Wing']['Alpha_deg'] = CalcInducedAOA(self.GEOM['Wing']['X_m'],
                                                         self.MASS['CG_m'][0],
                                                         self.EQM['VelRot_BodyAx_radps'][1],
                                                         self.ATM['TAS_mps'],
                                                         self.AERO['Wing']['Incidence_deg'],
                                                         self.ATM['Alpha_deg'],
                                                         self.AERO['Wing']['EPS_deg'])
    
        self.AERO['Wing']['Beta_deg'] = CalcInducedBETA(self.GEOM['Wing']['X_m'],
                                                        self.MASS['CG_m'][0],
                                                        self.EQM['VelRot_BodyAx_radps'][2],
                                                        self.ATM['TAS_mps'],
                                                        np.array([0,0]),
                                                        self.ATM['Beta_deg'],
                                                        np.array([0,0]))
        
        self.AERO['Fus']['Beta_deg'] = self.ATM['Beta_deg']
        
        # Calculate Coefficients in Stability Axis
        # CL and CD for the Flat Plate model - Jie Xu - Learning to Fly: Computational Controller Design for Hybrid ...-
        self.AERO['Wing']['CDS']   = 2*self.sind(self.AERO['Wing']['Alpha_deg'])*self.sind(self.AERO['Wing']['Alpha_deg'])*abs(self.cosd(self.AERO['Wing']['Beta_deg']))
        self.AERO['Wing']['CYS']   = np.array([0,0])
        self.AERO['Wing']['CLS']   = 2*self.sind(self.AERO['Wing']['Alpha_deg'])*self.cosd(self.AERO['Wing']['Alpha_deg'])*abs(self.cosd(self.AERO['Wing']['Beta_deg']))
        self.AERO['Wing']['CRS25'] = np.array([0,0])
        self.AERO['Wing']['CMS25'] = np.array([0,0])
        self.AERO['Wing']['CNS25'] = np.array([0,0])
        
        self.AERO['Fus']['CDS']   = 0.1
        self.AERO['Fus']['CYS']   = - 0.4 * self.sind(self.AERO['Fus']['Beta_deg'])
        self.AERO['Fus']['CLS']   = 0
        self.AERO['Fus']['CRS25'] = 0
        self.AERO['Fus']['CMS25'] = 0
        self.AERO['Fus']['CNS25'] = 0
       
        # Calculate Coefficcient in Body Axis - Rotate using Aricraft Alpha_deg
 
        self.AERO['Wing']['CDB']   = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Wing']['CDS'] 
                                      - self.sind(self.ATM['Alpha_deg']) * self.AERO['Wing']['CLS'] )
        self.AERO['Wing']['CYB']   = self.AERO['Wing']['CYS']
        self.AERO['Wing']['CLB']   = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Wing']['CLS'] 
                                      + self.sind(self.ATM['Alpha_deg']) * self.AERO['Wing']['CDS'] )    
      
        self.AERO['Wing']['CXB']   = -self.AERO['Wing']['CDB']
        self.AERO['Wing']['CZB']   = -self.AERO['Wing']['CLB']
        
        self.AERO['Wing']['CRB25'] = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Wing']['CRS25'] 
                                      - self.sind(self.ATM['Alpha_deg']) * self.AERO['Wing']['CNS25'] )
        self.AERO['Wing']['CMB25'] = self.AERO['Wing']['CMS25'] 
        self.AERO['Wing']['CNB25'] = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Wing']['CNS25'] 
                                      + self.sind(self.ATM['Alpha_deg']) * self.AERO['Wing']['CRS25'] )    
        
        
        self.AERO['Fus']['CDB']   = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Fus']['CDS'] 
                                      - self.sind(self.ATM['Alpha_deg']) * self.AERO['Fus']['CLS'] )
        self.AERO['Fus']['CYB']   = self.AERO['Fus']['CYS']
        self.AERO['Fus']['CLB']   = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Fus']['CLS'] 
                                      + self.sind(self.ATM['Alpha_deg']) * self.AERO['Fus']['CDS'] )    
      
        self.AERO['Fus']['CXB']   = -self.AERO['Fus']['CDB']
        self.AERO['Fus']['CZB']   = -self.AERO['Fus']['CLB']
        
        self.AERO['Fus']['CRB25'] = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Fus']['CRS25'] 
                                      - self.sind(self.ATM['Alpha_deg']) * self.AERO['Fus']['CNS25'] )
        self.AERO['Fus']['CMB25'] = np.array([0,0])
        self.AERO['Fus']['CNB25'] = (+ self.cosd(self.ATM['Alpha_deg']) * self.AERO['Fus']['CNS25'] 
                                      + self.sind(self.ATM['Alpha_deg']) * self.AERO['Fus']['CRS25'] )    


        # Calculate Moments in CG
        self.AERO['Wing']['CRBCG'] = ( + self.AERO['Wing']['CZB'] * (self.GEOM['Wing']['Y_m'] - self.MASS['CG_m'][1]) / self.GEOM['Wing']['b_m']
                                       + self.AERO['Wing']['CYB'] * (self.GEOM['Wing']['Z_m'] - self.MASS['CG_m'][2]) / self.GEOM['Wing']['b_m']
                                       + self.AERO['Wing']['CRS25'] )
                                      
        self.AERO['Wing']['CMBCG'] = ( + self.AERO['Wing']['CZB'] * (self.GEOM['Wing']['X_m'] - self.MASS['CG_m'][0]) / self.GEOM['Wing']['cma_m']
                                       - self.AERO['Wing']['CXB'] * (self.GEOM['Wing']['Z_m'] - self.MASS['CG_m'][2]) / self.GEOM['Wing']['cma_m']
                                       + self.AERO['Wing']['CMS25'] )
        
        self.AERO['Wing']['CNBCG'] = ( - self.AERO['Wing']['CYB'] * (self.GEOM['Wing']['X_m'] - self.MASS['CG_m'][0]) / self.GEOM['Wing']['b_m']
                                       - self.AERO['Wing']['CXB'] * (self.GEOM['Wing']['Y_m'] - self.MASS['CG_m'][1]) / self.GEOM['Wing']['b_m']
                                       + self.AERO['Wing']['CNS25'] )

        self.AERO['Fus']['CRBCG'] = ( + self.AERO['Fus']['CZB'] * (self.GEOM['Fus']['Y_m'] - self.MASS['CG_m'][1]) / self.GEOM['Fus']['b_m']
                                       + self.AERO['Fus']['CYB'] * (self.GEOM['Fus']['Z_m'] - self.MASS['CG_m'][2]) / self.GEOM['Fus']['b_m']
                                       + self.AERO['Fus']['CRS25'] )
                                      
        self.AERO['Fus']['CMBCG'] = ( + self.AERO['Fus']['CZB'] * (self.GEOM['Fus']['X_m'] - self.MASS['CG_m'][0]) / self.GEOM['Fus']['cma_m']
                                       - self.AERO['Fus']['CXB'] * (self.GEOM['Fus']['Z_m'] - self.MASS['CG_m'][2]) / self.GEOM['Fus']['cma_m']
                                       + self.AERO['Fus']['CMS25'] )
        
        self.AERO['Fus']['CNBCG'] = ( - self.AERO['Fus']['CYB'] * (self.GEOM['Fus']['X_m'] - self.MASS['CG_m'][0]) / self.GEOM['Fus']['b_m']
                                       - self.AERO['Fus']['CXB'] * (self.GEOM['Fus']['Y_m'] - self.MASS['CG_m'][1]) / self.GEOM['Fus']['b_m']
                                       + self.AERO['Fus']['CNS25'] )
        
        # Calculate Surfaces Forces and Moments
        self.AERO['Wing']['FXB_N']   = self.AERO['Wing']['CXB'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing']['S_m2']
        self.AERO['Wing']['FYB_N']   = self.AERO['Wing']['CYB'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing']['S_m2']
        self.AERO['Wing']['FZB_N']   = self.AERO['Wing']['CZB'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing']['S_m2']
        self.AERO['Wing']['MXB_Nm']  = self.AERO['Wing']['CRBCG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing']['S_m2'] * self.GEOM['Wing']['b_m']
        self.AERO['Wing']['MYB_Nm']  = self.AERO['Wing']['CMBCG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing']['S_m2'] * self.GEOM['Wing']['cma_m']
        self.AERO['Wing']['MZB_Nm']  = self.AERO['Wing']['CNBCG'] * self.ATM['DynPres_Pa'] * self.GEOM['Wing']['S_m2'] * self.GEOM['Wing']['b_m']

        self.AERO['Fus']['FXB_N']   = self.AERO['Fus']['CXB'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2']
        self.AERO['Fus']['FYB_N']   = self.AERO['Fus']['CYB'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2']
        self.AERO['Fus']['FZB_N']   = self.AERO['Fus']['CZB'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2']
        self.AERO['Fus']['MXB_Nm']  = self.AERO['Fus']['CRBCG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2'] * self.GEOM['Fus']['b_m']
        self.AERO['Fus']['MYB_Nm']  = self.AERO['Fus']['CMBCG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2'] * self.GEOM['Fus']['cma_m']
        self.AERO['Fus']['MZB_Nm']  = self.AERO['Fus']['CNBCG'] * self.ATM['DynPres_Pa'] * self.GEOM['Fus']['S_m2'] * self.GEOM['Fus']['b_m']
        
        # Calculate Total Forces and Moments
        self.AERO['TotalForce_BodyAx_N']  = self.OPT['UseAeroForce'] * (
                                            np.array([np.sum( self.AERO['Wing']['FXB_N'] ),
                                                       np.sum( self.AERO['Wing']['FYB_N'] ),
                                                       np.sum( self.AERO['Wing']['FZB_N'] )]) +
                                             np.array([np.sum( self.AERO['Fus']['FXB_N'] ),
                                                       np.sum( self.AERO['Fus']['FYB_N'] ),
                                                       np.sum( self.AERO['Fus']['FZB_N'] )]) )
                                            
        self.AERO['TotalMoment_BodyAx_Nm'] = self.OPT['UseAeroMoment'] * (
                                             np.array([np.sum( self.AERO['Wing']['MXB_Nm'] ),
                                                       np.sum( self.AERO['Wing']['MYB_Nm'] ),
                                                       np.sum( self.AERO['Wing']['MZB_Nm'] )]) + 
                                              np.array([np.sum( self.AERO['Fus']['MXB_Nm'] ),
                                                       np.sum( self.AERO['Fus']['MYB_Nm'] ),
                                                       np.sum( self.AERO['Fus']['MZB_Nm'] )]))

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
                
        # u_Vert = +(0.1*self.EQM['sta'][8] + 0.0*self.EQM['sta_dot'][8] + 0.05*self.EQM['sta_int'][8])
        
        u_Vert = action_vec[self.action_names.index('Throttle')]
        u_Pitc = action_vec[self.action_names.index('PitchThrottle')]
        u_Roll = action_vec[self.action_names.index('RollThrottle')]
        u_Yaw  = action_vec[self.action_names.index('YawThrottle')]
        
        if not(self.trimming):
            self.CONT['LastRPM_vec'] = self.CONT['RPM_vec'].copy()
        self.CONT['RPM_vec']     = ControlMixer(VerticalControlAllocation(u_Vert),PitchControlAllocation(u_Pitc),RollControlAllocation(u_Roll),YawControlAllocation(u_Yaw))
        # print(self.CONT['RPM_vec'] )
        TILT_vec = (np.array([action_vec[self.action_names.index('W1_Tilt')],
                              action_vec[self.action_names.index('W2_Tilt')]])+1)/2
        
        # RPM_vec = action_vec[0:self.MOT['n_motor']]
        # TILT_vec = action_vec[self.MOT['n_motor']:self.MOT['n_motor'] + self.CONT['n_TiltSurf']]

        self.CONT['RPM_p']  = self.CONT['RPM_vec'] ** (1/2) 
        self.CONT['Tilt_p']   = TILT_vec
        
        self.CONT['Elev1_p'] = action_vec[self.action_names.index('W1_Elevator')] - 0.5*action_vec[self.action_names.index('W1_Aileron')]
        self.CONT['Elev2_p'] = action_vec[self.action_names.index('W1_Elevator')] + 0.5*action_vec[self.action_names.index('W1_Aileron')]
        self.CONT['Elev3_p'] = action_vec[self.action_names.index('W2_Elevator')] - 0.5*action_vec[self.action_names.index('W2_Aileron')]
        self.CONT['Elev4_p'] = action_vec[self.action_names.index('W2_Elevator')] + 0.5*action_vec[self.action_names.index('W2_Aileron')]

