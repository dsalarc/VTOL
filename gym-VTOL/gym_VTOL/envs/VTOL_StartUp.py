#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  9 22:58:00 2021

@author: dsalarc
"""
import numpy as np

def StartUp (self):
 
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
  self.GEOM['Wing']['cma_m'] = np.array([1,1])
  self.GEOM['Wing']['b_m']   = np.array([8,8])
  self.GEOM['Wing']['S_m2']  = np.array([8,8])
  self.GEOM['Wing']['X_m']   = np.array([0.3,3.4])
  self.GEOM['Wing']['Y_m']   = np.array([0,0])
  self.GEOM['Wing']['Z_m']   = np.array([0,0])
  
  self.GEOM['Fus']          = {}
  self.GEOM['Fus']['cma_m'] = np.array([3])
  self.GEOM['Fus']['b_m']   = np.array([.9])
  self.GEOM['Fus']['S_m2']  = np.array([np.pi * 0.45**2])
  self.GEOM['Fus']['X_m']   = np.array([1.85])
  self.GEOM['Fus']['Y_m']   = np.array([0])
  self.GEOM['Fus']['Z_m']   = np.array([0])
 
  # MASS
  self.MASS['Pax']             = np.array([1,1])
  self.MASS['PaxWeight_kgf']   = 100
  self.MASS['EmptyWeight_kgf'] = 820 
  self.MASS['Weight_kgf'] = self.MASS['EmptyWeight_kgf'] + np.sum(self.MASS['Pax']) * self.MASS['PaxWeight_kgf']
  self.MASS['Empty_CG_m'] = np.array([1.59 , 0.0 , 0.43])
  self.MASS['PaxPos_m'] = np.array([[0.9 , 0.0 , 1.0],
                                    [2.5 , 0.0 , 1.0]])
  self.MASS['CG_m'] = np.array(self.MASS['Empty_CG_m']    * self.MASS['EmptyWeight_kgf'] + 
                               self.MASS['PaxPos_m'][0,:] * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                               self.MASS['PaxPos_m'][1,:] * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf']) / self.MASS['Weight_kgf']
  Pax2CG_sq = (self.MASS['PaxPos_m'] - self.MASS['CG_m'])**2                  
  Emp2CG_sq = (self.MASS['Empty_CG_m'] - self.MASS['CG_m'])**2                  
  Ixx = 2774 + ((Pax2CG_sq[0][1] + Pax2CG_sq[0][2]) * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                (Pax2CG_sq[1][1] + Pax2CG_sq[1][2]) * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf'] + 
                (Emp2CG_sq[1]    + Emp2CG_sq[2])    * self.MASS['EmptyWeight_kgf'])
  Iyy = 1812 + ((Pax2CG_sq[0][0] + Pax2CG_sq[0][2]) * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                (Pax2CG_sq[1][0] + Pax2CG_sq[1][2]) * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf'] +
                (Emp2CG_sq[0]    + Emp2CG_sq[2])    * self.MASS['EmptyWeight_kgf'])
  Izz = 3740 + ((Pax2CG_sq[0][0] + Pax2CG_sq[0][1]) * self.MASS['Pax'][0] * self.MASS['PaxWeight_kgf'] +
                (Pax2CG_sq[1][0] + Pax2CG_sq[1][1]) * self.MASS['Pax'][1] * self.MASS['PaxWeight_kgf'] +
                (Emp2CG_sq[0]    + Emp2CG_sq[1])    * self.MASS['EmptyWeight_kgf'])
  self.MASS['I_kgm'] = np.array([[Ixx,0,0],[0,Iyy,0],[0,0,Izz]])
                               
  
  # AERO
  self.AERO['Wing'] = {}
  self.AERO['Fus']  = {}

  # MOTOR
  x1 = 0.0375
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

  self.MOT['MaxRPM']        = np.ones(self.MOT['n_motor']) * 3000
  self.MOT['MinRPM']        = np.ones(self.MOT['n_motor']) * 0.01
  self.MOT['RPMRange']      = self.MOT['MaxRPM'] - self.MOT['MinRPM'] 
  self.MOT['Diameter_m']    = np.ones(self.MOT['n_motor']) * 1.5
  self.MOT['RotationSense'] = np.array([-1,+1,-1,+1,
                                        +1,-1,+1,-1])  
  
  # CT e CP - Vide planilha
  self.MOT['CT_J']       = np.array([[0.0 , 0.01 , 0.80] , 
                                     [0.14, 0.14 , 0.00]])
  self.MOT['CP_J']       = np.array([[0.0 , 0.01 , 0.80] , 
                                     [0.06, 0.06 , 0.01]])

  self.MOT['TiltSurf_link']  = np.array([0,0,0,0,1,1,1,1])                 #ID of surface which the motor is linked. Every motor will rotate the same amount
  
  # CONTROL
  self.CONT['n_TiltSurf']    = 2
  self.CONT['MinTilt_deg']   = np.ones(self.CONT['n_TiltSurf']) * 0
  self.CONT['MaxTilt_deg']   = np.ones(self.CONT['n_TiltSurf']) * 90
  self.CONT['TiltRange_deg'] = self.CONT['MaxTilt_deg'] - self.CONT['MinTilt_deg'] 
 
  # OTHER CONSTANTS
