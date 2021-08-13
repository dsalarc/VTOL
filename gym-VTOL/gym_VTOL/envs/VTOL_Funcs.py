#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 20:21:05 2021

@author: dsalarc
"""

def CONT_fcn(self,action_vec):
    def VerticalControlAllocation(u):    
      return np.array([+1,+1,+1,+1,+1,+1,+1,+1]) * u
   
    # RPM_vec = VerticalControlAllocation(action_vec[0:self.MOT['n_motor']])
    # TILT_vec = action_vec[self.MOT['n_motor']:self.MOT['n_motor'] + self.CONT['n_TiltSurf']]
    
    RPM_vec = action_vec[0:self.MOT['n_motor']]
    TILT_vec = action_vec[self.MOT['n_motor']:self.MOT['n_motor'] + self.CONT['n_TiltSurf']]

    self.CONT['RPM_p']  = RPM_vec ** (1/2) 
    self.CONT['Tilt_p'] = TILT_vec