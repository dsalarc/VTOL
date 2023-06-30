#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 20:10:33 2021

@author: dsalarc
"""

# SimTime = n_steps*TestEnv.t_step
TimeVec = np.arange(0,SimTime,TestEnv.t_step)
if (SimTime - TimeVec[-1]) > TestEnv.t_step/2:
    TimeVec = np.append(TimeVec,SimTime)

PlotTime1 = 0
PlotTime2 = SimTime
fig = plt.figure()

linewidth = 1
plt.rcParams.update({'font.size': 7})

plt_l = 3
plt_c = 3
plt_n = 1

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Q_degps'],'b', linewidth = linewidth, label='True')
plt.plot(TimeVec,np.rad2deg(SaveVec['SENS_Q_radps']),'r', linewidth = linewidth, label='Sensor')
plt.ylabel('Q [deg/s]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Theta_deg'],'b', linewidth = linewidth, label='True')
plt.plot(TimeVec,np.rad2deg(SaveVec['SENS_Theta_rad']),'r', linewidth = linewidth, label='Sensor')
plt.ylabel('Theta [deg]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['VX_mps'],'b', linewidth = linewidth, label='True')
plt.plot(TimeVec,SaveVec['SENS_VX_mps'],'r', linewidth = linewidth, label='Sensor')
plt.ylabel('VX [m/s]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['VZ_mps'],'b', linewidth = linewidth, label='True')
plt.plot(TimeVec,SaveVec['SENS_VZ_mps'],'r', linewidth = linewidth, label='Sensor')
plt.ylabel('VZ [m/s]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['NX_mps2'],'b', linewidth = linewidth, label='True')
plt.plot(TimeVec,SaveVec['SENS_NX_mps2'],'r', linewidth = linewidth, label='Sensor')
plt.ylabel('NX [g]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['NZ_mps2'],'b', linewidth = linewidth, label='True')
plt.plot(TimeVec,SaveVec['SENS_NZ_mps2'],'r', linewidth = linewidth, label='Sensor')
plt.ylabel('NZ [g]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['CAS_mps'],'b', linewidth = linewidth, label='True')
plt.plot(TimeVec,SaveVec['SENS_CAS_mps'],'r', linewidth = linewidth, label='Sensor')
plt.ylabel('CAS [m/s]')
plt.legend(loc='best')
plt.xlabel('Time [s]')



# fig.set_size_inches(7, 5)
fig.tight_layout() 
plt.show()
