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

linewidth = 1
plt.rcParams.update({'font.size': 10})

plt_l = 3
plt_c = 3
plt_n = 0
plt.subplots(plt_l, plt_c)

ax1 = plt.subplot(plt_l,plt_c,plt_n+1); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['W1_Alpha_deg'],'b', linewidth = linewidth, label='W1')
plt.plot(TimeVec,SaveVec['W2_Alpha_deg'],'r', linewidth = linewidth, label='W2')
plt.ylabel('Alpha [deg]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n+1, sharex=ax1); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['AERO_W1_CLS_25Local'],'b', linewidth = linewidth, label='W1')
plt.plot(TimeVec,SaveVec['AERO_W2_CLS_25Local'],'r', linewidth = linewidth, label='W2')
plt.ylabel('CLS 25Local')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n+1, sharex=ax1); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['AERO_W1_CDS_25Local'],'b', linewidth = linewidth, label='W1')
plt.plot(TimeVec,SaveVec['AERO_W2_CDS_25Local'],'r', linewidth = linewidth, label='W2')
plt.ylabel('CDS 25Local')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n+1, sharex=ax1); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['AERO_W1_CMS_25Local'],'b', linewidth = linewidth, label='W1')
plt.plot(TimeVec,SaveVec['AERO_W2_CMS_25Local'],'r', linewidth = linewidth, label='W2')
plt.ylabel('CMS 25Local')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n+1, sharex=ax1); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['AERO_W1_CLB_CG'],'b', linewidth = linewidth, label='W1')
plt.plot(TimeVec,SaveVec['AERO_W2_CLB_CG'],'r', linewidth = linewidth, label='W2')
plt.ylabel('CLB CG')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n+1, sharex=ax1); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['AERO_W1_CDB_CG'],'b', linewidth = linewidth, label='W1')
plt.plot(TimeVec,SaveVec['AERO_W2_CDB_CG'],'r', linewidth = linewidth, label='W2')
plt.ylabel('CDB_CG')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n+1, sharex=ax1); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['AERO_W1_CMB_CG'],'b', linewidth = linewidth, label='W1')
plt.plot(TimeVec,SaveVec['AERO_W2_CMB_CG'],'r', linewidth = linewidth, label='W2')
plt.ylabel('CMB CG')
plt.legend(loc='best')
plt.xlabel('Time [s]')


# fig.set_size_inches(7, 5)
fig.tight_layout() 
plt.show()
