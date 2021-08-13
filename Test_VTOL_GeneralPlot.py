#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 20:10:33 2021

@author: dsalarc
"""

SimTime = n_steps*TestEnv.t_step
TimeVec = np.arange(TestEnv.t_step,SimTime,TestEnv.t_step)
if (SimTime - TimeVec[-1]) > TestEnv.t_step/2:
    TimeVec = np.append(TimeVec,SimTime)

PlotTime1 = 0
PlotTime2 = SimTime
fig = plt.figure()

plt_l = 5
plt_c = 2
plt_n = 1

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Z_m'])
plt.ylabel('Z [m]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['U_mps'],label='U [mps]')
plt.plot(TimeVec,SaveVec['V_mps'],label='V [mps]')
plt.plot(TimeVec,SaveVec['W_mps'],label='W [mps]')
plt.plot(WRef[0,:],-WRef[1,:],'k--')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.ylabel('Body Speed [m/s]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['AY_mps2'],label='AY')
plt.plot(TimeVec,SaveVec['AZ_mps2'],label='AZ')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.ylabel('Acc [m/s²]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['MY_Nm'],label='MY Total')
plt.plot(TimeVec,SaveVec['MYaero_Nm'],label='MY Aero')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.ylabel('MYa [N]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Theta_deg'])
plt.plot(ThetaRef[0,:],ThetaRef[1,:],'k--')
plt.ylabel('Theta [deg]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Phi_deg'])
plt.plot(PhiRef[0,:],PhiRef[1,:],'k--')
plt.ylabel('Phi [deg]')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([1900,2400])
plt.plot(TimeVec,SaveVec['RPM_1'],label='1')
plt.plot(TimeVec,SaveVec['RPM_4'],label='4')
plt.plot(TimeVec,SaveVec['RPM_5'],label='5')
plt.ylabel('RPM')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# # plt.ylim([1200,1300])
# plt.plot(TimeVec,SaveVec['Thrust1_N'],label='1')
# plt.plot(TimeVec,SaveVec['Thrust4_N'],label='4')
# plt.plot(TimeVec,SaveVec['Thrust5_N'],label='5')
# plt.ylabel('Thrust [N]')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([1200,1300])
# plt.plot(TimeVec,SaveVec['FYaero_N'],label='FYaero')
plt.plot(TimeVec,SaveVec['FX_N'],label='FX Total')
plt.ylabel('Force [N]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.yticks(np.arange(-180, 180, 90))
# plt.ylim([1200,1300])
plt.plot(TimeVec,SaveVec['Alpha_deg'],label='Alpha')
plt.ylabel('Angles [deg]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.yticks(np.arange(-180, 180, 90))
plt.plot(TimeVec,SaveVec['Beta_deg'],label='Beta')
plt.ylabel('Angles [deg]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

fig.set_size_inches(10, 10)
fig.tight_layout() 

plt.show()
