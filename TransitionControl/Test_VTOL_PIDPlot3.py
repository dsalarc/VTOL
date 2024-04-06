#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 20:10:33 2021

@author: dsalarc
"""

# MULTIPLOT OPTIONS
line1 = 'k'
line2 = 'b'
line3 = 'g'
# line1 = 'k'
# line2 = 'k--'
# line2 = 'r'

# SimTime = n_steps*TestEnv.t_step
TimeVec = np.arange(0,SimTime,TestEnv.t_step)
if (SimTime - TimeVec[-1]) > TestEnv.t_step/2:
    TimeVec = np.append(TimeVec,SimTime)

PlotTime1 = 0
PlotTime2 = SimTime
fig = plt.figure()

linewidth = 1
plt.rcParams.update({'font.size': 7})

plt_l = 4
plt_c = 4
plt_n = 1
ax = []
axx = plt.subplot(plt_l,plt_c,plt_n); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['H_m'],line1, linewidth = linewidth, label='Altitude [m]')
plt.legend(loc='best')
plt.ylabel('Altitude [m]')
plt.xlabel('Time [s]')
plt.ylim([90,+110])


axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['VX_mps'],line2, linewidth = linewidth, label='VX [mps]')
plt.plot(TimeVec,SaveVec['CAS_mps'],line3, linewidth = linewidth, label='CAS [mps]')
plt.plot(TimeVec,-SaveVec['VZ_mps'],line1, linewidth = linewidth, label='-VZ [mps]')
try:
    plt.plot(Ref['VX_mps'][0,:],Ref['VX_mps'][1,:],'r:', linewidth = linewidth, label = 'VX ref')
except:
    pass
plt.legend(loc='best')
plt.ylabel('Body Speed [m/s]')
plt.xlabel('Time [s]')
plt.ylim([-5,+100])


# axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.yticks(np.arange(-180, 180, 90))
# # plt.ylim([1200,1300])
# plt.plot(TimeVec,SaveVec['Alpha_deg'],'k', linewidth = linewidth, label=r'$\alpha$')
# plt.ylabel('Angles [deg]')
# plt.legend(loc='best')
# plt.xlabel('Time [s]')
# plt.ylim([-5,+185])


axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([-5,+45])
plt.plot(TimeVec,SaveVec['Q_degps'],line2, linewidth = linewidth, label='Q')
plt.plot(TimeVec,SaveVec['Theta_deg'],line1, linewidth = linewidth, label=r'$\theta$')
plt.ylabel(r'$\theta$ [deg] / q [deg/s]')
plt.xlabel('Time [s]')
plt.ylim([-10,+10])
plt.legend(loc='best')


# plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# # plt.ylim([-5,+5])
# plt.plot(TimeVec,SaveVec['P_degps'], linewidth = linewidth, label='P')
# plt.plot(TimeVec,SaveVec['Phi_deg'], linewidth = linewidth, label='Phi')
# plt.ylabel('Phi [deg] / P [deg/s]')
# plt.xlabel('Time [s]')
# plt.ylim([-10,+10])
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([-5,+5])
plt.plot(TimeVec,SaveVec['Elevon3'],line1, linewidth = linewidth, label='Elevator')
plt.ylabel('Elevon Deflection [deg]')
plt.xlabel('Time [s]')
plt.ylim([-15,+15])
# plt.legend(loc='best')


# plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# # plt.ylim([-5,+5])
# plt.plot(TimeVec,SaveVec['R_degps'], linewidth = linewidth, label='R')
# plt.plot(TimeVec,SaveVec['Psi_deg'], linewidth = linewidth, label='Psi')
# plt.ylabel('Psi [deg] / R [deg/s]')
# plt.xlabel('Time [s]')
# plt.ylim([-10,+10])
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.ylim([0,3500])
plt.plot(TimeVec,SaveVec['RPM_1'],line2, linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['RPM_5'],line1, linewidth = linewidth, label='Back')
plt.ylabel('RPM')
plt.legend(loc='best')
plt.xlabel('Time [s]')


axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.ylim([0,100])
plt.plot(TimeVec,SaveVec['W1_Incidence_deg'],line2, linewidth = linewidth, label='Wing 1')
plt.plot(TimeVec,SaveVec['W2_Incidence_deg'],line1, linewidth = linewidth, label='Wing 2')
plt.ylabel('Wing Incidence [deg]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.ylim([0,100])
plt.plot(TimeVec,SaveVec['W1_Alpha_deg'], line2, linewidth = linewidth, label='Wing 1')
plt.plot(TimeVec,SaveVec['W2_Alpha_deg'], line1, linewidth = linewidth, label='Wing 2')
plt.ylabel('Wing AOA [deg]')
plt.legend(loc='best')
plt.xlabel('Time [s]')

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Thrust1_N'],line2, linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['Thrust5_N'],line1, linewidth = linewidth, label='Back')
plt.ylabel('Thrust [N]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
plt.ylim([0,3000])

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Throttle1_N'],line2, linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['Throttle5_N'],line1, linewidth = linewidth, label='Back')
plt.ylabel('Throttle [u]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
plt.ylim([0,1])

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Charge1_Ah'],line2, linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['Charge5_Ah'],line1, linewidth = linewidth, label='Back')
plt.ylabel('Charge Consummed [Ah]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
# plt.ylim([0,3000])

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Current1_A'],line2, linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['Current5_A'],line1, linewidth = linewidth, label='Back')
# plt.plot(TimeVec,SaveVec['AvgCurrent1_A'],'b', linewidth = linewidth, label='Front')
# plt.plot(TimeVec,SaveVec['AvgCurrent5_A'],'r', linewidth = linewidth, label='Back')
plt.ylabel('Current [A]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
# plt.ylim([0,3000])

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Voltage1_v'],line2, linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['Voltage5_v'],line1, linewidth = linewidth, label='Back')
plt.ylabel('Voltage [V]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
# plt.ylim([0,3000])

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['act_Throttle'],line1, linewidth = linewidth)
plt.ylabel('Throttle Action [u]')
plt.xlabel('Time [s]')

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['act_PitchThrottle'],line1, linewidth = linewidth)
plt.ylabel('Pitch Throttle Action [u]')
plt.xlabel('Time [s]')

# axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.plot(TimeVec,SaveVec['Reward'],line1, linewidth = linewidth)
# plt.ylabel('Reward [u]')
# plt.xlabel('Time [s]')

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['WindX_mps'],line1, linewidth = linewidth, label = 'Fuselage')
plt.plot(TimeVec,SaveVec['WindX_W1_mps'],line2, linewidth = linewidth,  label = 'Wing 1')
plt.plot(TimeVec,SaveVec['WindX_W2_mps'],line3, linewidth = linewidth,  label = 'Wind 2')
plt.ylabel('WindX [m/s]')
plt.xlabel('Time [s]')

axx = plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1; ax.append(axx)
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['WindZ_mps'],line1, linewidth = linewidth, label = 'Fuselage')
plt.plot(TimeVec,SaveVec['WindZ_W1_mps'],line2, linewidth = linewidth,  label = 'Wing 1')
plt.plot(TimeVec,SaveVec['WindZ_W2_mps'],line3, linewidth = linewidth,  label = 'Wind 2')
plt.ylabel('WindZ [m/s]')
plt.xlabel('Time [s]')


# plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.plot(TimeVec,SaveVec['J1'], linewidth = linewidth, label='1')
# plt.ylabel('Advance Ratio')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.xlabel('Time [s]')

# plt.subplot(plt_l,plt_c,plt_n,sharex=ax[0]); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.plot(TimeVec,SaveVec['FZaero_N'], linewidth = linewidth, label='Z Aero')
# plt.plot(TimeVec,SaveVec['FZ_N'], linewidth = linewidth, label='Z Tot')
# plt.ylabel('Forces [N]')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.xlabel('Time [s]')

# figManager = plt.get_current_fig_manager()
# figManager.window.showMaximized()
fig.set_size_inches(14, 8)
plt.tight_layout(w_pad=0.1, h_pad=0.1)
plt.show()

# print('Weight = {:0.0f}'.format(SaveVec['Weight_kgf'][0]))

# fig.savefig('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/AircraftModel_PID' + '_W' + str('{:0.0f}'.format(SaveVec['W_mps'][0])) +'_T' + str('{:0.0f}'.format(SaveVec['Theta_deg'][0])) +'_P' + str('{:0.0f}'.format(SaveVec['Phi_deg'][0])) +'_P' + str('{:0.0f}'.format(SaveVec['Psi_deg'][0])) + '.pdf', bbox_inches='tight')
# print('Fig Saved')
