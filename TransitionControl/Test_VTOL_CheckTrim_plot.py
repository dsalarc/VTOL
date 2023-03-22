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
plt.plot(TimeVec,SaveVec['H_m'],'k', linewidth = linewidth, label='Altitude [m]')
plt.legend(loc='best')
plt.ylabel('Altitude [m]')
plt.xlabel('Time [s]')
plt.ylim([-1+SaveVec['H_m'][0],+1+SaveVec['H_m'][0]])


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['VX_mps'],'k', linewidth = linewidth, label='VX [mps]')
plt.plot(TimeVec,-SaveVec['VZ_mps'],'k--', linewidth = linewidth, label='-VZ [mps]')
try:
    plt.plot(Ref['VX_mps'][0,:],Ref['VX_mps'][1,:],'r:', linewidth = linewidth, label = 'VX ref')
except:
    pass
plt.legend(loc='best')
plt.ylabel('Body Speed [m/s]')
plt.xlabel('Time [s]')
plt.ylim([-5+min(SaveVec['VX_mps'][0] , SaveVec['VZ_mps'][0]),+5+max(SaveVec['VX_mps'][0] , SaveVec['VZ_mps'][0])])


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.yticks(np.arange(-180, 180, 90))
# plt.ylim([1200,1300])
plt.plot(TimeVec,SaveVec['Alpha_deg'],'k', linewidth = linewidth, label=r'$\alpha$')
plt.ylabel('Angles [deg]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
plt.ylim([-1+SaveVec['Alpha_deg'][0],+1+SaveVec['Alpha_deg'][0]])


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([-5,+45])
plt.plot(TimeVec,SaveVec['Q_degps'],'k', linewidth = linewidth, label='Q')
plt.plot(TimeVec,SaveVec['Theta_deg'],'k--', linewidth = linewidth, label=r'$\theta$')
plt.ylabel(r'$\theta$ [deg] / q [deg/s]')
plt.xlabel('Time [s]')
plt.ylim([-1+SaveVec['Theta_deg'][0],+1+SaveVec['Theta_deg'][0]])
plt.legend(loc='best')


# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# # plt.ylim([-5,+5])
# plt.plot(TimeVec,SaveVec['P_degps'], linewidth = linewidth, label='P')
# plt.plot(TimeVec,SaveVec['Phi_deg'], linewidth = linewidth, label='Phi')
# plt.ylabel('Phi [deg] / P [deg/s]')
# plt.xlabel('Time [s]')
# plt.ylim([-10,+10])
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([-5,+5])
plt.plot(TimeVec,SaveVec['Elevon3'],'k', linewidth = linewidth, label='Elevator')
plt.ylabel('Elevon Deflection [deg]')
plt.xlabel('Time [s]')
plt.ylim([-1+SaveVec['Elevon3'][0],+1+SaveVec['Elevon3'][0]])
# plt.legend(loc='best')


# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# # plt.ylim([-5,+5])
# plt.plot(TimeVec,SaveVec['R_degps'], linewidth = linewidth, label='R')
# plt.plot(TimeVec,SaveVec['Psi_deg'], linewidth = linewidth, label='Psi')
# plt.ylabel('Psi [deg] / R [deg/s]')
# plt.xlabel('Time [s]')
# plt.ylim([-10,+10])
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# # plt.ylim([-1.5,+1.5])
# plt.plot(TimeVec,u_Pitch[1:],'k', linewidth = linewidth, label='PitchThrottle')
# plt.plot(TimeVec,u_Vert[1:],'k--', linewidth = linewidth, label='Throttle')
# plt.plot(TimeVec,u_Elev2[1:],'k:', linewidth = linewidth, label='Elevator')
# plt.ylabel('Norm Command')
# plt.legend(loc='best')
# plt.xlabel('Time [s]')
# plt.ylim([-0.2,+0.7])

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['RPM_1'],'k', linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['RPM_5'],'k--', linewidth = linewidth, label='Back')
plt.ylabel('RPM')
plt.legend(loc='best')
plt.xlabel('Time [s]')
plt.ylim([-5+min(SaveVec['RPM_1'][0] , SaveVec['RPM_1'][0]),+5+max(SaveVec['RPM_1'][0] , SaveVec['RPM_1'][0])])


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['W1_Incidence_deg'],'k', linewidth = linewidth, label='Wing 1')
plt.plot(TimeVec,SaveVec['W2_Incidence_deg'],'k--', linewidth = linewidth, label='Wing 2')
plt.ylabel('Wing Incidence [deg]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
plt.ylim([-5+min(SaveVec['W1_Incidence_deg'][0] , SaveVec['W2_Incidence_deg'][0]),+5+max(SaveVec['W1_Incidence_deg'][0] , SaveVec['W2_Incidence_deg'][0])])

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([0,100])
# plt.plot(TimeVec,SaveVec['W1_Alpha_deg'], linewidth = linewidth, label='Wing 1')
# plt.plot(TimeVec,SaveVec['W2_Alpha_deg'], linewidth = linewidth, label='Wing 2')
# plt.ylabel('Wing AOA [deg]')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Thrust1_N'],'k', linewidth = linewidth, label='Front')
plt.plot(TimeVec,SaveVec['Thrust5_N'],'k--', linewidth = linewidth, label='Back')
plt.ylabel('Thrust [N]')
plt.legend(loc='best')
plt.xlabel('Time [s]')
plt.ylim([0,3000])

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.plot(TimeVec,SaveVec['J1'], linewidth = linewidth, label='1')
# plt.ylabel('Advance Ratio')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.xlabel('Time [s]')

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.plot(TimeVec,SaveVec['FZaero_N'], linewidth = linewidth, label='Z Aero')
# plt.plot(TimeVec,SaveVec['FZ_N'], linewidth = linewidth, label='Z Tot')
# plt.ylabel('Forces [N]')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.xlabel('Time [s]')

fig.set_size_inches(7, 5)
fig.tight_layout() 
plt.show()

# print('Weight = {:0.0f}'.format(SaveVec['Weight_kgf'][0]))

# fig.savefig('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/AircraftModel_PID' + '_W' + str('{:0.0f}'.format(SaveVec['W_mps'][0])) +'_T' + str('{:0.0f}'.format(SaveVec['Theta_deg'][0])) +'_P' + str('{:0.0f}'.format(SaveVec['Phi_deg'][0])) +'_P' + str('{:0.0f}'.format(SaveVec['Psi_deg'][0])) + '.pdf', bbox_inches='tight')
# print('Fig Saved')
