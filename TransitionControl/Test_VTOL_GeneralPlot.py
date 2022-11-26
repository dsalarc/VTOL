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

plt_l = 4
plt_c = 3
plt_n = 1

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['VX_mps'],label='VX [mps]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.ylabel('Body Speed [m/s]')
plt.xlabel('Time [s]')
# plt.ylim([-10,+10])

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['VX_mps'],label='VX [mps]')
plt.plot(TimeVec,SaveVec['VY_mps'],label='VY [mps]')
plt.plot(TimeVec,-SaveVec['VZ_mps'],label='-VZ [mps]')
plt.plot(VZ_Ref[0,:],-VZ_Ref[1,:],'k--',label = '-W ref')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.ylabel('Body Speed [m/s]')
plt.xlabel('Time [s]')
plt.ylim([-10,+50])

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.yticks(np.arange(-180, 180, 90))
# plt.ylim([1200,1300])
plt.plot(TimeVec,SaveVec['Alpha_deg'],label='Alpha')
plt.plot(TimeVec,SaveVec['Beta_deg'],label='Beta')
plt.ylabel('Angles [deg]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([-5,+45])
plt.plot(TimeVec,SaveVec['Q_degps'],label='Q')
plt.plot(TimeVec,SaveVec['Theta_deg'],label='Theta')
plt.plot(The_Ref[0,:],np.rad2deg(The_Ref[1,:]),'k--')
plt.ylabel('Theta [deg] / Q [deg/s]')
plt.xlabel('Time [s]')
plt.ylim([-10,+10])
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([-5,+5])
plt.plot(TimeVec,SaveVec['P_degps'],label='P')
plt.plot(TimeVec,SaveVec['Phi_deg'],label='Phi')
plt.plot(Phi_Ref[0,:],np.rad2deg(Phi_Ref[1,:]),'k--')
plt.ylabel('Phi [deg] / P [deg/s]')
plt.xlabel('Time [s]')
plt.ylim([-10,+10])
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([-5,+5])
plt.plot(TimeVec,SaveVec['R_degps'],label='R')
plt.plot(TimeVec,SaveVec['Psi_deg'],label='Psi')
plt.plot(Psi_Ref[0,:],np.rad2deg(Psi_Ref[1,:]),'k--')
plt.ylabel('Psi [deg] / R [deg/s]')
plt.xlabel('Time [s]')
plt.ylim([-10,+10])
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.ylim([0,4000])
plt.plot(TimeVec,SaveVec['RPM_1'],label='1')
plt.plot(TimeVec,SaveVec['RPM_4'],label='4')
plt.plot(TimeVec,SaveVec['RPM_5'],label='5')
plt.plot(TimeVec,SaveVec['RPM_8'],label='8')
plt.ylabel('RPM')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.ylim([-1.5,+1.5])
plt.plot(TimeVec,u_Vert[1:],label='Vertical')
plt.plot(TimeVec,u_Pitch[1:],label='Pitch')
plt.plot(TimeVec,u_Roll[1:],label='Roll')
plt.plot(TimeVec,u_Yaw[1:],label='Yaw')
plt.ylabel('Norm Command')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')


plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.ylim([0,100])
plt.plot(TimeVec,SaveVec['W1_Incidence_deg'],label='Wing 1')
plt.plot(TimeVec,SaveVec['W2_Incidence_deg'],label='Wing 2')
plt.ylabel('Wing Incidence [deg]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

# plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
# plt.grid('on')
# plt.xlim([PlotTime1,PlotTime2])
# plt.ylim([0,100])
# plt.plot(TimeVec,SaveVec['W1_Alpha_deg'],label='Wing 1')
# plt.plot(TimeVec,SaveVec['W2_Alpha_deg'],label='Wing 2')
# plt.ylabel('Wing AOA [deg]')
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['Thrust1_N'],label='1')
plt.plot(TimeVec,SaveVec['Thrust4_N'],label='4')
plt.plot(TimeVec,SaveVec['Thrust5_N'],label='5')
plt.plot(TimeVec,SaveVec['Thrust8_N'],label='8')
plt.ylabel('Thrust [N]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['J1'],label='1')
plt.ylabel('Advance Ratio')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
plt.grid('on')
plt.xlim([PlotTime1,PlotTime2])
plt.plot(TimeVec,SaveVec['FZaero_N'],label='Z Aero')
plt.plot(TimeVec,SaveVec['FZ_N'],label='Z Tot')
plt.ylabel('Forces [N]')
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.xlabel('Time [s]')

fig.set_size_inches(20, 10)
fig.tight_layout() 
plt.show()

print('Weight = {:0.0f}'.format(SaveVec['Weight_kgf'][0]))

fig.savefig('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/AircraftModel_PID' + '_W' + str('{:0.0f}'.format(SaveVec['W_mps'][0])) +'_T' + str('{:0.0f}'.format(SaveVec['Theta_deg'][0])) +'_P' + str('{:0.0f}'.format(SaveVec['Phi_deg'][0])) +'_P' + str('{:0.0f}'.format(SaveVec['Psi_deg'][0])) + '.pdf', bbox_inches='tight')
print('Fig Saved')
