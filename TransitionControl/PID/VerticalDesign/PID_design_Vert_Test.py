#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  2 21:35:32 2023

@author: dsalarc
"""
import gym
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interp
from PID_design_VertCosts import CalculateIndividualCosts, CalculateTotalCost
from PID_design_VertClosedLoops import gen_ClosedLoops
from PID_design_VertPlots import gen_Plots
from PID_design_VertFunctions import gen_EngActuator, gen_Controller, gen_Sensor, gen_Aircraft
import pickle

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
except:
    pass

plt.close('all')

# %%LOAD MODEL    
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')


# %% VERT CONTROLLER
with open('SavedGains_VerticalController_20231229_1038_Nelder-Mead_smooth.pkl', 'rb') as fp:
    aux = pickle.load(fp)

TrimVec = aux['TrimVec']
Vert_gains = aux['GainsVec'].copy()
Vert_gains['VX_mps'] = aux['TrimVec']['VX_mps'][:]

# %% TEST VECTOR
TestVec = {}
# TestVec['VX_mps'] = np.array([0.0, 30.0, 60.0])#TrimVec['VX_mps']
TestVec['VX_mps'] = TrimVec['VX_mps']
TestVec['AX_mps2'] = np.array([-5.0, -2.5, 0.0, +2.5, +5.0])
TestVec['CostVec'] = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))

# %%
if len(TestVec['AX_mps2']) ==5:
    line_type_vec = ['--' ,'-.' , '-' ,'-.' ,  '--']
elif len(TestVec['AX_mps2']) == 3:
    line_type_vec = ['--' ,'-' , '--']
elif len(TestVec['AX_mps2']) == 1:
    line_type_vec = ['-']
else:
    raise Exception('Adjust line_type_vec')

SaveAircraft = []
SaveTrim = []

# %% GENERAL FUNCTIONS

EngActuator = gen_EngActuator(wn_radps = 40,  inp_name = 'ThrottleCmd_u', out_name = 'Throttle_u' , act_name = 'EngActuator')
Sensor_vz   = gen_Sensor(wn_radps = 40, inp_name = 'VZ_mps', out_name = 'VZ_sen_mps' , sensor_name = 'Sensor_vz')
Sensor_z    = gen_Sensor(wn_radps = 40, inp_name = 'Z_m', out_name = 'Z_sen_m' , sensor_name = 'Sensor_z')
Sensor_az   = gen_Sensor(wn_radps = 40, inp_name = 'AZi_mps2', out_name = 'AZi_sen_mps2' , sensor_name = 'Sensor_az')

for nv in range(len(TestVec['VX_mps'])):
    for nt in range(len(TestVec['AX_mps2'])):

        VX_mps = TestVec['VX_mps'][nv]
        AX_mps2 = TestVec['AX_mps2'][nt]
        Elevator_deg = None

        # Generate Vert Controller for specific speed
        GainsVert = {}
        for kk in Vert_gains.keys():
            f = interp.interp1d(Vert_gains['VX_mps'] , Vert_gains[kk])
            GainsVert[kk] = f(VX_mps)
                    
        AltController = gen_Controller(GainsVert)

        # Generate Aircraft Closed Loop Controller
        Aircraft, TrimData = gen_Aircraft(
            TestEnv, VX_mps=VX_mps, AX_mps2=AX_mps2, Elevator_deg=Elevator_deg)
        SaveAircraft.append(Aircraft)
        SaveTrim.append(TrimData)
        
        # Generate Full Closed Loop Model
        ClosedLoops = gen_ClosedLoops(
                Aircraft, AltController, Sensor_vz , Sensor_z, Sensor_az, EngActuator)
        Criteria    = CalculateIndividualCosts(ClosedLoops)
        TotalCost   = CalculateTotalCost(Criteria)
        TestVec['CostVec'][nv, nt] = TotalCost
        
        color_u1 = (TestVec['VX_mps'][nv] - np.min(TestVec['VX_mps'])) / (np.max(TestVec['VX_mps']) - np.min(TestVec['VX_mps']))
        if len(TestVec['AX_mps2']) == 1:
            color_u2 = 0
        else:
            color_u2 = np.abs(TestVec['AX_mps2'][nt]) / np.max(np.abs(TestVec['AX_mps2']))
            
        if ((nv == len(TestVec['VX_mps'])-1) and (nt == len(TestVec['AX_mps2'])-1)):
            plot_criteria = True
            plot_legend = True
        else:
            plot_criteria = False
            plot_legend = False
                
        # gen_Plots(ClosedLoops, Criteria, ('%0.1f m/s / AX %0.1f m/s2' %(TestVec['VX_mps'][nv],TestVec['AX_mps2'][nt])), color_rgb = ((1-color_u1),color_u2,color_u1), line_type = line_type_vec[nt], plot_criteria = plot_criteria, plot_legend = plot_legend)
        # gen_Plots(ClosedLoops, Criteria, ('$%0.1f m/s$' %(TestVec['VX_mps'][nv])), color_rgb = ((1-color_u1),color_u2,color_u1), line_type = line_type_vec[nt], plot_criteria = plot_criteria, plot_legend = plot_legend)

# %% COST PLOT
fig = plt.figure()
plt.rcParams.update({'font.size': 10})

plt_l = int(np.ceil(np.sqrt(len(Vert_gains.keys()))))
plt_c = int(np.ceil(len(Vert_gains.keys()) / plt_l))

color = '-ob'
for nt in range(len(TestVec['AX_mps2'])):
    plt_n = 0
    for kk in Vert_gains.keys():
        if kk != 'VX_mps':
            plt_n += 1
            plt.subplot(plt_l, plt_c, plt_n)
            plt.grid('on')
            plt.xlim([np.min(Vert_gains['VX_mps']),np.max(Vert_gains['VX_mps'])])
            plt.plot(Vert_gains['VX_mps'] , Vert_gains[kk],color, linewidth = 2)
            plt.xlabel('CAS [m/s]')
            plt.ylabel(kk)
            plt.xticks(np.arange(0, 61, 5))

        
plt_n += 1
plt.subplot(plt_l, plt_c, plt_n)
plt.grid('on')
plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
plt.plot(TestVec['VX_mps'] , TestVec['CostVec'],color, linewidth = 2)
plt.xlabel('CAS [m/s]')
plt.xticks(np.arange(0, 61, 5))
plt.ylabel('Cost')
fig.set_size_inches(8, 6)
fig.tight_layout(w_pad=0, h_pad=0.0,rect = (0,0,1,0.95))
plt.savefig('FinalGains_VerticalController.pdf', dpi=fig.dpi)


# %% RESHAPE SAVED DATA

SaveTrim = np.reshape(SaveTrim,(len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))

# %% PLOT TEST POINTS
TrimRes = {}

TrimRes['Trimmed']       = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['W1_Tilt_deg']   = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['W2_Tilt_deg']   = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['PitchThrottle'] = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Elev2_u']       = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Throttle']      = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Thrust_1']      = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Thrust_5']      = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['RPM_1']         = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['RPM_5']         = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['RPM_5']         = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Elevon3']       = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Elevon4']       = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['i1_A']          = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['i2_A']          = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['V1_V']          = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['V2_V']          = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Throttle1_p']   = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Throttle5_p']   = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['Reward']        = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['W1_CLS']        = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['W2_CLS']        = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))
TrimRes['AX_mps2']        = np.zeros((len(TestVec['VX_mps']) , len(TestVec['AX_mps2'])))

for n_t in range(len(TestVec['AX_mps2'])):
    for n_sp in range(len(TestVec['VX_mps'])):
        
        if SaveTrim[n_sp,n_t]['Trimmed'] > 0.5:
            c = 1
        else:
            c = np.nan
    
        TrimRes['Trimmed'][n_sp,n_t]       = c*SaveTrim[n_sp,n_t]['Trimmed']
        TrimRes['W1_Tilt_deg'][n_sp,n_t]   = c*SaveTrim[n_sp,n_t]['info']['AERO']['Wing1']['Incidence_deg']
        TrimRes['W2_Tilt_deg'][n_sp,n_t]   = c*SaveTrim[n_sp,n_t]['info']['AERO']['Wing2']['Incidence_deg']
        TrimRes['PitchThrottle'][n_sp,n_t] = c*SaveTrim[n_sp,n_t]['Action'][TestEnv.action_names.index('PitchThrottle')]
        TrimRes['Elev2_u'][n_sp,n_t]       = c*SaveTrim[n_sp,n_t]['Action'][TestEnv.action_names.index('W2_Elevator')]
        TrimRes['Throttle'][n_sp,n_t]      = c*SaveTrim[n_sp,n_t]['Action'][TestEnv.action_names.index('Throttle')]
        TrimRes['Thrust_1'][n_sp,n_t]      = c*SaveTrim[n_sp,n_t]['info']['MOT']['Thrust_N'][0]
        TrimRes['Thrust_5'][n_sp,n_t]      = c*SaveTrim[n_sp,n_t]['info']['MOT']['Thrust_N'][4]
        TrimRes['RPM_1'][n_sp,n_t]         = c*SaveTrim[n_sp,n_t]['info']['MOT']['RPM'][0]
        TrimRes['RPM_5'][n_sp,n_t]         = c*SaveTrim[n_sp,n_t]['info']['MOT']['RPM'][4]
        TrimRes['Elevon3'][n_sp,n_t]       = c*SaveTrim[n_sp,n_t]['info']['CONT']['Elevon_deg'][2]
        TrimRes['Elevon4'][n_sp,n_t]       = c*SaveTrim[n_sp,n_t]['info']['CONT']['Elevon_deg'][3]
        TrimRes['i1_A'][n_sp,n_t]          = c*SaveTrim[n_sp,n_t]['info']['MOT']['ASSEMBLY']['obj'][0].MOTOR.i_A
        TrimRes['i2_A'][n_sp,n_t]          = c*SaveTrim[n_sp,n_t]['info']['MOT']['ASSEMBLY']['obj'][4].MOTOR.i_A
        TrimRes['V1_V'][n_sp,n_t]          = c*SaveTrim[n_sp,n_t]['info']['MOT']['ASSEMBLY']['obj'][0].V_V
        TrimRes['V2_V'][n_sp,n_t]          = c*SaveTrim[n_sp,n_t]['info']['MOT']['ASSEMBLY']['obj'][4].V_V
        TrimRes['Throttle1_p'][n_sp,n_t]   = c*SaveTrim[n_sp,n_t]['info']['CONT']['Throttle_p'][0]
        TrimRes['Throttle5_p'][n_sp,n_t]   = c*SaveTrim[n_sp,n_t]['info']['CONT']['Throttle_p'][4]
        TrimRes['W1_CLS'][n_sp,n_t]        = c*SaveTrim[n_sp,n_t]['info']['AERO']['Wing1']['CLS_25Local']
        TrimRes['W2_CLS'][n_sp,n_t]        = c*SaveTrim[n_sp,n_t]['info']['AERO']['Wing2']['CLS_25Local']
        TrimRes['AX_mps2'][n_sp,n_t]        = c*SaveTrim[n_sp,n_t]['info']['EQM']['AccLin_EarthAx_mps2'][0]


colors = ['-or', '-om', '-ok', '-oc', '-ob']
# colors = ['-ob']
fig = plt.figure()
plt.rcParams.update({'font.size': 10})

plt_l = 3
plt_c = 4
XTICKS = np.arange(0, 61, 10)
for n_t in range(len(TestVec['AX_mps2'])):
    color = colors[n_t]
    plt_n = 1
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['W1_Tilt_deg'][:,n_t],color, linewidth = 2, label = 'AX: %0.1f m/s'%TestVec['AX_mps2'][n_t])
    plt.ylim([0, 15])
    plt.yticks(np.arange(0,100.1,20))
    plt.xlabel('CAS [m/s]')
    plt.ylabel('Wing Tilt [deg]')
    plt.xticks(XTICKS)
    # plt.legend()

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['Throttle'][:,n_t],color, linewidth = 2)
    plt.ylim([-1, +1])
    plt.xlabel('CAS [m/s]')
    plt.ylabel('Throttle Action')
    plt.xticks(XTICKS)

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['PitchThrottle'][:,n_t],color, linewidth = 2)
    # plt.ylim([-0.05, 0.05])
    plt.xlabel('CAS [m/s]')
    plt.ylabel('PitchThrottle [p]')
    plt.xticks(XTICKS)


    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['Elevon3'][:,n_t],color, linewidth = 2)
    plt.ylim([-15, 15])
    plt.yticks(np.arange(-15,15.1,5))
    plt.xlabel('CAS [m/s]')
    plt.ylabel('Elevon [deg]')
    plt.xticks(XTICKS)

   
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['Thrust_1'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 1250])
    plt.xlabel('CAS [m/s]')
    plt.ylabel('Thrust [N] \n Front Engines')
    plt.xticks(XTICKS)

   
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['Thrust_5'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 1250])
    plt.xlabel('CAS [m/s]')
    plt.ylabel('Thrust [N] \n Back Engines')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['RPM_1'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 2500])
    plt.xlabel('CAS [m/s]')
    plt.ylabel('RPM \n Front Engines')
    plt.xticks(XTICKS)

   
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['RPM_5'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 2500])
    plt.xlabel('CAS [m/s]')
    plt.ylabel('RPM \n Back Engines')
    plt.xticks(XTICKS)

   
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['Throttle1_p'][:,n_t],color, linewidth = 2)
    plt.xlabel('CAS [m/s]')
    plt.ylim([0, 0.6])
    plt.ylabel('Throttle [p] \n Front Engines')
    plt.xticks(XTICKS)

   
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['Throttle5_p'][:,n_t],color, linewidth = 2, label = ('$AX = %0.1f m/s^2$'%(TestVec['AX_mps2'][n_t])) )
    plt.xlabel('CAS [m/s]')
    plt.ylim([0, 0.6])
    plt.ylabel('Throttle [p] \n Back Engines')
    plt.xticks(XTICKS)
   
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    plt.plot(TestVec['VX_mps'] , TrimRes['i1_A'][:,n_t]+TrimRes['i2_A'][:,n_t],color, linewidth = 2, label = ('$AX = %0.1f m/s^2$'%(TestVec['AX_mps2'][n_t])) )
    plt.xlabel('CAS [m/s]')
    plt.ylabel('Total Electrical \n Current [A]')
    plt.xticks(XTICKS)
   
    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TestVec['VX_mps']),np.max(TestVec['VX_mps'])])
    # plt.plot(TestVec['VX_mps'] , TrimRes['AX_mps2'][:,n_t],color, linewidth = 2)
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('Inertial X Acceleration [m/s2]')
    # plt.xticks(XTICKS)



fig.set_size_inches(12, 6)
fig.tight_layout(w_pad=0, h_pad=0.0,rect = (0,0,1,0.95))

# fig.set_size_inches(12, 6)
# fig.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
plt.show()
plt.legend(bbox_to_anchor=(1.2, 1))

plt.savefig('LinearController_DesignPoints.pdf', dpi=fig.dpi)