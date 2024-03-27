#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 20:21:50 2023

@author: dsalarc
"""
import gym
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
from PID_design_VertCosts import CalculateIndividualCosts, CalculateTotalCost
from PID_design_VertClosedLoops import gen_ClosedLoops
from PID_design_VertPlots import gen_Plots
from PID_design_VertFunctions import gen_EngActuator, gen_Controller, gen_Sensor, gen_Aircraft
import control as ct
import time
import pickle
from datetime import datetime
from doepy import build

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
except:
    pass

NoteToSave = ''

# %%LOAD MODEL
env_dict = gym.envs.registration.registry.env_specs.copy()
for env in env_dict:
    if 'Vahana_VertFlight-v0' in env:
        print("Remove {} from registry".format(env))
        del gym.envs.registration.registry.env_specs[env]
        
TestEnv = gym.make('gym_VTOL:Vahana_VertFlight-v0')

opt_type = 'Nelder-Mead'
# opt_type = 'TNC'
# opt_type = 'SLSQP'
# opt_type = 'Powell'
# opt_type = 'L-BFGS-B'
# opt_type = 'BFGS'
# %% DEFINE TRIM VECTOR
TrimVec = {}
TrimVec['VX_mps'] = np.arange(0.0, 60.1, 5.0)
TrimVec['AX_mps2'] = np.array([-5.0, -2.5, 0.0, +2.5, +5.0])

# TrimVec['VX_mps']        = np.array([ 0.0 , 40.0, 60.0])
# TrimVec['AX_mps2']  = np.array([-2.5, 0.0, +2.5])
# TrimVec['AX_mps2']  = np.array([0.0])

TrimVec['TrimTilt_deg'] = 0.0*TrimVec['VX_mps']
TrimVec['TestTilt_deg'] = np.zeros(
    (len(TrimVec['VX_mps']), len(TrimVec['AX_mps2'])))

# %% INITIALIZE GAINS VECTOR
GainsVec = {}
GainsVec['Kvzp']  = 0.0*TrimVec['VX_mps'] 
GainsVec['Kvzi']  = 0.0*TrimVec['VX_mps'] 
GainsVec['Kz']    = 0.0*TrimVec['VX_mps'] 
GainsVec['Kvzff'] = 0.0*TrimVec['VX_mps'] 
GainsVec['Knzp']  = 0.0*TrimVec['VX_mps'] 
CostVec = np.zeros( len(TrimVec['VX_mps']))

def GeneralFunction(InpGains = [0.04, 0.02GainsVec, 0.9, 0.0, 0.0], AircraftList = None):

    Gains = {}
    Gains['Kvzp']  = InpGains[0]
    Gains['Kvzi']  = InpGains[1]
    Gains['Kz']    = InpGains[2]
    Gains['Kvzff'] = InpGains[3]
    Gains['Knzp']  = InpGains[4]
    
    TotalCost = 0
    if AircraftList == None:
        TotalCost = 1e3
        raise TypeError('GeneralFunction called without AircraftList')
    else:
        TotalCost = 0
        
        Controller = gen_Controller(Gains)
    
        for i in range(len(AircraftList)):
            ClosedLoops = gen_ClosedLoops(AircraftList[i], Controller, Sensor_vz, Sensor_z, Sensor_az, EngActuator)
            Criteria    = CalculateIndividualCosts(ClosedLoops)
            TotalCost   += CalculateTotalCost(Criteria)

    return TotalCost

# %%
SaveAircraft = []
SaveTrim = []
Trimmed = np.zeros((len(TrimVec['VX_mps']), len(TrimVec['AX_mps2'])))

for nv_trim in range(len(TrimVec['VX_mps'])):
    # Do a nominal trim first
    Elevator_deg = None
    
    obs = TestEnv.reset(VX_mps=TrimVec['VX_mps'][nv_trim], VZ_mps=0.0, Elevator_deg=Elevator_deg, THETA=0.0, DispMessages=False,
                        TermTheta_deg=45, StaFreezeList=[], UNC_seed=None, UNC_enable=0)
    TrimTilt_deg = TestEnv.TrimData['info']['AERO']['Wing1']['Incidence_deg']

    print(' ')
    print("Optimizing Speed %d / %d" % (nv_trim+1, len(TrimVec['VX_mps'])))
    
    # Generate Aircraft Linear model
    SaveAircraft_Speed = []
    for nt in range(len(TrimVec['AX_mps2'])):
        nt_trim = nt
        TestAX_mps2 = TrimVec['AX_mps2'][nt_trim]
        
        print("Trimming Acc %d / %d" % (nt+1, len(TrimVec['AX_mps2'])))
        Aircraft, TrimData = gen_Aircraft(
            TestEnv, VX_mps=TrimVec['VX_mps'][nv_trim], AX_mps2=TestAX_mps2, Elevator_deg=Elevator_deg)
        Trimmed[nv_trim, nt_trim] = TrimData['Trimmed']
        TrimVec['TestTilt_deg'][nv_trim, nt_trim] = TrimData['info']['AERO']['Wing1']['Incidence_deg']
        
        Trimmed[nv_trim, nt_trim] = TrimData['Trimmed']
        if np.abs(Trimmed[nv_trim, nt_trim] - 1) > 0.1:
            SaveAircraft.append(Aircraft)
            SaveTrim.append(TrimData)
            print('Condition not trimmed')

        else:
            EngActuator = gen_EngActuator(wn_radps = 40,  inp_name = 'ThrottleCmd_u', out_name = 'Throttle_u' , act_name = 'EngActuator')
            Sensor_vz   = gen_Sensor(wn_radps = 40, inp_name = 'VZ_mps', out_name = 'VZ_sen_mps' , sensor_name = 'Sensor_vz')
            Sensor_z    = gen_Sensor(wn_radps = 40, inp_name = 'Z_m', out_name = 'Z_sen_m' , sensor_name = 'Sensor_z')
            Sensor_az   = gen_Sensor(wn_radps = 40, inp_name = 'AZi_mps2', out_name = 'AZi_sen_mps2' , sensor_name = 'Sensor_az')

            SaveAircraft.append(Aircraft)
            SaveAircraft_Speed.append(Aircraft)
            SaveTrim.append(TrimData)    
    # %% OPTIMIZE FOR ALL TILTS

    if (nv_trim == 0):
        x0 = [-0.10, -0.025, 0.9, -0.01, -0.009]
    else :
        x0 = [GainsVec['Kvzp'][nv_trim-1], 
              GainsVec['Kvzi'][nv_trim-1], 
              GainsVec['Kz'][nv_trim-1], 
              GainsVec['Kvzff'][nv_trim-1],
              GainsVec['Knzp'][nv_trim-1]]

    bnds = ((-2.0, +0.0),
            (-0.1, +0.0),
            (+0.1, +5.0),
            (-0.1, +0.0),
            (-0.1, -0.0))

    DOEGain = 1.5
    DOE = build.full_fact(
    {'Kvzp':[x0[0]/DOEGain , x0[0], x0[0]*DOEGain],
     'Kvzi':[x0[1]/DOEGain , x0[1], x0[1]*DOEGain],
       'Kz':[x0[2]/DOEGain , x0[2], x0[2]*DOEGain],
    'Kvzff':[x0[3]/DOEGain , x0[3], x0[3]*DOEGain],
     'Knzp':[x0[4]/DOEGain , x0[4], x0[4]*DOEGain]})

    DOECost = np.zeros((len(DOE['Kvzp']),1))
    print('Running DOE ...')
    for i in range(len(DOE['Kvzp'])):
        DOECost[i] = GeneralFunction(np.array([DOE['Kvzp'][i] , DOE['Kvzi'][i] , DOE['Kz'][i] , DOE['Kvzff'][i] , DOE['Knzp'][i]]), SaveAircraft_Speed)
    n_min = np.argmin(DOECost)
    x1 = np.array([DOE['Kvzp'][n_min] , 
                   DOE['Kvzi'][n_min] , 
                   DOE['Kz'][n_min] ,
                   DOE['Kvzff'][n_min] , 
                   DOE['Knzp'][n_min]])
    
    t = time.time()
    print('Running Optimization ...')
    OptGains = opt.minimize(GeneralFunction, x1, args=(
        SaveAircraft_Speed), method=opt_type, bounds=bnds)
    elapsed = time.time() - t
    print(opt_type + ' Elapsed Time [s]: %0.1f' % (elapsed))          
    
    # SAVE GAINS
    GainsVec['Kvzp']   [nv_trim] = OptGains['x'][0]
    GainsVec['Kvzi']   [nv_trim] = OptGains['x'][1]
    GainsVec['Kz']     [nv_trim] = OptGains['x'][2]
    GainsVec['Kvzff']  [nv_trim] = OptGains['x'][3]
    GainsVec['Knzp']   [nv_trim] = OptGains['x'][4]   

    Gains = {}
    for kk in GainsVec.keys():
        Gains[kk] = GainsVec[kk][nv_trim]
    
    Controller  = gen_Controller(Gains)
    ClosedLoops = gen_ClosedLoops(Aircraft , Controller, Sensor_vz , Sensor_z, Sensor_az, EngActuator)
    Criteria    = CalculateIndividualCosts(ClosedLoops)
    TotalCost   = CalculateTotalCost(Criteria)
    CostVec[nv_trim] = TotalCost

    print('Final Cost: %0.2f' %(TotalCost))
    
    print_gains = ''
    for kk in GainsVec.keys():
        print_gains = print_gains + kk + ': ' + \
            format(GainsVec[kk][nv_trim], '0.4f') + ';  '
    print(print_gains)
    
# %% PLOT RESULTS
plt.close('all')
n_save = -1
line_type_vec = ['--' ,'-.' , '-' ,'-.' ,  '--']

for nv_trim in range(len(TrimVec['VX_mps'])):
    for nt in range(len(TrimVec['AX_mps2'])):
        nt_trim = nt

        TestTilt_deg = TrimVec['TestTilt_deg'][nv_trim, nt_trim]
        n_save += 1
        if Trimmed[nv_trim, nt_trim] != 1:
            print("Speed: %0.1f / Tilt: %02.1f / Not trimmed" %
                  (TrimVec['VX_mps'][nv_trim], TestTilt_deg))
        else:
            Gains = {}
            for kk in GainsVec.keys():
                Gains[kk] = GainsVec[kk][nv_trim]

            Controller = gen_Controller(Gains)
            ClosedLoops = gen_ClosedLoops(
                SaveAircraft[n_save], Controller, Sensor_vz , Sensor_z, Sensor_az, EngActuator)
            Criteria = CalculateIndividualCosts(ClosedLoops)
            TotalCost = CalculateTotalCost(Criteria)

            print("Speed: %0.1f / Tilt: %02.1f / TotalCost: %0.2f " %
                  (TrimVec['VX_mps'][nv_trim], TestTilt_deg, TotalCost))
            color_u1 = (TrimVec['VX_mps'][nv_trim] - np.min(TrimVec['VX_mps'])) / (np.max(TrimVec['VX_mps']) - np.min(TrimVec['VX_mps']))
            color_u2 = np.abs(TrimVec['AX_mps2'][nt_trim]) / np.max(np.abs(TrimVec['AX_mps2']))
            
            if ((nv_trim == len(TrimVec['VX_mps'])-1) and (nt == len(TrimVec['AX_mps2'])-1)):
                plot_criteria = True
            else:
                plot_criteria = False
            
            gen_Plots(ClosedLoops, Criteria, ('%0.1f m/s / Tilt %0.1f' %(TrimVec['VX_mps'][nv_trim],TestTilt_deg)), color_rgb = ((1-color_u1),color_u2,color_u1), line_type = line_type_vec[nt], plot_criteria = plot_criteria)
    
# %% PLOT GAINS

plt.figure('Gains')

keys = GainsVec.keys()
l = int(np.ceil(np.sqrt(1+len(keys))))
c = int(np.ceil(len(keys) / l))

for i in range(len(keys)-1):
    plt.subplot(l,c,i+1)
    plt.plot(TrimVec['VX_mps'], GainsVec[list(keys)[i]][:], 'o-')

    plt.legend()
    plt.grid('on')
    plt.ylabel(list(keys)[i])
    # plt.ylim(bnds[i])
    plt.xlabel('VX [mps]')
    
plt.subplot(l,c,i+2)
plt.plot(TrimVec['VX_mps'] , CostVec)
plt.grid('on')
plt.ylabel('Cost')
plt.xlabel('VX [mps]')
 
plt.tight_layout()
plt.show()

File2Save = {'GainsVec': GainsVec, 'TrimVec': TrimVec, 'CostVec': CostVec, 'SaveTrim': SaveTrim, 'NoteToSave': NoteToSave}

now = datetime.today()
save_name = 'SavedGains_VerticalController_' + '{:04.0f}'.format(now.year) + '{:02.0f}'.format(now.month) + '{:02.0f}'.format(now.day) + '_' + '{:02.0f}'.format(now.hour) + '{:02.0f}'.format(now.minute) + '_' + opt_type +'.pkl'
with open(save_name, 'wb') as fp:
    pickle.dump(File2Save, fp)

# %% PLOT DESIGN POINTS

SaveTrim = np.reshape(SaveTrim,(len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))

plt_l = 3
plt_c = 4

plt.rcParams.update({'font.size': 10})
fig = plt.figure()
TrimRes = {}

TrimRes['Trimmed']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['W1_Tilt_deg']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['W2_Tilt_deg']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['PitchThrottle'] = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Elev2_u']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Throttle']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Thrust_1']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Thrust_5']      = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['RPM_1']         = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['RPM_5']         = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['RPM_5']         = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Elevon3']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Elevon4']       = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['i1_A']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['i2_A']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['V1_V']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['V2_V']          = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Throttle1_p']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Throttle5_p']   = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['Reward']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['W1_CLS']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['W2_CLS']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
TrimRes['AX_mps2']        = np.zeros((len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))

for n_t in range(len(TrimVec['AX_mps2'])):
    for n_sp in range(len(TrimVec['VX_mps'])):
        
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


colors = ['r', 'm', 'k', 'c', 'b']
for n_t in range(len(TrimVec['AX_mps2'])):
    color = colors[n_t]
    plt_n = 1
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['W1_Tilt_deg'][:,n_t],color, linewidth = 2, label = 'AX: %0.1f m/s'%TrimVec['AX_mps2'][n_t])
    plt.ylim([0, 90])
    plt.yticks(np.arange(0,120,30))
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Wing Tilt [deg]')
    plt.legend()

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle'][:,n_t],color, linewidth = 2)
    plt.ylim([-1, +1])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Throttle Action')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['PitchThrottle'][:,n_t],color, linewidth = 2)
    plt.ylim([-0.10, 0.05])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('PitchThrottle [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Elevon3'][:,n_t],color, linewidth = 2)
    plt.ylim([-15, 15])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Elevon [deg]')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_1'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 1250])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Thrust - Front Engines [N]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_5'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 1250])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Thrust - Back Engines [N]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_1'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 3000])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('RPM - Front Engines')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_5'][:,n_t],color, linewidth = 2)
    plt.ylim([0, 3000])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('RPM - Back Engines')


    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['i1_A'][:,n_t],color, linewidth = 2,label='Motor 1')
    # plt.plot(TrimVec['VX_mps'] , TrimRes['i2_A'][:,n_t],'k--', linewidth = 2,label='Motor 5')
    # # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([0, 180])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('Current [A]')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['V1_V'],color, linewidth = 2,label='Motor 1')
    # plt.plot(TrimVec['VX_mps'] , TrimRes['V2_V'],'k--', linewidth = 2,label='Motor 5')
    # # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([0, 450])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('Voltage [V]')

    # plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    # plt.grid('on')
    # plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    # plt.plot(TrimVec['VX_mps'] , TrimRes['W1_CLS'][:,n_t],color, linewidth = 2,label='Motor 1')
    # plt.plot(TrimVec['VX_mps'] , TrimRes['W2_CLS'][:,n_t],'k--', linewidth = 2,label='Motor 5')
    # # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # plt.legend(loc='best')
    # plt.ylim([0, 2])
    # plt.xlabel('Inertial X Speed [m/s]')
    # plt.ylabel('Wing CLS')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle1_p'][:,n_t],color, linewidth = 2)
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Throttle - Front Engines [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle5_p'][:,n_t],color, linewidth = 2)
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Throttle - Back Engines [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['AX_mps2'][:,n_t],color, linewidth = 2)
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Inertial X Acceleration [m/s2]')



# fig.set_size_inches(8, 5)
fig.set_size_inches(14, 8)
fig.tight_layout() 
 