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
import scipy.interpolate as interp
from PID_design_VertFwdCosts import CalculateIndividualCosts, CalculateTotalCost
from PID_design_VertFwdClosedLoops import gen_ClosedLoops
from PID_design_VertFwdPlots import gen_Plots
from PID_design_VertFwdFunctions import gen_Actuator, gen_AltController, gen_SpeedController, gen_Sensor, gen_Aircraft
import control as ct
import time
import pickle
from datetime import datetime
from doepy import build
import sys
import pandas
sys.path.insert(1, '/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PID/')
from PID_design_PitchFunctions import Controller as gen_PitchController
from PID_design_PitchFunctions import gen_ControlAllocation
from PID_design_PitchClosedLoops import PitchClosedLoops as gen_PitchClosedLoops

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

# %% LOAD PITCH CONTROLLER
with open('/home/dsalarc/Documents/DOUTORADO/Environments/VTOL/TransitionControl/PID/SavedGains_20231222_1134_Nelder-Mead_smooth.pkl', 'rb') as fp:
    aux = pickle.load(fp)
PitchCont_gains = aux['GainsVec'].copy()
PitchCont_gains['VX_mps'] = aux['TrimVec']['VX_mps'][:]

opt_type = 'Nelder-Mead'
# opt_type = 'TNC'
# opt_type = 'SLSQP'
# opt_type = 'Powell'
# opt_type = 'L-BFGS-B'
# opt_type = 'BFGS'
# %% DEFINE TRIM VECTOR
TrimVec = {}
# TrimVec['VX_mps']  = np.array([70.0])
TrimVec['AX_mps2'] = np.array([0.0])

TrimVec['VX_mps']   = np.arange(45.0, 70.1, 5.0)
# TrimVec['AX_mps2']  = np.array([0.0, +2.5])

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
GainsVec['Kvxp']  = 0.0*TrimVec['VX_mps'] 
GainsVec['Kvxi']  = 0.0*TrimVec['VX_mps'] 
GainsVec['Kaxp']  = 0.0*TrimVec['VX_mps'] 

CostVec = np.zeros( len(TrimVec['VX_mps']))

def GeneralFunction(InpGains = [-1.0, -0.05, 0.9, 0.0, 0.0, 0.1, 0.01, 0.0], AircraftList = None):

    Gains = {}
    Gains['Kvzp']  = InpGains[0]
    Gains['Kvzi']  = InpGains[1]
    Gains['Kz']    = InpGains[2]
    Gains['Kvzff'] = InpGains[3]
    Gains['Knzp']  = InpGains[4]
    Gains['Kvxp']  = InpGains[5]
    Gains['Kvxi']  = InpGains[6]
    Gains['Kaxp']  = InpGains[7]
    
    TotalCost = 0
    if AircraftList == None:
        TotalCost = 1e3
        raise TypeError('GeneralFunction called without AircraftList')
    else:
        TotalCost = 0
        
        AltController = gen_AltController(Gains)
        SpeedController = gen_SpeedController(Gains)
    
        for i in range(len(AircraftList)):
            ClosedLoops = gen_ClosedLoops(AircraftList[i] , AltController, SpeedController, EngActuator, Sensor_vx , Sensor_vz , Sensor_z, Sensor_ax, Sensor_az)
            Criteria    = CalculateIndividualCosts(ClosedLoops)
            TotalCost   += CalculateTotalCost(Criteria)

    return TotalCost

# %%
SaveAircraft = []
SaveAircraftPitch = []
SaveTrim = []
Trimmed = np.zeros((len(TrimVec['VX_mps']), len(TrimVec['AX_mps2'])))

print(' ')
print("Generating linear models")

SaveAircraftPitch_Speed = []
for nv_trim in range(len(TrimVec['VX_mps'])):
    VX_mps = TrimVec['VX_mps'][nv_trim]
    # Do a nominal trim first
    Elevator_deg = None
    
    obs = TestEnv.reset(VX_mps=VX_mps, VZ_mps=0.0, Elevator_deg=Elevator_deg, THETA=0.0, DispMessages=False,
                        TermTheta_deg=45, StaFreezeList=[], UNC_seed=None, UNC_enable=0)
    TrimTilt_deg = TestEnv.TrimData['info']['AERO']['Wing1']['Incidence_deg']

    
    # Generate Pitch Controller for specific speed
    GainsPitch = {}
    for kk in PitchCont_gains.keys():
        f = interp.interp1d(PitchCont_gains['VX_mps'] , PitchCont_gains[kk])
        GainsPitch[kk] = f(np.max((np.min((VX_mps,PitchCont_gains['VX_mps'][-1])),PitchCont_gains['VX_mps'][0])))
        
    PitchController = gen_PitchController(GainsPitch)
    ControlAllocation = gen_ControlAllocation(GainsPitch)
       
    # Generate Aircraft Linear model
    SaveAircraftPitch_Speed.append([])
    for nt in range(len(TrimVec['AX_mps2'])):
        nt_trim = nt
        TestAX_mps2 = TrimVec['AX_mps2'][nt_trim]
        
        print("Trimming Acc %d / %d" % (nt+1, len(TrimVec['AX_mps2'])))
        Aircraft, TrimData = gen_Aircraft(
            TestEnv, VX_mps=TrimVec['VX_mps'][nv_trim], AX_mps2=TestAX_mps2, Elevator_deg=Elevator_deg)
        
        Trimmed[nv_trim, nt_trim] = TrimData['Trimmed']
        TrimVec['TestTilt_deg'][nv_trim, nt_trim] = TrimData['info']['AERO']['Wing1']['Incidence_deg']
        
        if ( (np.abs(Trimmed[nv_trim, nt_trim] - 1) > 0.1) or (TrimData['Action'][0] < -0.99) or (TrimData['Action'][0] > +0.99)):
            SaveAircraftPitch.append([])
            SaveTrim.append(TrimData)
            print('Condition not trimmed')

        else:
            EngActuator  = gen_Actuator(wn_radps = 40,  inp_name = 'ThrottleCmd_u', out_name = 'Throttle_u' , act_name = 'EngActuator')
            EngPitchActuator  = gen_Actuator(wn_radps = 40,  inp_name = 'PitchCmd_u', out_name = 'PitchThrottle_u' , act_name = 'EngActuator')
            ElevActuator = gen_Actuator(wn_radps = 40,  inp_name = 'ElevatorCmd_u', out_name = 'Elevator_u' , act_name = 'ElevActuator')
            Sensor_vx   = gen_Sensor(wn_radps = 40, inp_name = 'VX_mps', out_name = 'VX_sen_mps' , sensor_name = 'Sensor_vx')
            Sensor_vz   = gen_Sensor(wn_radps = 40, inp_name = 'VZ_mps', out_name = 'VZ_sen_mps' , sensor_name = 'Sensor_vz')
            Sensor_z    = gen_Sensor(wn_radps = 40, inp_name = 'Z_m', out_name = 'Z_sen_m' , sensor_name = 'Sensor_z')
            Sensor_ax   = gen_Sensor(wn_radps = 40, inp_name = 'AXi_mps2', out_name = 'AXi_sen_mps2' , sensor_name = 'Sensor_ax')
            Sensor_az   = gen_Sensor(wn_radps = 40, inp_name = 'AZi_mps2', out_name = 'AZi_sen_mps2' , sensor_name = 'Sensor_az')
            Sensor_q    = gen_Sensor(wn_radps = 40, inp_name = 'Q_degps' , out_name = 'Q_sen_degps'  , sensor_name = 'Sensor_q')
            Sensor_t    = gen_Sensor(wn_radps = 40, inp_name = 'Theta_deg',out_name = 'Theta_sen_deg', sensor_name = 'Sensor_t')

            aux = gen_PitchClosedLoops(Aircraft , PitchController, Sensor_q , Sensor_t , EngPitchActuator, ElevActuator, ControlAllocation)
            AircraftPitch = {}
            AircraftPitch['SS'] = aux['AltitudeIncluded'].copy();
            AircraftPitch['SS'].name = 'AircraftPitch'
            
            # SaveAircraft.append(Aircraft)
            SaveAircraftPitch.append(AircraftPitch)
            SaveAircraftPitch_Speed[nv_trim].append(AircraftPitch)
            SaveTrim.append(TrimData)    
            
    # %% SECTION ONLY FOR TESTING
    if False:
        Gains = {}
        Gains['Kvzp']  = -1.0
        Gains['Kvzi']  = -0.05
        Gains['Kz']    = 0.9
        Gains['Kvzff'] = 0.0
        Gains['Knzp']  = 0.0
        Gains['Kvxp']  = 0.1
        Gains['Kvxi']  = 0.01
        Gains['Kaxp']  = 0.0
        AltController = gen_AltController(Gains)
        SpeedController = gen_SpeedController(Gains)
        ClosedLoops = gen_ClosedLoops(AircraftPitch , AltController, SpeedController, EngActuator, Sensor_vx , Sensor_vz , Sensor_z, Sensor_ax, Sensor_az)
        Criteria    = CalculateIndividualCosts(ClosedLoops)
        TotalCost   = CalculateTotalCost(Criteria)
        print(('Total Cost: %0.1f' %(TotalCost)))
        # T, yout = ct.step_response(ClosedLoops['Complete'] , T=20, input = 1)
    
        # l = int(np.ceil(np.sqrt(len(yout))))
        # c = int(np.ceil(len(yout) / l))
        # fig1 = plt.figure('Z_Cmd_Step')
        # for i in range(len(yout)):
        #     plt.subplot(l,c,i+1)
        #     plt.plot(T,yout[i][0], '-', color = 'b')
        #     plt.grid('on')
        #     plt.ylabel(ClosedLoops['Complete'].output_labels[i])
        #     plt.xlim([0,max(T)] )
        # plt.suptitle('Z Command Step')
        # figManager = plt.get_current_fig_manager()
        # figManager.window.showMaximized()
        # plt.show()
        
        # T, yout = ct.step_response(ClosedLoops['Complete'] , T=60, input = 2)
    
        # l = int(np.ceil(np.sqrt(len(yout))))
        # c = int(np.ceil(len(yout) / l))
        # fig2 = plt.figure('VX_Cmd_Step')
        # for i in range(len(yout)):
        #     plt.subplot(l,c,i+1)
        #     plt.plot(T,yout[i][0], '-', color = 'b')
        #     plt.grid('on')
        #     plt.ylabel(ClosedLoops['Complete'].output_labels[i])
        #     plt.xlim([0,max(T)] )
        # plt.suptitle('VX Command Step')
        # figManager = plt.get_current_fig_manager()
        # figManager.window.showMaximized()
        # plt.show()

        gen_Plots(ClosedLoops, Criteria, ('%0.1f m/s / Tilt %0.1f' %(TrimVec['VX_mps'][nv_trim],TrimVec['TestTilt_deg'][nv_trim, nt_trim])), color_rgb = (0,0,1), line_type = '-', plot_criteria = True)

SaveAircraftPitch = np.reshape(SaveAircraftPitch      ,(len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))
SaveTrim          = np.reshape(SaveTrim               ,(len(TrimVec['VX_mps']) , len(TrimVec['AX_mps2'])))

    # %% OPTIMIZE FOR ALL TILTS

for nv in range(len(TrimVec['VX_mps'])):
    nv_trim = len(TrimVec['VX_mps']) - nv - 1
    
    print(' ')
    print("Optimizing Speed %d / %d" % (nv+1, len(TrimVec['VX_mps'])))

    if (nv == 0):        
        x0 = [-0.8, -0.5, 0.7, -0.2, -0.1, 0.1, 0.01, 0.0]
    else :
        x0 = [GainsVec['Kvzp'][nv_trim+1], 
              GainsVec['Kvzi'][nv_trim+1], 
              GainsVec['Kz'][nv_trim+1], 
              GainsVec['Kvzff'][nv_trim+1], 
              GainsVec['Knzp'][nv_trim+1], 
              GainsVec['Kvxp'][nv_trim+1], 
              GainsVec['Kvxi'][nv_trim+1], 
              GainsVec['Kaxp'][nv_trim+1]]       

    # bnds = ((-2.0, +0.0),
    #         (-0.1, +0.0),
    #         (+0.1, +5.0),
    #         (-0.1, +0.0),
    #         (-0.1, +0.0),
    #         (-0.1, +0.0),
    #         (-0.1, +0.0),
    #         (+0.0, +1.0))

    DOEGain = 1.5
    # DOE = build.full_fact(
    # {'Kvzp':[x0[0]/DOEGain , x0[0], x0[0]*DOEGain],
    #  'Kvzi':[x0[1]/DOEGain , x0[1], x0[1]*DOEGain],
    #    'Kz':[x0[2]/DOEGain , x0[2], x0[2]*DOEGain],
    # 'Kvzff':[x0[3]/DOEGain , x0[3], x0[3]*DOEGain],
    #  'Knzp':[x0[4]/DOEGain , x0[4], x0[4]*DOEGain],
    #  'Kvxp':[x0[5]/DOEGain , x0[5], x0[5]*DOEGain],
    #  'Kvxi':[x0[6]/DOEGain , x0[6], x0[6]*DOEGain],
    #  'Kaxp':[x0[7]/DOEGain , x0[7], x0[7]*DOEGain]})
    
    # DOECost = np.zeros((len(DOE['Kvzp']),1))
    # print('Running DOE ...')
    # for i in range(len(DOE['Kvzp'])):
    #     DOECost[i] = GeneralFunction(np.array([DOE['Kvzp'][i] , DOE['Kvzi'][i] , DOE['Kz'][i]   , DOE['Kvzff'][i] , 
    #                                            DOE['Knzp'][i] , DOE['Kvxp'][i] , DOE['Kvxi'][i] , DOE['Kaxp'][i]]), SaveAircraftPitch_Speed)
    # n_min = np.argmin(DOECost)
    # x1 = np.array([DOE['Kvzp'][n_min] , 
    #                DOE['Kvzi'][n_min] , 
    #                DOE['Kz'][n_min] ,
    #                DOE['Kvzff'][n_min] , 
    #                DOE['Knzp'][n_min] , 
    #                DOE['Kvxp'][n_min] , 
    #                DOE['Kvxi'][n_min] , 
    #                DOE['Kaxp'][n_min]])
    
    x1 = x0[:]
    # CurrentCost = GeneralFunction(x1, SaveAircraftPitch_Speed)
    # for i in range(len(x1)):
    #     x_test_up = x1[:]
    #     x_test_up[i] = x1[i] * DOEGain
    #     x_test_dw = x1[:]
    #     x_test_dw[i] = x1[i] / DOEGain
        
    #     TestCost_up = GeneralFunction(x_test_up, SaveAircraftPitch_Speed)
    #     TestCost_dw = GeneralFunction(x_test_dw, SaveAircraftPitch_Speed)
    #     if TestCost_up < CurrentCost:
    #         CurrentCost = TestCost_up
    #         x1 = x_test_up
    #     if TestCost_dw < CurrentCost:
    #         CurrentCost = TestCost_dw
    #         x1 = x_test_dw
         
    t = time.time()
    print('Running Optimization ...')
    # OptGains = opt.minimize(GeneralFunction, x1, args=(
    #     SaveAircraftPitch_Speed), method=opt_type, bounds=bnds)
    OptGains = opt.minimize(GeneralFunction, x1, args=(
        SaveAircraftPitch_Speed[nv_trim]), method=opt_type)
    elapsed = time.time() - t
    print(opt_type + ' Elapsed Time [s]: %0.1f' % (elapsed))          
    
    # SAVE GAINS
    GainsVec['Kvzp'] [nv_trim] = OptGains['x'][0]
    GainsVec['Kvzi'] [nv_trim] = OptGains['x'][1]
    GainsVec['Kz']   [nv_trim] = OptGains['x'][2]
    GainsVec['Kvzff'][nv_trim] = OptGains['x'][3]
    GainsVec['Knzp'] [nv_trim] = OptGains['x'][4]   
    GainsVec['Kvxp'] [nv_trim] = OptGains['x'][5]   
    GainsVec['Kvxi'] [nv_trim] = OptGains['x'][6]   
    GainsVec['Kaxp'] [nv_trim] = OptGains['x'][7]   

    Gains = {}
    for kk in GainsVec.keys():
        Gains[kk] = GainsVec[kk][nv_trim]
    
    AltController = gen_AltController(Gains)
    SpeedController = gen_SpeedController(Gains)
    TotalCost = GeneralFunction(InpGains = OptGains['x'], AircraftList = SaveAircraftPitch_Speed[nv_trim])
        
    # ClosedLoops = gen_ClosedLoops(SaveAircraftPitch_Speed[nv_trim] , AltController, SpeedController, EngActuator, Sensor_vx , Sensor_vz , Sensor_z, Sensor_ax, Sensor_az)
    # Criteria    = CalculateIndividualCosts(ClosedLoops)
    # TotalCost   = CalculateTotalCost(Criteria)
    CostVec[nv_trim] = TotalCost

    print('Final Cost: %0.2f' %(TotalCost))
    
    print_gains = ''
    for kk in GainsVec.keys():
        print_gains = print_gains + kk + ': ' + \
            format(GainsVec[kk][nv_trim], '0.4f') + ';  '
    print(print_gains)


        # AltController = gen_AltController(Gains)
        # SpeedController = gen_SpeedController(Gains)
    
        # ClosedLoops = gen_ClosedLoops(SaveAircraftPitch_Speed[nv_trim][0] , AltController, SpeedController, EngActuator, Sensor_vx , Sensor_vz , Sensor_z, Sensor_ax, Sensor_az)
        # Criteria    = CalculateIndividualCosts(ClosedLoops)
        # TotalCost   = CalculateTotalCost(Criteria)


        #     ClosedLoops = gen_ClosedLoops(SaveAircraftPitch[nv_trim , nt_trim] , AltController, SpeedController, EngActuator, Sensor_vx , Sensor_vz , Sensor_z, Sensor_ax, Sensor_az)
        #     Criteria    = CalculateIndividualCosts(ClosedLoops)
        #     TotalCost   = CalculateTotalCost(Criteria)

# %% PLOT RESULTS
plt.close('all')
n_save = -1
if len(TrimVec['AX_mps2']) ==5:
    line_type_vec = ['--' ,'-.' , '-' ,'-.' ,  '--']
elif len(TrimVec['AX_mps2']) == 3:
    line_type_vec = ['--' ,'-' , '--']
elif len(TrimVec['AX_mps2']) == 1:
    line_type_vec = ['-']
else:
    raise Exception('Adjust line_type_vec')

for nv_trim in range(len(TrimVec['VX_mps'])):
    for nt in range(len(TrimVec['AX_mps2'])):
        nt_trim = nt

        TestTilt_deg = TrimVec['TestTilt_deg'][nv_trim, nt_trim]
        n_save += 1
        if not(SaveAircraftPitch[nv_trim , nt_trim]):
            print("Speed: %0.1f mps / AX: %0.1f mps2 / Tilt: %02.1f --> Not valid" %
                  (TrimVec['VX_mps'][nv_trim], TrimVec['AX_mps2'][nt_trim], TestTilt_deg))
        else:
            Gains = {}
            for kk in GainsVec.keys():
                Gains[kk] = GainsVec[kk][nv_trim]

            AltController = gen_AltController(Gains)
            SpeedController = gen_SpeedController(Gains)
            ClosedLoops = gen_ClosedLoops(SaveAircraftPitch[nv_trim , nt_trim] , AltController, SpeedController, EngActuator, Sensor_vx , Sensor_vz , Sensor_z, Sensor_ax, Sensor_az)
            Criteria    = CalculateIndividualCosts(ClosedLoops)
            TotalCost   = CalculateTotalCost(Criteria)

            print("Speed: %0.1f mps / AX: %0.1f mps2 / Tilt: %02.1f --> TotalCost: %0.2f " %
                  (TrimVec['VX_mps'][nv_trim], TrimVec['AX_mps2'][nt_trim], TestTilt_deg, TotalCost))
            color_u1 = (TrimVec['VX_mps'][nv_trim] - np.min(TrimVec['VX_mps'])) / (np.max(TrimVec['VX_mps']) - np.min(TrimVec['VX_mps']))
            color_u2 = 0#np.abs(TrimVec['AX_mps2'][nt_trim]) / np.max(np.abs(TrimVec['AX_mps2']))
            
            if ((nv_trim == len(TrimVec['VX_mps'])-1) and (nt == len(TrimVec['AX_mps2'])-1)):
                plot_criteria = True
            else:
                plot_criteria = False
            # color_u1 = 0
            # color_u2 = 1
            gen_Plots(ClosedLoops, Criteria, ('%0.1f m/s / Tilt %0.1f' %(TrimVec['VX_mps'][nv_trim],TestTilt_deg)), color_rgb = ((1-color_u1),color_u2,color_u1), line_type = line_type_vec[nt], plot_criteria = plot_criteria)
    
# %% PLOT GAINS

fig = plt.figure('Gains')

keys = GainsVec.keys()
l = int(np.ceil(np.sqrt(1+len(keys))))
c = int(np.ceil(len(keys) / l))

for i in range(len(keys)-1):
    plt.subplot(l,c,i+1)
    plt.plot(TrimVec['VX_mps'], GainsVec[list(keys)[i]][:], 'o-')

    # plt.legend()
    plt.grid('on')
    plt.ylabel(list(keys)[i])
    # plt.ylim(bnds[i])
    plt.xlabel('VX [mps]')
    
plt.subplot(l,c,i+2)
plt.plot(TrimVec['VX_mps'] , CostVec)
plt.grid('on')
plt.ylabel('Cost')
plt.xlabel('VX [mps]')
 
fig.set_size_inches(14, 8)
fig.tight_layout(h_pad = 0.1 , w_pad = 0.02)
fig.savefig('DesignGains.png', bbox_inches='tight')
plt.show()

File2Save = {'GainsVec': GainsVec, 'TrimVec': TrimVec, 'CostVec': CostVec, 'SaveTrim': SaveTrim, 'NoteToSave': NoteToSave}

now = datetime.today()
save_name = 'SavedGains_VertFwdController_' + '{:04.0f}'.format(now.year) + '{:02.0f}'.format(now.month) + '{:02.0f}'.format(now.day) + '_' + '{:02.0f}'.format(now.hour) + '{:02.0f}'.format(now.minute) + '_' + opt_type +'.pkl'
with open(save_name, 'wb') as fp:
    pickle.dump(File2Save, fp)

# %% PLOT DESIGN POINTS
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
    plt.plot(TrimVec['VX_mps'] , TrimRes['W1_Tilt_deg'][:,n_t], 'o-', color = color, linewidth = 2, label = 'AX: %0.1f m/s'%TrimVec['AX_mps2'][n_t])
    plt.ylim([0, 90])
    plt.yticks(np.arange(0,120,30))
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Wing Tilt [deg]')
    plt.legend()

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.ylim([-1, +1])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Throttle Action')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['PitchThrottle'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.ylim([-0.10, 0.05])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('PitchThrottle [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Elevon3'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.ylim([-15, 15])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Elevon [deg]')
    
    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_1'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.ylim([0, 1250])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Thrust - Front Engines [N]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Thrust_5'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.ylim([0, 1250])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Thrust - Back Engines [N]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_1'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.ylim([0, 3000])
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('RPM - Front Engines')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['RPM_5'][:,n_t], 'o-', color = color, linewidth = 2)
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
    plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle1_p'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Throttle - Front Engines [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['Throttle5_p'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Throttle - Back Engines [p]')

    plt.subplot(plt_l,plt_c,plt_n); plt_n+=1
    plt.grid('on')
    plt.xlim([np.min(TrimVec['VX_mps']),np.max(TrimVec['VX_mps'])])
    plt.plot(TrimVec['VX_mps'] , TrimRes['AX_mps2'][:,n_t], 'o-', color = color, linewidth = 2)
    plt.xlabel('Inertial X Speed [m/s]')
    plt.ylabel('Inertial X Acceleration [m/s2]')



# fig.set_size_inches(8, 5)
fig.set_size_inches(14, 8)
fig.tight_layout(h_pad = 0.1 , w_pad = 0.02)
fig.savefig('DesignPoints.png', bbox_inches='tight')

 