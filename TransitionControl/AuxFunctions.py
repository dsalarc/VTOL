#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 20:28:58 2023

@author: dsalarc
"""
import numpy as np


def AppendValue(SaveVec,name,Value,):
    if name in SaveVec:
        SaveVec[name] = np.append(SaveVec[name],Value)
    else:
        SaveVec[name] = np.array([Value])
         
    return SaveVec

def SaveSelection(SaveVec,info):
    SaveVec = AppendValue(SaveVec,'X_m',info['EQM']['PosLin_EarthAx_m'][0])
    SaveVec = AppendValue(SaveVec,'Y_m',info['EQM']['PosLin_EarthAx_m'][1])
    SaveVec = AppendValue(SaveVec,'Z_m',info['EQM']['PosLin_EarthAx_m'][2])
    SaveVec = AppendValue(SaveVec,'H_m',-info['EQM']['PosLin_EarthAx_m'][2])
    SaveVec = AppendValue(SaveVec,'Altitude_m',info['ATM']['Altitude_m'])

    SaveVec = AppendValue(SaveVec,'U_mps',info['EQM']['VelLin_BodyAx_mps'][0])
    SaveVec = AppendValue(SaveVec,'V_mps',info['EQM']['VelLin_BodyAx_mps'][1])
    SaveVec = AppendValue(SaveVec,'W_mps',info['EQM']['VelLin_BodyAx_mps'][2])
    
    SaveVec = AppendValue(SaveVec,'dU_mps2',info['EQM']['AccLin_BodyAx_mps2'][0])
    SaveVec = AppendValue(SaveVec,'dV_mps2',info['EQM']['AccLin_BodyAx_mps2'][1])
    SaveVec = AppendValue(SaveVec,'dW_mps2',info['EQM']['AccLin_BodyAx_mps2'][2])
    
    SaveVec = AppendValue(SaveVec,'VX_mps',info['EQM']['VelLin_EarthAx_mps'][0])
    SaveVec = AppendValue(SaveVec,'VY_mps',info['EQM']['VelLin_EarthAx_mps'][1])
    SaveVec = AppendValue(SaveVec,'VZ_mps',info['EQM']['VelLin_EarthAx_mps'][2])
    
    SaveVec = AppendValue(SaveVec,'dVX_mps2',info['EQM']['AccLin_EarthAx_mps2'][0])
    SaveVec = AppendValue(SaveVec,'dVY_mps2',info['EQM']['AccLin_EarthAx_mps2'][1])
    SaveVec = AppendValue(SaveVec,'dVZ_mps2',info['EQM']['AccLin_EarthAx_mps2'][2])
    
    SaveVec = AppendValue(SaveVec,'NX_mps2',info['EQM']['LoadFactor_mps2'][0])
    SaveVec = AppendValue(SaveVec,'NY_mps2',info['EQM']['LoadFactor_mps2'][1])
    SaveVec = AppendValue(SaveVec,'NZ_mps2',info['EQM']['LoadFactor_mps2'][2])
    
    SaveVec = AppendValue(SaveVec,'Phi_deg',np.rad2deg(info['EQM']['EulerAngles_rad'][0]))
    SaveVec = AppendValue(SaveVec,'Theta_deg',np.rad2deg(info['EQM']['EulerAngles_rad'][1]))
    SaveVec = AppendValue(SaveVec,'Psi_deg',np.rad2deg(info['EQM']['EulerAngles_rad'][2]))
    
    SaveVec = AppendValue(SaveVec,'P_degps',np.rad2deg(info['EQM']['VelRot_BodyAx_radps'][0]))
    SaveVec = AppendValue(SaveVec,'Q_degps',np.rad2deg(info['EQM']['VelRot_BodyAx_radps'][1]))
    SaveVec = AppendValue(SaveVec,'R_degps',np.rad2deg(info['EQM']['VelRot_BodyAx_radps'][2]))
    
    SaveVec = AppendValue(SaveVec,'Pdot_radps2',np.rad2deg(info['EQM']['AccRot_BodyAx_radps2'][0]))
    SaveVec = AppendValue(SaveVec,'Qdot_radps2',np.rad2deg(info['EQM']['AccRot_BodyAx_radps2'][1]))
    SaveVec = AppendValue(SaveVec,'Rdot_radps2',np.rad2deg(info['EQM']['AccRot_BodyAx_radps2'][2]))
 
    SaveVec = AppendValue(SaveVec,'FX_N',info['EQM']['TotalForce'][0])
    SaveVec = AppendValue(SaveVec,'FY_N',info['EQM']['TotalForce'][1])
    SaveVec = AppendValue(SaveVec,'FZ_N',info['EQM']['TotalForce'][2])
 
    SaveVec = AppendValue(SaveVec,'MX_Nm',info['EQM']['TotalMoment'][0])
    SaveVec = AppendValue(SaveVec,'MY_Nm',info['EQM']['TotalMoment'][1])
    SaveVec = AppendValue(SaveVec,'MZ_Nm',info['EQM']['TotalMoment'][2])
 
    SaveVec = AppendValue(SaveVec,'MXaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][0])
    SaveVec = AppendValue(SaveVec,'MYaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][1])
    SaveVec = AppendValue(SaveVec,'MZaero_Nm',info['AERO']['TotalMoment_BodyAx_Nm'][2])
 
    SaveVec = AppendValue(SaveVec,'FXaero_N',info['AERO']['TotalForce_BodyAx_N'][0])
    SaveVec = AppendValue(SaveVec,'FYaero_N',info['AERO']['TotalForce_BodyAx_N'][1])
    SaveVec = AppendValue(SaveVec,'FZaero_N',info['AERO']['TotalForce_BodyAx_N'][2])
    
    SaveVec = AppendValue(SaveVec,'Alpha_deg',info['ATM']['Alpha_deg'])
    SaveVec = AppendValue(SaveVec,'Beta_deg',info['ATM']['Beta_deg'])
    SaveVec = AppendValue(SaveVec,'DynPres_Pa',info['ATM']['DynPres_Pa'])
    SaveVec = AppendValue(SaveVec,'CAS_mps',info['ATM']['CAS_mps'])
    SaveVec = AppendValue(SaveVec,'TowerWindLocalX_mps',info['ATM']['TowerWindLocalX_mps'])
    SaveVec = AppendValue(SaveVec,'TowerWindLocalY_mps',info['ATM']['TowerWindLocalY_mps'])
    SaveVec = AppendValue(SaveVec,'WindX_mps',info['ATM']['WindVec_mps'][0])
    SaveVec = AppendValue(SaveVec,'WindY_mps',info['ATM']['WindVec_mps'][1])
    SaveVec = AppendValue(SaveVec,'WindZ_mps',info['ATM']['WindVec_mps'][2])
    SaveVec = AppendValue(SaveVec,'WindX_W1_mps',info['ATM']['WindVec_W1_mps'][0])
    SaveVec = AppendValue(SaveVec,'WindY_W1_mps',info['ATM']['WindVec_W1_mps'][1])
    SaveVec = AppendValue(SaveVec,'WindZ_W1_mps',info['ATM']['WindVec_W1_mps'][2])
    SaveVec = AppendValue(SaveVec,'WindX_W2_mps',info['ATM']['WindVec_W2_mps'][0])
    SaveVec = AppendValue(SaveVec,'WindY_W2_mps',info['ATM']['WindVec_W2_mps'][1])
    SaveVec = AppendValue(SaveVec,'WindZ_W2_mps',info['ATM']['WindVec_W2_mps'][2])

    SaveVec = AppendValue(SaveVec,'W1_Alpha_deg',info['AERO']['Wing1']['Alpha_deg'])
    SaveVec = AppendValue(SaveVec,'W2_Alpha_deg',info['AERO']['Wing2']['Alpha_deg'])
    SaveVec = AppendValue(SaveVec,'W1_Incidence_deg',info['AERO']['Wing1']['Incidence_deg'])
    SaveVec = AppendValue(SaveVec,'W2_Incidence_deg',info['AERO']['Wing2']['Incidence_deg'])
    SaveVec = AppendValue(SaveVec,'W1_CLS',info['AERO']['Wing1']['CLS_25Local'])
    SaveVec = AppendValue(SaveVec,'W2_CLS',info['AERO']['Wing2']['CLS_25Local'])
    SaveVec = AppendValue(SaveVec,'W1_CDS',info['AERO']['Wing1']['CDS_25Local'])
    SaveVec = AppendValue(SaveVec,'W2_CDS',info['AERO']['Wing2']['CDS_25Local'])

    SaveVec = AppendValue(SaveVec,'AERO_W1_CLS_25Local',info['AERO']['Wing1']['CLS_25Local'])
    SaveVec = AppendValue(SaveVec,'AERO_W2_CLS_25Local',info['AERO']['Wing2']['CLS_25Local'])
    SaveVec = AppendValue(SaveVec,'AERO_W1_CDS_25Local',info['AERO']['Wing1']['CDS_25Local'])
    SaveVec = AppendValue(SaveVec,'AERO_W2_CDS_25Local',info['AERO']['Wing2']['CDS_25Local'])
    SaveVec = AppendValue(SaveVec,'AERO_W1_CMS_25Local',info['AERO']['Wing1']['CMS_25Local'])
    SaveVec = AppendValue(SaveVec,'AERO_W2_CMS_25Local',info['AERO']['Wing2']['CMS_25Local'])
    SaveVec = AppendValue(SaveVec,'AERO_W1_CLB_CG',info['AERO']['Wing1']['CLB_CG'])
    SaveVec = AppendValue(SaveVec,'AERO_W2_CLB_CG',info['AERO']['Wing2']['CLB_CG'])
    SaveVec = AppendValue(SaveVec,'AERO_W1_CDB_CG',info['AERO']['Wing1']['CDB_CG'])
    SaveVec = AppendValue(SaveVec,'AERO_W2_CDB_CG',info['AERO']['Wing2']['CDB_CG'])
    SaveVec = AppendValue(SaveVec,'AERO_W1_CMB_CG',info['AERO']['Wing1']['CMB_CG'])
    SaveVec = AppendValue(SaveVec,'AERO_W2_CMB_CG',info['AERO']['Wing2']['CMB_CG'])



    try:
        SaveVec = AppendValue(SaveVec,'Elevon1',info['AERO']['Elevon']['Deflection_deg'][0])
        SaveVec = AppendValue(SaveVec,'Elevon2',info['AERO']['Elevon']['Deflection_deg'][1])
        SaveVec = AppendValue(SaveVec,'Elevon3',info['AERO']['Elevon']['Deflection_deg'][2])
        SaveVec = AppendValue(SaveVec,'Elevon4',info['AERO']['Elevon']['Deflection_deg'][3])
    except:
        SaveVec = AppendValue(SaveVec,'Elevon1',info['CONT']['Elevon_deg'][0])
        SaveVec = AppendValue(SaveVec,'Elevon2',info['CONT']['Elevon_deg'][1])
        SaveVec = AppendValue(SaveVec,'Elevon3',info['CONT']['Elevon_deg'][2])
        SaveVec = AppendValue(SaveVec,'Elevon4',info['CONT']['Elevon_deg'][3])
           

    SaveVec = AppendValue(SaveVec,'RPM_1',info['MOT']['RPM'][0])
    SaveVec = AppendValue(SaveVec,'RPM_2',info['MOT']['RPM'][1])
    SaveVec = AppendValue(SaveVec,'RPM_4',info['MOT']['RPM'][3])
    SaveVec = AppendValue(SaveVec,'RPM_5',info['MOT']['RPM'][4])
    SaveVec = AppendValue(SaveVec,'RPM_8',info['MOT']['RPM'][7])

    SaveVec = AppendValue(SaveVec,'Thrust1_N',info['MOT']['Thrust_N'][0])
    SaveVec = AppendValue(SaveVec,'Thrust2_N',info['MOT']['Thrust_N'][1])
    SaveVec = AppendValue(SaveVec,'Thrust3_N',info['MOT']['Thrust_N'][2])
    SaveVec = AppendValue(SaveVec,'Thrust4_N',info['MOT']['Thrust_N'][3])
    SaveVec = AppendValue(SaveVec,'Thrust5_N',info['MOT']['Thrust_N'][4])
    SaveVec = AppendValue(SaveVec,'Thrust6_N',info['MOT']['Thrust_N'][5])
    SaveVec = AppendValue(SaveVec,'Thrust7_N',info['MOT']['Thrust_N'][6])
    SaveVec = AppendValue(SaveVec,'Thrust8_N',info['MOT']['Thrust_N'][7])

    SaveVec = AppendValue(SaveVec,'Voltage1_V',info['MOT']['ASSEMBLY']['obj'][0].V_V)
    SaveVec = AppendValue(SaveVec,'Voltage4_V',info['MOT']['ASSEMBLY']['obj'][3].V_V)
    SaveVec = AppendValue(SaveVec,'Voltage5_V',info['MOT']['ASSEMBLY']['obj'][4].V_V)
    SaveVec = AppendValue(SaveVec,'Voltage8_V',info['MOT']['ASSEMBLY']['obj'][7].V_V)

    SaveVec = AppendValue(SaveVec,'Throttle1_u',info['MOT']['ASSEMBLY']['obj'][0].Throttle)
    SaveVec = AppendValue(SaveVec,'Throttle4_u',info['MOT']['ASSEMBLY']['obj'][3].Throttle)
    SaveVec = AppendValue(SaveVec,'Throttle5_u',info['MOT']['ASSEMBLY']['obj'][4].Throttle)
    SaveVec = AppendValue(SaveVec,'Throttle8_u',info['MOT']['ASSEMBLY']['obj'][7].Throttle)


    SaveVec = AppendValue(SaveVec,'SENS_P_radps',info['SENS']['P_radps']  )
    SaveVec = AppendValue(SaveVec,'SENS_Q_radps',info['SENS']['Q_radps']  )
    SaveVec = AppendValue(SaveVec,'SENS_R_radps',info['SENS']['R_radps']  )
    SaveVec = AppendValue(SaveVec,'SENS_Phi_rad',info['SENS']['Phi_rad']  )
    SaveVec = AppendValue(SaveVec,'SENS_Theta_rad',info['SENS']['Theta_rad'])
    SaveVec = AppendValue(SaveVec,'SENS_Psi_rad',info['SENS']['Psi_rad']  )
    SaveVec = AppendValue(SaveVec,'SENS_VX_mps',info['SENS']['VX_mps']   )
    SaveVec = AppendValue(SaveVec,'SENS_VY_mps',info['SENS']['VY_mps']   )
    SaveVec = AppendValue(SaveVec,'SENS_VZ_mps',info['SENS']['VZ_mps']   )
    SaveVec = AppendValue(SaveVec,'SENS_NX_mps2',info['SENS']['NX_mps2']     )
    SaveVec = AppendValue(SaveVec,'SENS_NY_mps2',info['SENS']['NY_mps2']     )
    SaveVec = AppendValue(SaveVec,'SENS_NZ_mps2',info['SENS']['NZ_mps2']     )
    SaveVec = AppendValue(SaveVec,'SENS_CAS_mps',info['SENS']['CAS_mps']  )

    try:
        SaveVec = AppendValue(SaveVec,'Throttle1_N',info['MOT']['ASSEMBLY']['obj'][0].Throttle)
        SaveVec = AppendValue(SaveVec,'Throttle5_N',info['MOT']['ASSEMBLY']['obj'][4].Throttle)
    
        SaveVec = AppendValue(SaveVec,'J1',info['MOT']['ASSEMBLY']['obj'][0].PROPELLER.J)
        SaveVec = AppendValue(SaveVec,'Charge1_Ah',info['MOT']['ASSEMBLY']['obj'][0].MOTOR.TotalCharge_Ah)
        SaveVec = AppendValue(SaveVec,'Charge5_Ah',info['MOT']['ASSEMBLY']['obj'][4].MOTOR.TotalCharge_Ah)
        SaveVec = AppendValue(SaveVec,'Energy1_Wh',info['MOT']['ASSEMBLY']['obj'][0].MOTOR.TotalEnergy_Wh)
        SaveVec = AppendValue(SaveVec,'Energy5_Wh',info['MOT']['ASSEMBLY']['obj'][4].MOTOR.TotalEnergy_Wh)
        SaveVec = AppendValue(SaveVec,'Current1_A',info['MOT']['ASSEMBLY']['obj'][0].MOTOR.i_A)
        SaveVec = AppendValue(SaveVec,'Current5_A',info['MOT']['ASSEMBLY']['obj'][4].MOTOR.i_A)
        SaveVec = AppendValue(SaveVec,'Voltage1_v',info['MOT']['ASSEMBLY']['obj'][0].V_V)
        SaveVec = AppendValue(SaveVec,'Voltage5_v',info['MOT']['ASSEMBLY']['obj'][4].V_V)
        
        SaveVec = AppendValue(SaveVec,'act_Throttle',info['Action'][0])
        SaveVec = AppendValue(SaveVec,'act_PitchThrottle',info['Action'][1])
        SaveVec = AppendValue(SaveVec,'act_W1_Tilt',info['Action'][2])
        SaveVec = AppendValue(SaveVec,'act_W2_Tilt',info['Action'][3])
        SaveVec = AppendValue(SaveVec,'act_W2_Elevator',info['Action'][4])
    except:
        pass      
    SaveVec = AppendValue(SaveVec,'AvgCurrent1_A',info['MOT']['ASSEMBLY']['obj'][0].AvgCurrent_A)
    SaveVec = AppendValue(SaveVec,'AvgCurrent5_A',info['MOT']['ASSEMBLY']['obj'][4].AvgCurrent_A)

    SaveVec = AppendValue(SaveVec,'Weight_kgf',info['MASS']['Weight_kgf'])
    SaveVec = AppendValue(SaveVec,'XCG_m',info['MASS']['CG_m'][0])
    SaveVec = AppendValue(SaveVec,'YCG_m',info['MASS']['CG_m'][0])
    SaveVec = AppendValue(SaveVec,'ZCG_m',info['MASS']['CG_m'][0])
    
    SaveVec = AppendValue(SaveVec,'Reward',info['REW'])

    return SaveVec