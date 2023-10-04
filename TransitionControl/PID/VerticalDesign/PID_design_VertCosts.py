import numpy as np
import control as ct

def CalculateTotalCost(Criteria):
    TotalCost = 0
    for i in range(len(Criteria.keys())):
        CriteriaCost = Criteria[list(Criteria.keys())[i]]['cost'] * Criteria[list(Criteria.keys())[i]]['weight']
        # print("%s Cost: %0.2f" %(list(Criteria.keys())[i] , CriteriaCost))
        TotalCost += CriteriaCost
    
    return TotalCost

def CalculateCost (Criteria_dict):
    if Criteria_dict['type'] == 'prop':
        cost = ((np.max([Criteria_dict['res']/Criteria_dict['target'] ,1]) - 1) * Criteria_dict['weight_over'] + 
               -(np.min([Criteria_dict['res']/Criteria_dict['target'] ,1]) - 1) * Criteria_dict['weight_down'])
    elif Criteria_dict['type'] == 'abs':
        cost = (np.max([Criteria_dict['res']-Criteria_dict['target'],0]) *  Criteria_dict['weight_over'] + 
                np.min([Criteria_dict['res']-Criteria_dict['target'],0]) * -Criteria_dict['weight_down'])
    else:
        cost = 0
        
    return cost

def CalculateIndividualCosts(ClosedLoops):
    # %% CALCULATE CRITERIS
    #Damp
    #Phase / Gain Margin
    #Rise time
    #steady error
    
    Criteria = {}
    
    # VZ overshoot
    Criteria['vz_overshoot'] = {}
    Criteria['vz_overshoot']['weight_over'] = 1
    Criteria['vz_overshoot']['weight_down'] = 0
    Criteria['vz_overshoot']['weight'] = 1
    Criteria['vz_overshoot']['target'] = 0.2
    
    T, yout = ct.step_response(ClosedLoops['NoZFeedback'] , T=5, input = ClosedLoops['NoZFeedback'].input_labels.index('VZ_ref_mps'))
    max_q = np.max(yout[ClosedLoops['NoZFeedback'].output_labels.index('VZ_mps')][0])
    Criteria['vz_overshoot']['res'] = (max_q - 1)
    Criteria['vz_overshoot']['cost'] = (np.max([Criteria['vz_overshoot']['res'] - Criteria['vz_overshoot']['target'],0]) * Criteria['vz_overshoot']['weight_over'] + 
                                       np.min([Criteria['vz_overshoot']['res'] - Criteria['vz_overshoot']['target'],0]) * -Criteria['vz_overshoot']['weight_down'])

    # Z overshoot
    Criteria['z_overshoot'] = {}
    Criteria['z_overshoot']['weight_over'] = 1
    Criteria['z_overshoot']['weight_down'] = 0
    Criteria['z_overshoot']['weight'] = 1
    Criteria['z_overshoot']['target'] = 0.2
    
    T, yout = ct.step_response(ClosedLoops['PitchNotIncluded'] , T=5, input = ClosedLoops['PitchNotIncluded'].input_labels.index('Z_ref_m'))
    max_Z = np.max(yout[ClosedLoops['PitchNotIncluded'].output_labels.index('Z_m')][0])
    Criteria['z_overshoot']['res'] = (max_Z - 1)
    Criteria['z_overshoot']['cost'] = (np.max([Criteria['z_overshoot']['res'] - Criteria['z_overshoot']['target'],0]) * Criteria['z_overshoot']['weight_over'] + 
                                           np.min([Criteria['z_overshoot']['res'] - Criteria['z_overshoot']['target'],0]) * -Criteria['z_overshoot']['weight_down'])
    
    # Throttle Usage
    Criteria['max_throttle'] = {}
    Criteria['max_throttle']['weight_over'] = 1
    Criteria['max_throttle']['weight_down'] = 0
    Criteria['max_throttle']['weight'] = 1
    Criteria['max_throttle']['target'] = 0.1
    Criteria['max_throttle']['type'] = 'prop'

    T, yout = ct.step_response(ClosedLoops['PitchNotIncluded'] , T=5, input = ClosedLoops['PitchNotIncluded'].input_labels.index('Z_ref_m'))
    max_Thr = np.max(np.abs(yout[ClosedLoops['PitchNotIncluded'].output_labels.index('ThrottleCmd_u')][0]))
    Criteria['max_throttle']['res'] = max_Thr
    Criteria['max_throttle']['cost'] = CalculateCost (Criteria['max_throttle'])

    # Z Rise
    Criteria['z_risetime80'] = {}
    Criteria['z_risetime80']['weight_over'] = 1
    Criteria['z_risetime80']['weight_down'] = -0.1
    Criteria['z_risetime80']['weight'] = 5
    Criteria['z_risetime80']['target'] = 2.0
    
    T, yout = ct.step_response(ClosedLoops['PitchNotIncluded'] , T=10, input = ClosedLoops['PitchNotIncluded'].input_labels.index('Z_ref_m'))
    over_riseLim = T[yout[ClosedLoops['PitchNotIncluded'].output_labels.index('Z_m')][0] > 0.8]
    if len(over_riseLim) == 0:
        z_rt = 10
    else:
        z_rt = T[yout[ClosedLoops['PitchNotIncluded'].output_labels.index('Z_m')][0] > 0.8][0]
    Criteria['z_risetime80']['res'] = z_rt
    Criteria['z_risetime80']['cost'] = (np.max([z_rt - Criteria['z_risetime80']['target'],0]) * Criteria['z_risetime80']['weight_over'] + 
                                            np.min([z_rt - Criteria['z_risetime80']['target'],0]) * -Criteria['z_risetime80']['weight_down'])
    
    
    # Z Steady Error
    Criteria['z_steadystate'] = {}
    Criteria['z_steadystate']['weight_over'] = 1
    Criteria['z_steadystate']['weight_down'] = 1
    Criteria['z_steadystate']['weight'] = 10
    Criteria['z_steadystate']['target'] = 1.0
    
    dcgain = ct.dcgain(ClosedLoops['Z2Z'])
    if np.isnan(dcgain):
        T,yout = ct.step_response(ClosedLoops['Z2Z'],10)
        dcgain = yout[-1]
        
    Criteria['z_steadystate']['res'] = dcgain
        
    Criteria['z_steadystate']['cost'] = (np.max([Criteria['z_steadystate']['res'] - Criteria['z_steadystate']['target'],0]) * Criteria['z_steadystate']['weight_over'] + 
                                           np.min([Criteria['z_steadystate']['res'] - Criteria['z_steadystate']['target'],0]) * -Criteria['z_steadystate']['weight_down'])
    
    # VZ Steady Error
    Criteria['vz_steadystate'] = {}
    Criteria['vz_steadystate']['weight_over'] = 1
    Criteria['vz_steadystate']['weight_down'] = 1
    Criteria['vz_steadystate']['weight'] = 10
    Criteria['vz_steadystate']['target'] = 1.0
    

    dcgain = ct.dcgain(ClosedLoops['VZ2VZ'])
    if np.isnan(dcgain):
        T,yout = ct.step_response(ClosedLoops['VZ2VZ'],10)
        dcgain = yout[-1]
        
    Criteria['vz_steadystate']['res'] = dcgain
        
    Criteria['vz_steadystate']['cost'] = (np.max([Criteria['vz_steadystate']['res'] - Criteria['vz_steadystate']['target'],0]) * Criteria['vz_steadystate']['weight_over'] + 
                                           np.min([Criteria['vz_steadystate']['res'] - Criteria['vz_steadystate']['target'],0]) * -Criteria['vz_steadystate']['weight_down'])
    
    # %% CLOSED LOOP STABILITY
    Criteria['closedloop_stability'] = {}
    Criteria['closedloop_stability']['weight_over'] = 1
    Criteria['closedloop_stability']['weight_down'] = 0
    Criteria['closedloop_stability']['weight'] = 100
    Criteria['closedloop_stability']['target'] = 0.0
    Criteria['closedloop_stability']['type']   = 'abs'
   
    CL_poles = ct.pole(ClosedLoops['PitchNotIncluded'])
    CL_real = np.real(CL_poles)
    CL_imag = np.imag(CL_poles)
    CL_w    = (CL_real**2 + CL_imag**2)**0.5
    
    inst = np.max(CL_real * CL_w)
    
    if inst > 0.0:
        Criteria['closedloop_stability']['res'] = np.min([10.0, np.max([5.0 , inst])])
    else:
         Criteria['closedloop_stability']['res'] = 0.0
         
    Criteria['closedloop_stability']['cost'] = CalculateCost(Criteria['closedloop_stability'])
    
    # # %% Damping
    # # Q Damping
    # Criteria['vz_damp'] = {}
    # Criteria['vz_damp']['weight_over'] = 0
    # Criteria['vz_damp']['weight_down'] = 1
    # Criteria['vz_damp']['weight'] = 5*0
    # Criteria['vz_damp']['target'] = 0.5
    
    # aux = ct.damp(ClosedLoops['Only_VZ'] , doprint=False)
    # Criteria['vz_damp']['res'] = min(aux[1])
        
    # Criteria['vz_damp']['cost'] = (np.max([Criteria['vz_damp']['res'] - Criteria['vz_damp']['target'],0]) * Criteria['vz_damp']['weight_over'] + 
    #                                        np.min([Criteria['vz_damp']['res'] - Criteria['vz_damp']['target'],0]) * -Criteria['vz_damp']['weight_down'])
    
    
    
    # %% Phase/Gain Margin - OpenLoop ThrottleCmd
    Criteria['openloop_throttlecmd_gainmargin'] = {}
    Criteria['openloop_throttlecmd_gainmargin']['weight_over'] = 0
    Criteria['openloop_throttlecmd_gainmargin']['weight_down'] = 1
    Criteria['openloop_throttlecmd_gainmargin']['weight']      = 10
    Criteria['openloop_throttlecmd_gainmargin']['target']      = 6.0
    Criteria['openloop_throttlecmd_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_throttlecmd_phasemargin'] = {}
    Criteria['openloop_throttlecmd_phasemargin']['weight_over'] = 0
    Criteria['openloop_throttlecmd_phasemargin']['weight_down'] = 1
    Criteria['openloop_throttlecmd_phasemargin']['weight']      = 10
    Criteria['openloop_throttlecmd_phasemargin']['target']      = 45.0
    Criteria['openloop_throttlecmd_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_throttlecmd'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_throttlecmd_gainmargin']['res'] = GM_dB
    Criteria['openloop_throttlecmd_gainmargin']['cost'] = CalculateCost(Criteria['openloop_throttlecmd_gainmargin'])
    
    Criteria['openloop_throttlecmd_phasemargin']['res'] = PM_deg
    Criteria['openloop_throttlecmd_phasemargin']['cost'] = CalculateCost(Criteria['openloop_throttlecmd_phasemargin'])
    
    # %% Phase/Gain Margin - OpenLoop VZ
    Criteria['openloop_VZ_gainmargin'] = {}
    Criteria['openloop_VZ_gainmargin']['weight_over'] = 0
    Criteria['openloop_VZ_gainmargin']['weight_down'] = 1
    Criteria['openloop_VZ_gainmargin']['weight']      = 10
    Criteria['openloop_VZ_gainmargin']['target']      = 6.0
    Criteria['openloop_VZ_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_VZ_phasemargin'] = {}
    Criteria['openloop_VZ_phasemargin']['weight_over'] = 0
    Criteria['openloop_VZ_phasemargin']['weight_down'] = 1
    Criteria['openloop_VZ_phasemargin']['weight']      = 10
    Criteria['openloop_VZ_phasemargin']['target']      = 45.0
    Criteria['openloop_VZ_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_VZ'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_VZ_gainmargin']['res'] = GM_dB
    Criteria['openloop_VZ_gainmargin']['cost'] = CalculateCost(Criteria['openloop_VZ_gainmargin'])
    
    Criteria['openloop_VZ_phasemargin']['res'] = PM_deg
    Criteria['openloop_VZ_phasemargin']['cost'] = CalculateCost(Criteria['openloop_VZ_phasemargin'])
    
    # %% Phase/Gain Margin - OpenLoop Z
    Criteria['openloop_Z_gainmargin'] = {}
    Criteria['openloop_Z_gainmargin']['weight_over'] = 0
    Criteria['openloop_Z_gainmargin']['weight_down'] = 1
    Criteria['openloop_Z_gainmargin']['weight']      = 10
    Criteria['openloop_Z_gainmargin']['target']      = 6.0
    Criteria['openloop_Z_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_Z_phasemargin'] = {}
    Criteria['openloop_Z_phasemargin']['weight_over'] = 0
    Criteria['openloop_Z_phasemargin']['weight_down'] = 1
    Criteria['openloop_Z_phasemargin']['weight']      = 10
    Criteria['openloop_Z_phasemargin']['target']      = 45.0
    Criteria['openloop_Z_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_Z'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_Z_gainmargin']['res'] = GM_dB
    Criteria['openloop_Z_gainmargin']['cost'] = CalculateCost(Criteria['openloop_Z_gainmargin'])
    
    Criteria['openloop_Z_phasemargin']['res'] = PM_deg
    Criteria['openloop_Z_phasemargin']['cost'] = CalculateCost(Criteria['openloop_Z_phasemargin'])
    return Criteria
