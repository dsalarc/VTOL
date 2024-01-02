import numpy as np
import control as ct

def CalculateTotalCost(Criteria):
    TotalCost = 0
    for i in range(len(Criteria.keys())):
        Criteria[list(Criteria.keys())[i]]['cost_weighted'] = Criteria[list(Criteria.keys())[i]]['cost'] * Criteria[list(Criteria.keys())[i]]['weight']
        # print("%s Cost: %0.2f" %(list(Criteria.keys())[i] , CriteriaCost))
        TotalCost += Criteria[list(Criteria.keys())[i]]['cost_weighted']
        
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
    
    Criteria = {}
    
    # Q OVERSHOOT
    Criteria['q_overshoot'] = {}
    Criteria['q_overshoot']['weight_over'] = 1
    Criteria['q_overshoot']['weight_down'] = 0
    Criteria['q_overshoot']['weight'] = 10
    Criteria['q_overshoot']['target'] = 0.2
    
    T, yout = ct.step_response(ClosedLoops['NoThetaFeedback'] , T=5, input = ClosedLoops['NoThetaFeedback'].input_labels.index('Q_ref_degps'))
    max_q = np.max(yout[ClosedLoops['NoThetaFeedback'].output_labels.index('Q_degps')][0])
    Criteria['q_overshoot']['res'] = (max_q - 1)
    Criteria['q_overshoot']['cost'] = (np.max([Criteria['q_overshoot']['res'] - Criteria['q_overshoot']['target'],0]) * Criteria['q_overshoot']['weight_over'] + 
                                       np.min([Criteria['q_overshoot']['res'] - Criteria['q_overshoot']['target'],0]) * -Criteria['q_overshoot']['weight_down'])

    # THETA OVERSHOOT
    Criteria['theta_overshoot'] = {}
    Criteria['theta_overshoot']['weight_over'] = 1
    Criteria['theta_overshoot']['weight_down'] = 0
    Criteria['theta_overshoot']['weight'] = 10
    Criteria['theta_overshoot']['target'] = 0.2
    
    T, yout = ct.step_response(ClosedLoops['AltitudeIncluded'] , T=5, input = ClosedLoops['AltitudeIncluded'].input_labels.index('Theta_ref_deg'))
    max_T = np.max(yout[ClosedLoops['AltitudeIncluded'].output_labels.index('Theta_deg')][0])
    Criteria['theta_overshoot']['res'] = (max_T - 1)
    Criteria['theta_overshoot']['cost'] = (np.max([Criteria['theta_overshoot']['res'] - Criteria['theta_overshoot']['target'],0]) * Criteria['theta_overshoot']['weight_over'] + 
                                           np.min([Criteria['theta_overshoot']['res'] - Criteria['theta_overshoot']['target'],0]) * -Criteria['theta_overshoot']['weight_down'])
    
    # THETA RISE TIME
    Criteria['theta_risetime80'] = {}
    Criteria['theta_risetime80']['weight_over'] = 1
    Criteria['theta_risetime80']['weight_down'] = -0.1
    Criteria['theta_risetime80']['weight'] = 5
    Criteria['theta_risetime80']['target'] = 2.0
    
    T, yout = ct.step_response(ClosedLoops['AltitudeIncluded'] , T=10, input = ClosedLoops['AltitudeIncluded'].input_labels.index('Theta_ref_deg'))
    over_riseLim = T[yout[ClosedLoops['AltitudeIncluded'].output_labels.index('Theta_deg')][0] > 0.8]
    if len(over_riseLim) == 0:
        theta_rt = 10
    else:
        theta_rt = T[yout[ClosedLoops['AltitudeIncluded'].output_labels.index('Theta_deg')][0] > 0.8][0]
    Criteria['theta_risetime80']['res'] = theta_rt
    Criteria['theta_risetime80']['cost'] = (np.max([theta_rt - Criteria['theta_risetime80']['target'],0]) * Criteria['theta_risetime80']['weight_over'] + 
                                            np.min([theta_rt - Criteria['theta_risetime80']['target'],0]) * -Criteria['theta_risetime80']['weight_down'])
    
    
    # THETA STEADY ERROR
    Criteria['theta_steadystate'] = {}
    Criteria['theta_steadystate']['weight_over'] = 1
    Criteria['theta_steadystate']['weight_down'] = 1
    Criteria['theta_steadystate']['weight'] = 10
    Criteria['theta_steadystate']['target'] = 1.0
    
    dcgain = ct.dcgain(ClosedLoops['T2T'])
    if np.isnan(dcgain):
        T,yout = ct.step_response(ClosedLoops['T2T'],10)
        dcgain = yout[-1]
        
    Criteria['theta_steadystate']['res'] = dcgain
        
    Criteria['theta_steadystate']['cost'] = (np.max([Criteria['theta_steadystate']['res'] - Criteria['theta_steadystate']['target'],0]) * Criteria['theta_steadystate']['weight_over'] + 
                                           np.min([Criteria['theta_steadystate']['res'] - Criteria['theta_steadystate']['target'],0]) * -Criteria['theta_steadystate']['weight_down'])
    
    # Q STEADY ERROR
    Criteria['q_steadystate'] = {}
    Criteria['q_steadystate']['weight_over'] = 1
    Criteria['q_steadystate']['weight_down'] = 1
    Criteria['q_steadystate']['weight'] = 10
    Criteria['q_steadystate']['target'] = 1.0
    

    dcgain = ct.dcgain(ClosedLoops['Q2Q'])
    if np.isnan(dcgain):
        T,yout = ct.step_response(ClosedLoops['Q2Q'],10)
        dcgain = yout[-1]
        
    Criteria['q_steadystate']['res'] = dcgain
        
    Criteria['q_steadystate']['cost'] = (np.max([Criteria['q_steadystate']['res'] - Criteria['q_steadystate']['target'],0]) * Criteria['q_steadystate']['weight_over'] + 
                                           np.min([Criteria['q_steadystate']['res'] - Criteria['q_steadystate']['target'],0]) * -Criteria['q_steadystate']['weight_down'])
    
    # THROTTLE DISTURBANCE REJECTION
    Criteria['disturb_rejection'] = {}
    Criteria['disturb_rejection']['weight_over'] = 1
    Criteria['disturb_rejection']['weight_down'] = -0.1
    Criteria['disturb_rejection']['weight'] = 0.5
    Criteria['disturb_rejection']['target'] = 2.0
    Criteria['disturb_rejection']['type']   = 'abs'
    
    T, yout = ct.step_response(ClosedLoops['AltitudeIncluded'] , T=5, input = ClosedLoops['AltitudeIncluded'].input_labels.index('Throttle_inp'))

    accomod_time = T[np.abs(yout[ClosedLoops['AltitudeIncluded'].output_labels.index('Theta_deg')][0]) > 0.2]
    if len(accomod_time) == 0:
        theta_acc = 0
    else:
        theta_acc = accomod_time[-1]
    Criteria['disturb_rejection']['res'] = theta_acc
    Criteria['disturb_rejection']['cost'] = CalculateCost(Criteria['disturb_rejection'])
    
 
    # %% CLOSED LOOP STABILITY
    Criteria['closedloop_stability'] = {}
    Criteria['closedloop_stability']['weight_over'] = 1
    Criteria['closedloop_stability']['weight_down'] = 0
    Criteria['closedloop_stability']['weight'] = 100
    Criteria['closedloop_stability']['target'] = 0.0
    Criteria['closedloop_stability']['type']   = 'abs'
   
    CL_poles = ct.pole(ClosedLoops['AltitudeNotIncluded'])
    CL_real = np.real(CL_poles)
    CL_imag = np.imag(CL_poles)
    CL_w    = (CL_real**2 + CL_imag**2)**0.5
    
    inst = np.max(CL_real * CL_w)
    
    if inst > 0.0:
        Criteria['closedloop_stability']['res'] = np.min([10.0, np.max([5.0 , inst])])
    else:
         Criteria['closedloop_stability']['res'] = 0.0
         
    Criteria['closedloop_stability']['cost'] = CalculateCost(Criteria['closedloop_stability'])
   
    # %% Damping
    # Q Damping
    Criteria['q_damp'] = {}
    Criteria['q_damp']['weight_over'] = 0
    Criteria['q_damp']['weight_down'] = 1
    Criteria['q_damp']['weight'] = 5*0
    Criteria['q_damp']['target'] = 0.5
    
    aux = ct.damp(ClosedLoops['Q2Q'] , doprint=False)
    Criteria['q_damp']['res'] = min(aux[1])
        
    Criteria['q_damp']['cost'] = (np.max([Criteria['q_damp']['res'] - Criteria['q_damp']['target'],0]) * Criteria['q_damp']['weight_over'] + 
                                           np.min([Criteria['q_damp']['res'] - Criteria['q_damp']['target'],0]) * -Criteria['q_damp']['weight_down'])
    
    
    
    # %% Phase/Gain Margin - OpenLoop PitchCmd
    Criteria['openloop_pitchcmd_gainmargin'] = {}
    Criteria['openloop_pitchcmd_gainmargin']['weight_over'] = 0
    Criteria['openloop_pitchcmd_gainmargin']['weight_down'] = 1
    Criteria['openloop_pitchcmd_gainmargin']['weight']      = 10
    Criteria['openloop_pitchcmd_gainmargin']['target']      = 8.0
    Criteria['openloop_pitchcmd_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_pitchcmd_phasemargin'] = {}
    Criteria['openloop_pitchcmd_phasemargin']['weight_over'] = 0
    Criteria['openloop_pitchcmd_phasemargin']['weight_down'] = 1
    Criteria['openloop_pitchcmd_phasemargin']['weight']      = 10
    Criteria['openloop_pitchcmd_phasemargin']['target']      = 60.0
    Criteria['openloop_pitchcmd_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_pitchcmd'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_pitchcmd_gainmargin']['res'] = GM_dB
    Criteria['openloop_pitchcmd_gainmargin']['cost'] = CalculateCost(Criteria['openloop_pitchcmd_gainmargin'])
    
    Criteria['openloop_pitchcmd_phasemargin']['res'] = PM_deg
    Criteria['openloop_pitchcmd_phasemargin']['cost'] = CalculateCost(Criteria['openloop_pitchcmd_phasemargin'])
    
    # %% Phase/Gain Margin - OpenLoop Q
    Criteria['openloop_q_gainmargin'] = {}
    Criteria['openloop_q_gainmargin']['weight_over'] = 0
    Criteria['openloop_q_gainmargin']['weight_down'] = 1
    Criteria['openloop_q_gainmargin']['weight']      = 10
    Criteria['openloop_q_gainmargin']['target']      = 8.0
    Criteria['openloop_q_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_q_phasemargin'] = {}
    Criteria['openloop_q_phasemargin']['weight_over'] = 0
    Criteria['openloop_q_phasemargin']['weight_down'] = 1
    Criteria['openloop_q_phasemargin']['weight']      = 10
    Criteria['openloop_q_phasemargin']['target']      = 60.0
    Criteria['openloop_q_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_q'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_q_gainmargin']['res'] = GM_dB
    Criteria['openloop_q_gainmargin']['cost'] = CalculateCost(Criteria['openloop_q_gainmargin'])
    
    Criteria['openloop_q_phasemargin']['res'] = PM_deg
    Criteria['openloop_q_phasemargin']['cost'] = CalculateCost(Criteria['openloop_q_phasemargin'])
    
    # %% Phase/Gain Margin - OpenLoop THETA
    Criteria['openloop_theta_gainmargin'] = {}
    Criteria['openloop_theta_gainmargin']['weight_over'] = 0
    Criteria['openloop_theta_gainmargin']['weight_down'] = 1
    Criteria['openloop_theta_gainmargin']['weight']      = 10
    Criteria['openloop_theta_gainmargin']['target']      = 8.0
    Criteria['openloop_theta_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_theta_phasemargin'] = {}
    Criteria['openloop_theta_phasemargin']['weight_over'] = 0
    Criteria['openloop_theta_phasemargin']['weight_down'] = 1
    Criteria['openloop_theta_phasemargin']['weight']      = 10
    Criteria['openloop_theta_phasemargin']['target']      = 60.0
    Criteria['openloop_theta_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_theta'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_theta_gainmargin']['res'] = GM_dB
    Criteria['openloop_theta_gainmargin']['cost'] = CalculateCost(Criteria['openloop_theta_gainmargin'])
    
    Criteria['openloop_theta_phasemargin']['res'] = PM_deg
    Criteria['openloop_theta_phasemargin']['cost'] = CalculateCost(Criteria['openloop_theta_phasemargin'])
    
    
    return Criteria
