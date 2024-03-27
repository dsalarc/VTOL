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
    
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=5, input = ClosedLoops['Complete'].input_labels.index('Z_ref_m'))
    max_Z = np.max(yout[ClosedLoops['Complete'].output_labels.index('Z_m')][0])
    Criteria['z_overshoot']['res'] = (max_Z - 1)
    Criteria['z_overshoot']['cost'] = (np.max([Criteria['z_overshoot']['res'] - Criteria['z_overshoot']['target'],0]) * Criteria['z_overshoot']['weight_over'] + 
                                           np.min([Criteria['z_overshoot']['res'] - Criteria['z_overshoot']['target'],0]) * -Criteria['z_overshoot']['weight_down'])
    
    # Z Rise
    Criteria['z_risetime80'] = {}
    Criteria['z_risetime80']['weight_over'] = 1
    Criteria['z_risetime80']['weight_down'] = -0.1
    Criteria['z_risetime80']['weight'] = 5
    Criteria['z_risetime80']['target'] = 3.0
    
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=10, input = ClosedLoops['Complete'].input_labels.index('Z_ref_m'))
    over_riseLim = T[yout[ClosedLoops['Complete'].output_labels.index('Z_m')][0] > 0.8]
    if len(over_riseLim) == 0:
        z_rt = 10
    else:
        z_rt = T[yout[ClosedLoops['Complete'].output_labels.index('Z_m')][0] > 0.8][0]
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
    Criteria['vz_steadystate']['weight'] = 50
    Criteria['vz_steadystate']['target'] = 1.0
    

    dcgain = ct.dcgain(ClosedLoops['VZ2VZ'])
    if np.isnan(dcgain):
        T,yout = ct.step_response(ClosedLoops['VZ2VZ'],10)
        dcgain = yout[-1]
        
    Criteria['vz_steadystate']['res'] = dcgain
        
    Criteria['vz_steadystate']['cost'] = (np.max([Criteria['vz_steadystate']['res'] - Criteria['vz_steadystate']['target'],0]) * Criteria['vz_steadystate']['weight_over'] + 
                                           np.min([Criteria['vz_steadystate']['res'] - Criteria['vz_steadystate']['target'],0]) * -Criteria['vz_steadystate']['weight_down'])
    
    # VX overshoot
    Criteria['vx_overshoot'] = {}
    Criteria['vx_overshoot']['weight_over'] = 1
    Criteria['vx_overshoot']['weight_down'] = 0
    Criteria['vx_overshoot']['weight'] = 1
    Criteria['vx_overshoot']['target'] = 0.2
    
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=5, input = ClosedLoops['Complete'].input_labels.index('VX_ref_mps'))
    max_VX = np.max(yout[ClosedLoops['Complete'].output_labels.index('VX_mps')][0])
    Criteria['vx_overshoot']['res'] = (max_VX - 1)
    Criteria['vx_overshoot']['cost'] = (np.max([Criteria['vx_overshoot']['res'] - Criteria['vx_overshoot']['target'],0]) * Criteria['vx_overshoot']['weight_over'] + 
                                           np.min([Criteria['vx_overshoot']['res'] - Criteria['vx_overshoot']['target'],0]) * -Criteria['vx_overshoot']['weight_down'])
    
    # VX Rise
    Criteria['vx_risetime80'] = {}
    Criteria['vx_risetime80']['weight_over'] = 1
    Criteria['vx_risetime80']['weight_down'] = -0.1
    Criteria['vx_risetime80']['weight'] = 5
    Criteria['vx_risetime80']['target'] = 3.0
    
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=10, input = ClosedLoops['Complete'].input_labels.index('VX_ref_mps'))
    over_riseLim = T[yout[ClosedLoops['Complete'].output_labels.index('VX_mps')][0] > 0.8]
    if len(over_riseLim) == 0:
        vx_rt = 10
    else:
        vx_rt = T[yout[ClosedLoops['Complete'].output_labels.index('VX_mps')][0] > 0.8][0]
    Criteria['vx_risetime80']['res'] = vx_rt
    Criteria['vx_risetime80']['cost'] = (np.max([vx_rt - Criteria['vx_risetime80']['target'],0]) * Criteria['vx_risetime80']['weight_over'] + 
                                            np.min([vx_rt - Criteria['vx_risetime80']['target'],0]) * -Criteria['vx_risetime80']['weight_down'])
    
    
    # VX Steady Error
    Criteria['vx_steadystate'] = {}
    Criteria['vx_steadystate']['weight_over'] = 1
    Criteria['vx_steadystate']['weight_down'] = 1
    Criteria['vx_steadystate']['weight'] = 10
    Criteria['vx_steadystate']['target'] = 1.0
    
    dcgain = ct.dcgain(ClosedLoops['VX2VX'])
    if np.isnan(dcgain):
        T,yout = ct.step_response(ClosedLoops['VX2VX'],20)
        dcgain = yout[-1]
        
    Criteria['vx_steadystate']['res'] = dcgain
        
    Criteria['vx_steadystate']['cost'] = (np.max([Criteria['vx_steadystate']['res'] - Criteria['vx_steadystate']['target'],0]) * Criteria['vx_steadystate']['weight_over'] + 
                                           np.min([Criteria['vx_steadystate']['res'] - Criteria['vx_steadystate']['target'],0]) * -Criteria['vx_steadystate']['weight_down'])
    # Theta Response
    # Reduce initial Theta response
    Criteria['max_theta'] = {}
    Criteria['max_theta']['weight_over'] = 1
    Criteria['max_theta']['weight_down'] = 0
    Criteria['max_theta']['weight'] = 5
    Criteria['max_theta']['target'] = 0
    Criteria['max_theta']['type'] = 'abs'

    T, yout = ct.step_response(ClosedLoops['Complete'] , T=15, input = ClosedLoops['Complete'].input_labels.index('Z_ref_m'))

    max_the = np.max(np.abs(yout[ClosedLoops['Complete'].output_labels.index('Theta_deg')][0]))
    Criteria['max_theta']['res'] = max_the
    Criteria['max_theta']['cost'] = CalculateCost (Criteria['max_theta'])
    
    # Throttle Response
    # Reduce initial throttle response, comparing to trim throttle
    Criteria['max_throttle'] = {}
    Criteria['max_throttle']['weight_over'] = 1
    Criteria['max_throttle']['weight_down'] = -0.1
    Criteria['max_throttle']['weight'] = 1
    Criteria['max_throttle']['target'] = 4
    Criteria['max_throttle']['type'] = 'abs'

    T, yout = ct.step_response(ClosedLoops['Complete'] , T=15, input = ClosedLoops['Complete'].input_labels.index('VX_ref_mps'))
    # ini_Thr = yout[ClosedLoops['Complete'].output_labels.index('ThrottleCmd_u')][0][0]
    max_thr = np.max(np.abs(yout[ClosedLoops['Complete'].output_labels.index('ThrottleCmd_u')][0]))
    end_thr = yout[ClosedLoops['Complete'].output_labels.index('ThrottleCmd_u')][0][-1]
    thr_gain = np.abs(max_thr / end_thr)
    # max_Thr = np.max(np.abs(yout[ClosedLoops['Complete'].output_labels.index('ThrottleCmd_u')][0]))
    Criteria['max_throttle']['res'] = thr_gain
    Criteria['max_throttle']['cost'] = CalculateCost (Criteria['max_throttle'])

    # %% CLOSED LOOP STABILITY
    Criteria['closedloop_stability'] = {}
    Criteria['closedloop_stability']['weight_over'] = 1
    Criteria['closedloop_stability']['weight_down'] = 0
    Criteria['closedloop_stability']['weight'] = 100
    Criteria['closedloop_stability']['target'] = 0.0
    Criteria['closedloop_stability']['type']   = 'abs'
   
    CL_poles = ct.pole(ClosedLoops['Complete'])
    CL_real = np.real(CL_poles)
    CL_imag = np.imag(CL_poles)
    CL_w    = (CL_real**2 + CL_imag**2)**0.5
    
    inst = np.max(CL_real * CL_w)
    
    if inst > 0.0:
        Criteria['closedloop_stability']['res'] = np.min([10.0, np.max([5.0 , inst])])
    else:
         Criteria['closedloop_stability']['res'] = 0.0
         
    Criteria['closedloop_stability']['cost'] = CalculateCost(Criteria['closedloop_stability'])
    
    
    
    # %% Phase/Gain Margin - OpenLoop ThetaCmd
    Criteria['openloop_thetacmd_gainmargin'] = {}
    Criteria['openloop_thetacmd_gainmargin']['weight_over'] = 0
    Criteria['openloop_thetacmd_gainmargin']['weight_down'] = 1
    Criteria['openloop_thetacmd_gainmargin']['weight']      = 10
    Criteria['openloop_thetacmd_gainmargin']['target']      = 8.0
    Criteria['openloop_thetacmd_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_thetacmd_phasemargin'] = {}
    Criteria['openloop_thetacmd_phasemargin']['weight_over'] = 0
    Criteria['openloop_thetacmd_phasemargin']['weight_down'] = 1
    Criteria['openloop_thetacmd_phasemargin']['weight']      = 10
    Criteria['openloop_thetacmd_phasemargin']['target']      = 60.0
    Criteria['openloop_thetacmd_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_thetacmd'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_thetacmd_gainmargin']['res'] = GM_dB
    Criteria['openloop_thetacmd_gainmargin']['cost'] = CalculateCost(Criteria['openloop_thetacmd_gainmargin'])
    
    Criteria['openloop_thetacmd_phasemargin']['res'] = PM_deg
    Criteria['openloop_thetacmd_phasemargin']['cost'] = CalculateCost(Criteria['openloop_thetacmd_phasemargin'])
    
    # %% Phase/Gain Margin - OpenLoop ThrottleCmd
    Criteria['openloop_throttlecmd_gainmargin'] = {}
    Criteria['openloop_throttlecmd_gainmargin']['weight_over'] = 0
    Criteria['openloop_throttlecmd_gainmargin']['weight_down'] = 1
    Criteria['openloop_throttlecmd_gainmargin']['weight']      = 10
    Criteria['openloop_throttlecmd_gainmargin']['target']      = 8.0
    Criteria['openloop_throttlecmd_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_throttlecmd_phasemargin'] = {}
    Criteria['openloop_throttlecmd_phasemargin']['weight_over'] = 0
    Criteria['openloop_throttlecmd_phasemargin']['weight_down'] = 1
    Criteria['openloop_throttlecmd_phasemargin']['weight']      = 10
    Criteria['openloop_throttlecmd_phasemargin']['target']      = 60.0
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
    Criteria['openloop_VZ_gainmargin']['target']      = 8.0
    Criteria['openloop_VZ_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_VZ_phasemargin'] = {}
    Criteria['openloop_VZ_phasemargin']['weight_over'] = 0
    Criteria['openloop_VZ_phasemargin']['weight_down'] = 1
    Criteria['openloop_VZ_phasemargin']['weight']      = 10
    Criteria['openloop_VZ_phasemargin']['target']      = 60.0
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
    Criteria['openloop_Z_gainmargin']['target']      = 8.0
    Criteria['openloop_Z_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_Z_phasemargin'] = {}
    Criteria['openloop_Z_phasemargin']['weight_over'] = 0
    Criteria['openloop_Z_phasemargin']['weight_down'] = 1
    Criteria['openloop_Z_phasemargin']['weight']      = 10
    Criteria['openloop_Z_phasemargin']['target']      = 60.0
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

    # %% Phase/Gain Margin - OpenLoop VZ
    Criteria['openloop_VX_gainmargin'] = {}
    Criteria['openloop_VX_gainmargin']['weight_over'] = 0
    Criteria['openloop_VX_gainmargin']['weight_down'] = 1
    Criteria['openloop_VX_gainmargin']['weight']      = 10
    Criteria['openloop_VX_gainmargin']['target']      = 8.0
    Criteria['openloop_VX_gainmargin']['type']        = 'prop'
    
    Criteria['openloop_VX_phasemargin'] = {}
    Criteria['openloop_VX_phasemargin']['weight_over'] = 0
    Criteria['openloop_VX_phasemargin']['weight_down'] = 1
    Criteria['openloop_VX_phasemargin']['weight']      = 10
    Criteria['openloop_VX_phasemargin']['target']      = 60.0
    Criteria['openloop_VX_phasemargin']['type']        = 'prop'
    
    aux_out = ct.stability_margins(ClosedLoops['OpenLoop_VX'],returnall=True)
    if len(np.log10(aux_out[0])) > 0:
        GM_dB  = np.min(np.abs(20*np.log10(aux_out[0])))
    else:
        GM_dB = 100
    if len(np.log10(aux_out[1])) > 0:
        PM_deg = np.min(np.abs(aux_out[1]))
    else:
        PM_deg = 100
    
    Criteria['openloop_VX_gainmargin']['res'] = GM_dB
    Criteria['openloop_VX_gainmargin']['cost'] = CalculateCost(Criteria['openloop_VX_gainmargin'])
    
    Criteria['openloop_VX_phasemargin']['res'] = PM_deg
    Criteria['openloop_VX_phasemargin']['cost'] = CalculateCost(Criteria['openloop_VX_phasemargin'])
    
    # %% RETURN
    
    return Criteria
