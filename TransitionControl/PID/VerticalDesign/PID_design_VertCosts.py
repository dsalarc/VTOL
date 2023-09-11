import numpy as np
import control as ct

def CalculateTotalCost(Criteria):
    TotalCost = 0
    for i in range(len(Criteria.keys())):
        CriteriaCost = Criteria[list(Criteria.keys())[i]]['cost'] * Criteria[list(Criteria.keys())[i]]['weight']
        # print("%s Cost: %0.2f" %(list(Criteria.keys())[i] , CriteriaCost))
        TotalCost += CriteriaCost
    
    return TotalCost

def CalculteCost (Criteria_dict):
    if Criteria_dict['type'] == 'prop':
        cost = ((np.max([Criteria_dict['res'] ,1]) - 1) * Criteria_dict['weight_over'] + 
               -(np.min([Criteria_dict['res'] ,1]) - 1) * Criteria_dict['weight_down'])
    elif Criteria_dict['type'] == 'abs':
        cost = (np.max([Criteria_dict['res'],0]) *  Criteria_dict['weight_over'] + 
                np.min([Criteria_dict['res'],0]) * -Criteria_dict['weight_down'])
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
    
    T, yout = ct.step_response(ClosedLoops['Open_Z'] , T=5, input = ClosedLoops['Open_Z'].input_labels.index('VZ_ref_mps'))
    max_q = np.max(yout[ClosedLoops['Open_Z'].output_labels.index('VZ_mps')][0])
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
    
    # Throttle Usage
    Criteria['max_throttle'] = {}
    Criteria['max_throttle']['weight_over'] = 1
    Criteria['max_throttle']['weight_down'] = 0
    Criteria['max_throttle']['weight'] = 1
    Criteria['max_throttle']['target'] = 0.1
    Criteria['max_throttle']['type'] = 'prop'

    T, yout = ct.step_response(ClosedLoops['Complete'] , T=5, input = ClosedLoops['Complete'].input_labels.index('Z_ref_m'))
    max_Thr = np.max(np.abs(yout[ClosedLoops['Complete'].output_labels.index('ThrottleCmd_u')][0]))
    Criteria['max_throttle']['res'] = (max_Thr/Criteria['max_throttle']['target'])
    Criteria['max_throttle']['cost'] = CalculteCost (Criteria['max_throttle'])

    # Z Rise
    Criteria['z_risetime80'] = {}
    Criteria['z_risetime80']['weight_over'] = 1
    Criteria['z_risetime80']['weight_down'] = -0.1
    Criteria['z_risetime80']['weight'] = 5
    Criteria['z_risetime80']['target'] = 2.0
    
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
        T,yout = ct.step_response(ClosedLoops['Only_VZ'],10)
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
    

    dcgain = ct.dcgain(ClosedLoops['Only_VZ'])
    if np.isnan(dcgain):
        T,yout = ct.step_response(ClosedLoops['Only_VZ'],10)
        dcgain = yout[-1]
        
    Criteria['vz_steadystate']['res'] = dcgain
        
    Criteria['vz_steadystate']['cost'] = (np.max([Criteria['vz_steadystate']['res'] - Criteria['vz_steadystate']['target'],0]) * Criteria['vz_steadystate']['weight_over'] + 
                                           np.min([Criteria['vz_steadystate']['res'] - Criteria['vz_steadystate']['target'],0]) * -Criteria['vz_steadystate']['weight_down'])
    
    
    # %% Damping
    # Q Damping
    Criteria['vz_damp'] = {}
    Criteria['vz_damp']['weight_over'] = 0
    Criteria['vz_damp']['weight_down'] = 1
    Criteria['vz_damp']['weight'] = 5*0
    Criteria['vz_damp']['target'] = 0.5
    
    aux = ct.damp(ClosedLoops['Only_VZ'] , doprint=False)
    Criteria['vz_damp']['res'] = min(aux[1])
        
    Criteria['vz_damp']['cost'] = (np.max([Criteria['vz_damp']['res'] - Criteria['vz_damp']['target'],0]) * Criteria['vz_damp']['weight_over'] + 
                                           np.min([Criteria['vz_damp']['res'] - Criteria['vz_damp']['target'],0]) * -Criteria['vz_damp']['weight_down'])
    
    
    
    # %% Phase/Gain Margin Z2Z
    Criteria['z_gainmargin'] = {}
    Criteria['z_gainmargin']['weight_over'] = 0
    Criteria['z_gainmargin']['weight_down'] = 1
    Criteria['z_gainmargin']['weight'] = 1
    Criteria['z_gainmargin']['target'] = 6.0
    
    Criteria['z_phasemargin'] = {}
    Criteria['z_phasemargin']['weight_over'] = 0
    Criteria['z_phasemargin']['weight_down'] = 1
    Criteria['z_phasemargin']['weight'] = 1/5
    Criteria['z_phasemargin']['target'] = 45.0
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['Z2Z'],omega = 10**np.linspace(-2,2,num=100),plot = False)
    G_db = 20*np.log10(G_adm)
    P_deg = np.rad2deg(P_rad)
    PM_deg = P_deg - (-180)
    GM_dB = 0 - G_db
    
    
    [T_GM_adm, T_PM_deg, T_wGM_radps, T_wPM_radps] = ct.margin(G_adm,P_deg,w_radps);
    T_GM_dB = 20*np.log10(T_GM_adm)
    
    Criteria['z_gainmargin']['res'] = min(T_GM_dB,1000)
    Criteria['z_gainmargin']['cost'] = (np.max([Criteria['z_gainmargin']['res'] - Criteria['z_gainmargin']['target'],0]) * Criteria['z_gainmargin']['weight_over'] + 
                                           np.min([Criteria['z_gainmargin']['res'] - Criteria['z_gainmargin']['target'],0]) * -Criteria['z_gainmargin']['weight_down'])
    
    Criteria['z_phasemargin']['res'] = min(T_PM_deg,1000)
    Criteria['z_phasemargin']['cost'] = (np.max([Criteria['z_phasemargin']['res'] - Criteria['z_phasemargin']['target'],0]) * Criteria['z_phasemargin']['weight_over'] + 
                                           np.min([Criteria['z_phasemargin']['res'] - Criteria['z_phasemargin']['target'],0]) * -Criteria['z_phasemargin']['weight_down'])
    
    # %% Phase/Gain Margin Q
    Criteria['vz_gainmargin'] = {}
    Criteria['vz_gainmargin']['weight_over'] = 0
    Criteria['vz_gainmargin']['weight_down'] = 1
    Criteria['vz_gainmargin']['weight'] = 1
    Criteria['vz_gainmargin']['target'] = 6.0
    
    Criteria['vz_phasemargin'] = {}
    Criteria['vz_phasemargin']['weight_over'] = 0
    Criteria['vz_phasemargin']['weight_down'] = 1
    Criteria['vz_phasemargin']['weight'] = 1/5
    Criteria['vz_phasemargin']['target'] = 45.0
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['Only_VZ'],omega = 10**np.linspace(-2,2,num=100),plot = False)
    G_db = 20*np.log10(G_adm)
    P_deg = np.rad2deg(P_rad)
    PM_deg = P_deg - (-180)
    GM_dB = 0 - G_db
    
    
    [Q_GM_adm, Q_PM_deg, Q_wGM_radps, Q_wPM_radps] = ct.margin(G_adm,P_deg,w_radps);
    Q_GM_dB = 20*np.log10(Q_GM_adm)
    
    Criteria['vz_gainmargin']['res'] = min(Q_GM_dB,1000)
    Criteria['vz_gainmargin']['cost'] = (np.max([Criteria['vz_gainmargin']['res'] - Criteria['vz_gainmargin']['target'],0]) * Criteria['vz_gainmargin']['weight_over'] + 
                                            np.min([Criteria['vz_gainmargin']['res'] - Criteria['vz_gainmargin']['target'],0]) * -Criteria['vz_gainmargin']['weight_down'])
    
    Criteria['vz_phasemargin']['res'] = min(Q_PM_deg,1000)
    Criteria['vz_phasemargin']['cost'] = (np.max([Criteria['vz_phasemargin']['res'] - Criteria['vz_phasemargin']['target'],0]) * Criteria['vz_phasemargin']['weight_over'] + 
                                            np.min([Criteria['vz_phasemargin']['res'] - Criteria['vz_phasemargin']['target'],0]) * -Criteria['vz_phasemargin']['weight_down'])
    
    
    return Criteria
