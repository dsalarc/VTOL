import numpy as np
import control as ct
import matplotlib.pyplot as plt

def PitchPlots(ClosedLoops , Criteria):
    # %% THETA CMD RESPONSE
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=5, input = 1)
    
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig1 = plt.figure('ThetaStep')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0])
        plt.grid('on')
        plt.ylabel(ClosedLoops['Complete'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.suptitle('Theta Command Step')
    plt.show()

  
    # %% Q CMD RESPONSE, Theta Open
    T, yout = ct.step_response(ClosedLoops['ThetaOpen'] , T=5, input = 0)
    
    # plt.close('all')
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig2 = plt.figure('QStep')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0])
        plt.grid('on')
        plt.ylabel(ClosedLoops['ThetaOpen'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.suptitle('Q Command Step (No theta feedback')
    plt.show()
    
    # %% THETA MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['T2T'],omega = 10**np.linspace(-2,2,num=100),plot = False)
    G_db = 20*np.log10(G_adm)
    P_deg = np.rad2deg(P_rad)
    PM_deg = P_deg - (-180)
    GM_dB = 0 - G_db
    
    
    [T_GM_adm, T_PM_deg, T_wGM_radps, T_wPM_radps] = ct.margin(G_adm,P_deg,w_radps);
    T_GM_dB = 20*np.log10(T_GM_adm)
    
    
    fig3 = plt.figure('ThetaMarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(PM_deg , G_db)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(np.array([-Criteria['theta_phasemargin']['target'] , 0 , +Criteria['theta_phasemargin']['target'], 0 , -Criteria['theta_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['theta_gainmargin']['target'], 0 , -Criteria['theta_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.suptitle('Theta Loop Margin')
    plt.show()
    
    # %% Q MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OnlyQ'],omega = 10**np.linspace(-2,2,num=100),plot = False)
    P_deg = np.rad2deg(P_rad)
    
    if (abs(P_deg[0]+360) < 10):
        P_deg += 360
        
    P_rad = np.rad2deg(P_deg)
    G_db = 20*np.log10(G_adm)
    PM_deg = P_deg - (-180)
    GM_dB = 0 - G_db
    
    
    [T_GM_adm, T_PM_deg, T_wGM_radps, T_wPM_radps] = ct.margin(G_adm,P_deg,w_radps);
    T_GM_dB = 20*np.log10(T_GM_adm)
    
    
    fig3 = plt.figure('QMarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(PM_deg , G_db)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(np.array([-Criteria['theta_phasemargin']['target'] , 0 , +Criteria['theta_phasemargin']['target'], 0 , -Criteria['theta_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['theta_gainmargin']['target'], 0 , -Criteria['theta_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.suptitle('Q Loop Margin')
    plt.show()
