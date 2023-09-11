import numpy as np
import control as ct
import matplotlib.pyplot as plt

def gen_Plots(ClosedLoops , Criteria, PlotLabel):
    # %% Z CMD RESPONSE
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=15, input = 1)
    
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig1 = plt.figure('Z_Step')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0], label = PlotLabel)
        plt.grid('on')
        plt.ylabel(ClosedLoops['Complete'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.legend()
    plt.suptitle('Z Command Step')
    plt.show()

  
    # %% VZ CMD RESPONSE, Z Open
    T, yout = ct.step_response(ClosedLoops['Open_Z'] , T=10, input = 0)
    
    # plt.close('all')
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig2 = plt.figure('VZ_Step')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0], label = PlotLabel)
        plt.grid('on')
        plt.ylabel(ClosedLoops['Open_Z'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.legend()
    plt.suptitle('VZ Command Step (No Z feedback')
    plt.show()
    
    # %% Z MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['Z2Z'],omega = 10**np.linspace(-2,2,num=100),plot = False)
    G_db = 20*np.log10(G_adm)
    P_deg = np.rad2deg(P_rad)
    PM_deg = P_deg - (-180)
    GM_dB = 0 - G_db
    
    
    [T_GM_adm, T_PM_deg, T_wGM_radps, T_wPM_radps] = ct.margin(G_adm,P_deg,w_radps);
    T_GM_dB = 20*np.log10(T_GM_adm)
    
    
    fig3 = plt.figure('Z_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3, label = PlotLabel)
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
    plt.plot(np.array([-Criteria['z_phasemargin']['target'] , 0 , +Criteria['z_phasemargin']['target'], 0 , -Criteria['z_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['z_gainmargin']['target'], 0 , -Criteria['z_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.suptitle('Z Loop Margin')
    plt.show()
    
    # %% VZ MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['Only_VZ'],omega = 10**np.linspace(-2,2,num=100),plot = False)
    P_deg = np.rad2deg(P_rad)
    
    if (abs(P_deg[0]+360) < 10):
        P_deg += 360
        
    P_rad = np.rad2deg(P_deg)
    G_db = 20*np.log10(G_adm)
    PM_deg = P_deg - (-180)
    GM_dB = 0 - G_db
    
    
    [T_GM_adm, T_PM_deg, T_wGM_radps, T_wPM_radps] = ct.margin(G_adm,P_deg,w_radps);
    T_GM_dB = 20*np.log10(T_GM_adm)
    
    
    fig3 = plt.figure('VZ_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3, label = PlotLabel)
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
    plt.plot(np.array([-Criteria['vz_phasemargin']['target'] , 0 , +Criteria['vz_phasemargin']['target'], 0 , -Criteria['vz_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['vz_gainmargin']['target'], 0 , -Criteria['vz_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.suptitle('Q Loop Margin')
    plt.show()
