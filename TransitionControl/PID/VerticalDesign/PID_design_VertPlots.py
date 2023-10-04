import numpy as np
import control as ct
import matplotlib.pyplot as plt

def gen_Plots(ClosedLoops , Criteria, PlotLabel):
    
    BodePlot_minFreq_10e_radps = -3
    BodePlot_maxFreq_10e_radps = 2

    # %% Z CMD RESPONSE
    T, yout = ct.step_response(ClosedLoops['PitchNotIncluded'] , T=15, input = 1)
    
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig1 = plt.figure('Z_Step')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0], label = PlotLabel)
        plt.grid('on')
        plt.ylabel(ClosedLoops['PitchNotIncluded'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.legend()
    plt.suptitle('Z Command Step')
    fig1.savefig('Step_Z.png', bbox_inches='tight')
    plt.show()

  
    # %% VZ CMD RESPONSE, Z Open
    T, yout = ct.step_response(ClosedLoops['NoZFeedback'] , T=10, input = 0)
    
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig2 = plt.figure('VZ_Step')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0], label = PlotLabel)
        plt.grid('on')
        plt.ylabel(ClosedLoops['NoZFeedback'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.legend()
    plt.suptitle('VZ Command Step (No Z feedback')
    fig2.savefig('Step_Z_NoZFeedback.png', bbox_inches='tight')
    plt.show()
    
    # %% OPEN LOOP THROTTLE CMD MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_throttlecmd'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=100),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_throttlecmd'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    

    fig5 = plt.figure('OpenLoop_throttlecmd_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db)
    plt.plot(w_radps , (w_radps*0), 'k:')
    for i in range(len(GM_dB)):
        if margins_res[3][i] < 100:
            if GM_dB[i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[3][i] , w_radps, G_db)
            plt.plot(np.array([1, 1])*margins_res[3][i] , np.array([0, -GM_dB[i]]), plot_pm)       
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg, label = PlotLabel)
    plt.plot(w_radps , (w_radps*0-180), 'k:')
    plt.plot(w_radps , (w_radps*0+180), 'k:')
    for i in range(len(margins_res[1])):
        if margins_res[4][i] < 100:
            if margins_res[1][i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[4][i] , w_radps, P_deg)
            if P > 0:
                Pref = 180
            else:
                Pref = -180
    
            plt.plot(np.array([1, 1])*margins_res[4][i] , np.array([Pref, Pref+margins_res[1][i]]), plot_pm)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(P_deg , G_db)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(-180+np.array([-Criteria['openloop_throttlecmd_phasemargin']['target'] , 0 , +Criteria['openloop_throttlecmd_phasemargin']['target'], 0 , -Criteria['openloop_throttlecmd_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_throttlecmd_gainmargin']['target'], 0 , -Criteria['openloop_throttlecmd_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.suptitle('Open Loop PitchCmd Margin')
    fig5.tight_layout(rect=[0, 0.03, 1, 0.95]) 
    fig5.savefig('Bode_OpenLoop_throttlecmd.png', bbox_inches='tight')
    plt.show()
    
    # %% OPEN LOOP Q MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_VZ'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=100),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_VZ'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    

    fig6 = plt.figure('OpenLoop_VZ_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db)
    plt.plot(w_radps , (w_radps*0), 'k:')
    for i in range(len(GM_dB)):
        if margins_res[3][i] < 100:
            if GM_dB[i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[3][i] , w_radps, G_db)
            plt.plot(np.array([1, 1])*margins_res[3][i] , np.array([0, -GM_dB[i]]), plot_pm)       
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg, label = PlotLabel)
    plt.plot(w_radps , (w_radps*0-180), 'k:')
    plt.plot(w_radps , (w_radps*0+180), 'k:')
    for i in range(len(margins_res[1])):
        if margins_res[4][i] < 100:
            if margins_res[1][i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[4][i] , w_radps, P_deg)
            if P > 0:
                Pref = 180
            else:
                Pref = -180
    
            plt.plot(np.array([1, 1])*margins_res[4][i] , np.array([Pref, Pref+margins_res[1][i]]), plot_pm)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(P_deg , G_db)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(-180+np.array([-Criteria['openloop_VZ_phasemargin']['target'] , 0 , +Criteria['openloop_VZ_phasemargin']['target'], 0 , -Criteria['openloop_VZ_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_VZ_gainmargin']['target'], 0 , -Criteria['openloop_VZ_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.suptitle('Open Loop PitchCmd Margin')
    fig6.tight_layout(rect=[0, 0.03, 1, 0.95]) 
    fig6.savefig('Bode_OpenLoop_VZ.png', bbox_inches='tight')
    plt.show()
    
    # %% OPEN LOOP THETA MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_Z'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=100),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_Z'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    

    fig7 = plt.figure('OpenLoop_Z_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db)
    plt.plot(w_radps , (w_radps*0), 'k:')
    for i in range(len(GM_dB)):
        if margins_res[3][i] < 100:
            if GM_dB[i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[3][i] , w_radps, G_db)
            plt.plot(np.array([1, 1])*margins_res[3][i] , np.array([0, -GM_dB[i]]), plot_pm)       
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg, label = PlotLabel)
    plt.plot(w_radps , (w_radps*0-180), 'k:')
    plt.plot(w_radps , (w_radps*0+180), 'k:')
    for i in range(len(margins_res[1])):
        if margins_res[4][i] < 100:
            if margins_res[1][i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[4][i] , w_radps, P_deg)
            if P > 0:
                Pref = 180
            else:
                Pref = -180
    
            plt.plot(np.array([1, 1])*margins_res[4][i] , np.array([Pref, Pref+margins_res[1][i]]), plot_pm)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(P_deg , G_db)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(-180+np.array([-Criteria['openloop_Z_phasemargin']['target'] , 0 , +Criteria['openloop_Z_phasemargin']['target'], 0 , -Criteria['openloop_Z_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_Z_gainmargin']['target'], 0 , -Criteria['openloop_Z_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.suptitle('Open Loop PitchCmd Margin')
    fig7.tight_layout(rect=[0, 0.03, 1, 0.95]) 
    fig7.savefig('Bode_OpenLoop_Z.png', bbox_inches='tight')
    plt.show()
    
