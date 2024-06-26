import numpy as np
import control as ct
import matplotlib.pyplot as plt

def PitchPlots(ClosedLoops , Criteria, PlotLabel, color_rgb = (0,0,1), line_type = '-', plot_criteria = False):
    
    BodePlot_minFreq_10e_radps = -3
    BodePlot_maxFreq_10e_radps = 2
    
    # %% THETA CMD RESPONSE
    T, yout = ct.step_response(ClosedLoops['AltitudeIncluded'] , T=5, input = 1)
    
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig1 = plt.figure('ThetaStep')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['AltitudeIncluded'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=3)
    if plot_criteria:
        plt.subplot(l,c,1)
        plt.plot(np.array([Criteria['theta_risetime80']['target'] , np.max(T)]) , np.array([0.8 , 0.8]) , 'k--')
        plt.text(Criteria['theta_risetime80']['target'] , 0.8+0.01 , 'Rise Time Criteria')
        plt.plot(np.array([0                                      , np.max(T)]) , 1+Criteria['theta_overshoot']['target']*np.array([1 , 1]) , 'k--')
        plt.text(0 , 1+Criteria['theta_overshoot']['target']+0.01 , 'Overshoot Criteria')
        plt.ylim((0, 1.3))
    plt.suptitle('Theta Command Step')
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    fig1.savefig('LinearResponse_PitchController_Thetastep.png', bbox_inches='tight')
    plt.show()

  
    # %% Q CMD RESPONSE, Theta Open
    T, yout = ct.step_response(ClosedLoops['NoThetaFeedback'] , T=5, input = 0)
    
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig2 = plt.figure('QStep')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['NoThetaFeedback'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=3)
    if plot_criteria:
        plt.subplot(l,c,2)
        plt.plot(np.array([0                                      , np.max(T)]) , 1+Criteria['q_overshoot']['target']*np.array([1 , 1]) , 'k--')
        plt.text(0 , 1+Criteria['q_overshoot']['target']+0.01 , 'Overshoot Criteria')
        plt.ylim((0, 1.3))
    plt.suptitle('Q Command Step (No theta feedback')
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    fig2.savefig('LinearResponse_PitchController_Qstep.png', bbox_inches='tight')
    plt.show()

    # %% THETA RESPONSE TO THROTTLE INPUT
    T, yout = ct.step_response(ClosedLoops['AltitudeIncluded'] , T=5, input = 3)
    
    l = int(np.ceil(np.sqrt(len(yout))))
    c = int(np.ceil(len(yout) / l))
    fig3 = plt.figure('ThrottleDist')
    for i in range(len(yout)):
        plt.subplot(l,c,i+1)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['AltitudeIncluded'].output_labels[i])
        plt.xlim([0,max(T)] )
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=3)
    if plot_criteria:
        plt.subplot(l,c,1)
        plt.plot(np.array([Criteria['disturb_rejection']['target'] , np.max(T)]) , np.array([0.2 , 0.2]) , 'k--')
        plt.plot(np.array([Criteria['disturb_rejection']['target'] , np.max(T)]) , -np.array([0.2 , 0.2]) , 'k--')
        plt.text(Criteria['disturb_rejection']['target'] , 0.2+0.01 , 'Accomodation Time Criteria')
    plt.suptitle('Throttle Disturbance')
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    fig3.savefig('LinearResponse_PitchController_ThrottleInp.png', bbox_inches='tight')
    plt.show()

   
    # %% OPEN LOOP PITCH CMD MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_pitchcmd'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 15) and (P_deg[i-1] > 345)) or ((P_deg[i-1] < 15) and (P_deg[i] > 345))):
            P_deg[i] = np.nan
            
    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_pitchcmd'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    
    fig5 = plt.figure('OpenLoop_PitchCmd_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db, line_type, color = color_rgb)
    plt.plot(w_radps , (w_radps*0), 'k:')
    for i in range(len(GM_dB)):
        if margins_res[3][i] < 100:
            if GM_dB[i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[3][i] , w_radps, G_db)
            # plt.plot(np.array([1, 1])*margins_res[3][i] , np.array([0, -GM_dB[i]]), plot_pm)       
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg, line_type, label = PlotLabel, color = color_rgb)
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
    
            # plt.plot(np.array([1, 1])*margins_res[4][i] , np.array([Pref, Pref+margins_res[1][i]]), plot_pm)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(180+P_deg , G_db, line_type, color = color_rgb)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(+np.array([-Criteria['openloop_pitchcmd_phasemargin']['target'] , 0 , +Criteria['openloop_pitchcmd_phasemargin']['target'], 0 , -Criteria['openloop_pitchcmd_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_pitchcmd_gainmargin']['target'], 0 , -Criteria['openloop_pitchcmd_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.xlim((-90,90))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop PitchCmd Margin')
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    fig5.savefig('LinearResponse_PitchController_Bode_OpenLoop_pitchcmd.png', bbox_inches='tight')
    plt.show()

    # %% OPEN LOOP THETA MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_theta'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 15) and (P_deg[i-1] > 345)) or ((P_deg[i-1] < 15) and (P_deg[i] > 345))):
            P_deg[i] = np.nan
            
    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
       
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_theta'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    
    fig6 = plt.figure('OpenLoop_Theta_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db, line_type, color = color_rgb)
    plt.plot(w_radps , (w_radps*0), 'k:')
    
    
    for i in range(len(GM_dB)):
        if margins_res[3][i] < 100:
            if GM_dB[i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[3][i] , w_radps, G_db)
            # plt.plot(np.array([1, 1])*margins_res[3][i] , np.array([0, -GM_dB[i]]), plot_pm)       
    
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg, line_type, label = PlotLabel, color = color_rgb)
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
            # plt.plot(np.array([1, 1])*margins_res[4][i] , np.array([Pref, Pref+margins_res[1][i]]), plot_pm)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(180+P_deg , G_db, line_type, color = color_rgb)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(+np.array([-Criteria['openloop_theta_phasemargin']['target'] , 0 , +Criteria['openloop_theta_phasemargin']['target'], 0 , -Criteria['openloop_theta_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_theta_gainmargin']['target'], 0 , -Criteria['openloop_theta_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.xlim((-90,90))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop PitchCmd Margin')
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    fig6.savefig('LinearResponse_PitchController_Bode_OpenLoop_Theta.png', bbox_inches='tight')
    plt.show()

    # %% OPEN LOOP Q MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_q'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 15) and (P_deg[i-1] > 345)) or ((P_deg[i-1] < 15) and (P_deg[i] > 345))):
            P_deg[i] = np.nan
            
    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_q'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    
    fig7 = plt.figure('OpenLoop_Q_MarginPlot')
    plt.subplot(2,2,1)
    plt.plot(w_radps , G_db, line_type, color = color_rgb)
    plt.plot(w_radps , (w_radps*0), 'k:')
    
    
    for i in range(len(GM_dB)):
        if margins_res[3][i] < 100:
            if GM_dB[i] < 0:
                plot_pm = 'r--'
            else:
                plot_pm = 'g--'
            P = np.interp(margins_res[3][i] , w_radps, G_db)
            # plt.plot(np.array([1, 1])*margins_res[3][i] , np.array([0, -GM_dB[i]]), plot_pm)       
    
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Gain [dB]')
    
    plt.subplot(2,2,3)
    plt.plot(w_radps , P_deg, line_type, label = PlotLabel, color = color_rgb)
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
            # plt.plot(np.array([1, 1])*margins_res[4][i] , np.array([Pref, Pref+margins_res[1][i]]), plot_pm)
    plt.xscale("log")
    plt.grid('on')
    plt.xlabel('Frequency [rad/s]')
    plt.ylabel('Phase [deg]')
    
    plt.subplot(1,2,2)
    plt.plot(180+P_deg , G_db, line_type, color = color_rgb)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(np.array([-Criteria['openloop_q_phasemargin']['target'] , 0 , +Criteria['openloop_q_phasemargin']['target'], 0 , -Criteria['openloop_q_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_q_gainmargin']['target'], 0 , -Criteria['openloop_q_gainmargin']['target'] , 0]) ,
                        'r', linewidth = 2) 
    plt.xlim((-90,90))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop Q Margin')
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    fig7.savefig('LinearResponse_PitchController_Bode_OpenLoop_Q.png', bbox_inches='tight')
    plt.show()
