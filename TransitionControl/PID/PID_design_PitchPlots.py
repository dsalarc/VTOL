import numpy as np
import control as ct
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 12})

def PitchPlots(ClosedLoops , Criteria, PlotLabel, color_rgb = (0,0,1), line_type = '-', plot_criteria = False, plot_legend = True):
    
    ReportPlot = True

    BodePlot_minFreq_10e_radps = -3
    BodePlot_maxFreq_10e_radps = 2
    
    # %% THETA CMD RESPONSE
    T, yout = ct.step_response(ClosedLoops['AltitudeIncluded'] , T=5, input = 1)
    
    if ReportPlot:
        PlotList = np.array([0,1,2,4,5,6,7])
    else:
        PlotList = np.arange(len(yout))
        
    l = int(np.ceil(np.sqrt(len(PlotList))))
    c = int(np.ceil(len(PlotList) / l))
    fig1 = plt.figure('ThetaStep')
    n = 0
    for i in PlotList:
        n+=1
        plt.subplot(l,c,n)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['AltitudeIncluded'].output_labels[i])
        plt.xlim([0,max(T)] )
        if n > (l-1)*c:
            plt.xlabel('Time [s]')

    if plot_criteria:
        plt.subplot(l,c,1)
        plt.plot(np.array([Criteria['theta_risetime80']['target'] , Criteria['theta_risetime80']['target'] , Criteria['theta_risetime80']['target']+7]) , np.array([0.6, 0.8 , 0.8]) , 'k--')
        plt.text(Criteria['theta_risetime80']['target']+0.1 , 0.6 , 'Rise Time Criteria')
        plt.plot(np.array([0                                      , np.max(T)]) , 1+Criteria['theta_overshoot']['target']*np.array([1 , 1]) , 'k--')
        plt.text(0 , 1+Criteria['theta_overshoot']['target']+0.01 , 'Overshoot Criteria')
        plt.ylim((0, 1.4))
        
    plt.suptitle('Theta Command Step')
    fig1.set_size_inches(12, 6)
    fig1.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    plt.show()
    plt.subplot(l,c,n)
    if plot_legend:
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=3)
    plt.savefig('LinearResponse_PitchController_Thetastep.pdf', format="pdf", dpi=fig1.dpi)
  
    # %% Q CMD RESPONSE, Theta Open
    T, yout = ct.step_response(ClosedLoops['NoThetaFeedback'] , T=5, input = 0)
    
    if ReportPlot:
        PlotList = np.array([0,1,2,4,5,6,7])
    else:
        PlotList = np.arange(len(yout))

    l = int(np.ceil(np.sqrt(len(PlotList))))
    c = int(np.ceil(len(PlotList) / l))
    fig2 = plt.figure('QStep')
    n = 0
    for i in PlotList:
        n+=1
        plt.subplot(l,c,n)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['NoThetaFeedback'].output_labels[i])
        plt.xlim([0,max(T)] )
        if n > (l-1)*c:
            plt.xlabel('Time [s]')

    if plot_criteria:
        plt.subplot(l,c,2)
        plt.plot(np.array([0                                      , np.max(T)]) , 1+Criteria['q_overshoot']['target']*np.array([1 , 1]) , 'k--')
        plt.text(0 , 1+Criteria['q_overshoot']['target']+0.01 , 'Overshoot Criteria')
        plt.ylim((0, 1.4))
    plt.suptitle('Q Command Step (No theta feedback')
    fig2.set_size_inches(12, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    plt.show()
    plt.subplot(l,c,n)
    if plot_legend:
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=3)
    fig2.savefig('LinearResponse_PitchController_Qstep.pdf', format="pdf", dpi=fig1.dpi)

    # %% THETA RESPONSE TO THROTTLE INPUT
    T, yout = ct.step_response(ClosedLoops['AltitudeIncluded'] , T=5, input = 3)
    
    if ReportPlot:
        PlotList = np.array([0,1,2,4,5,6,7,15])
    else:
        PlotList = np.arange(len(yout))
    
    l = int(np.ceil(np.sqrt(len(PlotList))))
    c = int(np.ceil(len(PlotList) / l))
    fig3 = plt.figure('ThrottleDist')
    n = 0
    for i in PlotList:
        n+=1
        plt.subplot(l,c,n)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['AltitudeIncluded'].output_labels[i])
        plt.xlim([0,max(T)] )
        if n > (l-1)*c:
            plt.xlabel('Time [s]')

    if plot_criteria:
        plt.subplot(l,c,1)
        plt.plot(np.array([Criteria['disturb_rejection']['target'] , np.max(T)]) , np.array([0.2 , 0.2]) , 'k--')
        plt.plot(np.array([Criteria['disturb_rejection']['target'] , np.max(T)]) , -np.array([0.2 , 0.2]) , 'k--')
        plt.text(Criteria['disturb_rejection']['target'] , 0.2+0.01 , 'Accomodation Time Criteria')

    plt.suptitle('Throttle Disturbance')
    fig3.set_size_inches(12, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    plt.show()
    plt.subplot(l,c,n)
    if plot_legend:
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=3)
    fig3.savefig('LinearResponse_PitchController_ThrottleInp.pdf', format="pdf", dpi=fig1.dpi)

   
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
    
    fig4 = plt.figure('OpenLoop_PitchCmd_MarginPlot')
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
    plt.plot(180+P_deg , G_db, line_type, color = color_rgb, label = PlotLabel)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(+np.array([-Criteria['openloop_pitchcmd_phasemargin']['target'] , 0 , +Criteria['openloop_pitchcmd_phasemargin']['target'], 0 , -Criteria['openloop_pitchcmd_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_pitchcmd_gainmargin']['target'], 0 , -Criteria['openloop_pitchcmd_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-80,100))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop PitchCmd Margin')
    fig4.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='lower right',ncol=2, fontsize=8)
    plt.show()
    fig4.savefig('LinearResponse_PitchController_Bode_OpenLoop_pitchcmd.pdf', format="pdf", dpi=fig1.dpi)

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
    
    fig5 = plt.figure('OpenLoop_Theta_MarginPlot')
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
    plt.plot(180+P_deg , G_db, line_type, color = color_rgb, label = PlotLabel)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(+np.array([-Criteria['openloop_theta_phasemargin']['target'] , 0 , +Criteria['openloop_theta_phasemargin']['target'], 0 , -Criteria['openloop_theta_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_theta_gainmargin']['target'], 0 , -Criteria['openloop_theta_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-80,100))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop ThetaCmd Margin')
    fig5.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='lower right',ncol=2, fontsize=8)
    plt.show()
    fig5.savefig('LinearResponse_PitchController_Bode_OpenLoop_Theta.pdf', format="pdf", dpi=fig1.dpi)

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
    
    fig6 = plt.figure('OpenLoop_Q_MarginPlot')
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
    plt.plot(180+P_deg , G_db, line_type, color = color_rgb, label = PlotLabel)
    plt.grid('on')
    plt.xlabel('Phase Margin [deg]')
    plt.ylabel('Gain Margin [dB]')
    plt.plot(np.array([-Criteria['openloop_q_phasemargin']['target'] , 0 , +Criteria['openloop_q_phasemargin']['target'], 0 , -Criteria['openloop_q_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_q_gainmargin']['target'], 0 , -Criteria['openloop_q_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-80,100))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop ThetaCmd Margin')
    fig6.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='lower right',ncol=2, fontsize=8)
    plt.show()
    fig6.savefig('LinearResponse_PitchController_Bode_OpenLoop_Q.pdf', format="pdf", dpi=fig1.dpi)
