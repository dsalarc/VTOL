import numpy as np
import control as ct
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 12})

def gen_Plots(ClosedLoops , Criteria, PlotLabel, color_rgb = (0,0,1), line_type = '-', plot_criteria = False, plot_legend = True):
    
    ReportPlot = True
    
    BodePlot_minFreq_10e_radps = -3
    BodePlot_maxFreq_10e_radps = 2

    # %% Z CMD RESPONSE
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=15, input = 1)
    
    if ReportPlot:
        PlotList = np.array([0,1,2,3,4,5,7,10])
    else:
        PlotList = np.arange(len(yout))
    
    l = int(np.ceil(np.sqrt(len(PlotList))))
    c = int(np.ceil(len(PlotList) / l))
    fig1 = plt.figure('Z_Step')
    n = 0
    for i in PlotList:
        n+=1
        plt.subplot(l,c,n)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['Complete'].output_labels[i])
        plt.xlim([0,max(T)] )
        plt.xticks(np.arange(0, 15.1, step=3))
        if n > (l-1)*c:
            plt.xlabel('Time [s]')
            
    if plot_criteria:
        plt.subplot(l,c,1)
        plt.plot(np.array([Criteria['z_risetime80']['target'] , Criteria['z_risetime80']['target'] , Criteria['z_risetime80']['target']+7]) , np.array([0.6, 0.8 , 0.8]) , 'k--')
        plt.text(Criteria['z_risetime80']['target']+1 , 0.8+0.01 , 'Rise Time Criteria')
        plt.plot(np.array([0                                      , np.max(T)]) , 1+Criteria['z_overshoot']['target']*np.array([1 , 1]) , 'k--')
        plt.text(0 , 1+Criteria['z_overshoot']['target']+0.01 , 'Overshoot Criteria')
        plt.ylim((0, 1.3))
    plt.suptitle('Z Command Step')
    # figManager = plt.get_current_fig_manager()
    # figManager.window.showMaximized()
    fig1.set_size_inches(12, 6)
    fig1.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    plt.show()
    plt.subplot(l,c,n)
    if plot_legend:
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=2)
    plt.savefig('LinearResponse_AltitudeController_Zstep.pdf', format="pdf", dpi=fig1.dpi)

  
    # %% VZ CMD RESPONSE, Z Open
    T, yout = ct.step_response(ClosedLoops['NoZFeedback'] , T=15, input = 0)
    
    if ReportPlot:
        PlotList = np.array([0,1,2,3,4,5,7,10])
    else:
        PlotList = np.arange(len(yout))

    l = int(np.ceil(np.sqrt(len(PlotList))))
    c = int(np.ceil(len(PlotList) / l))
    fig2 = plt.figure('VZ_Step')
    n = 0
    for i in PlotList:
        n+=1
        plt.subplot(l,c,n)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['NoZFeedback'].output_labels[i])
        plt.xlim([0,max(T)] )
        plt.xticks(np.arange(0, 15.1, step=3))
        if n > (l-1)*c:
            plt.xlabel('Time [s]')

    if plot_criteria:
        plt.subplot(l,c,2)
        plt.plot(np.array([0                                      , np.max(T)]) , 1+Criteria['vz_overshoot']['target']*np.array([1 , 1]) , 'k--')
        plt.text(0 , 1+Criteria['vz_overshoot']['target']+0.01 , 'Overshoot Criteria')
        plt.ylim((0, 1.3))
    plt.suptitle('VZ Command Step (No Z feedback)')
    fig2.set_size_inches(12, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    plt.show()
    plt.subplot(l,c,n)
    if plot_legend:
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=2)
    plt.savefig('LinearResponse_AltitudeController_VZstep.pdf', dpi=fig2.dpi)
    
    # %% VX CMD RESPONSE
    T, yout = ct.step_response(ClosedLoops['Complete'] , T=20, input = 2)
    
    if ReportPlot:
        PlotList = np.array([0,1,2,3,4,5,7,10])
    else:
        PlotList = np.arange(len(yout))

    l = int(np.ceil(np.sqrt(len(PlotList))))
    c = int(np.ceil(len(PlotList) / l))
    fig3 = plt.figure('VX_Step')
    n = 0
    for i in PlotList:
        n+=1
        plt.subplot(l,c,n)
        plt.plot(T,yout[i][0], line_type, label = PlotLabel, color = color_rgb)
        plt.grid('on')
        plt.ylabel(ClosedLoops['Complete'].output_labels[i])
        plt.xlim([0,max(T)] )
        plt.xticks(np.arange(0, 21, step=4))
        if n > (l-1)*c:
            plt.xlabel('Time [s]')

    if plot_criteria:
        plt.subplot(l,c,3)
        plt.plot(np.array([Criteria['vx_risetime80']['target'] , Criteria['vx_risetime80']['target'] , Criteria['vx_risetime80']['target']+7]) , np.array([0.6, 0.8 , 0.8]) , 'k--')
        plt.text(Criteria['vx_risetime80']['target']+1 , 0.8+0.01 , 'Rise Time Criteria')
        plt.plot(np.array([0                                      , np.max(T)]) , 1+Criteria['vx_overshoot']['target']*np.array([1 , 1]) , 'k--')
        plt.text(0 , 1+Criteria['vx_overshoot']['target']+0.01 , 'Overshoot Criteria')
        plt.ylim((0, 1.3))
        
    plt.suptitle('VX Command Step')
    fig3.set_size_inches(12, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    plt.show()
    plt.subplot(l,c,n)
    if plot_legend:
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left',ncol=2)
    plt.savefig('LinearResponse_SpeedController_VX.pdf', dpi=fig3.dpi)

    # %% OPEN LOOP THETA CMD MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_thetacmd'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 15) and (P_deg[i-1] > 345)) or ((P_deg[i-1] < 15) and (P_deg[i] > 345))):
            P_deg[i] = np.nan

    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_thetacmd'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    
    fig4 = plt.figure('OpenLoop_thetacmd_MarginPlot')
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
    plt.plot(+np.array([-Criteria['openloop_thetacmd_phasemargin']['target'] , 0 , +Criteria['openloop_thetacmd_phasemargin']['target'], 0 , -Criteria['openloop_throttlecmd_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_thetacmd_gainmargin']['target'], 0 , -Criteria['openloop_thetacmd_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-80,100))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop ThetaCmd Margin')
    fig4.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='best',ncol=1)
    plt.show()
    plt.savefig('LinearResponse_AltitudeController_Bode_OpenLoop_thetacmd.pdf', dpi=fig4.dpi)

    # %% OPEN LOOP THROTTLE CMD MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_throttlecmd'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 30) and (P_deg[i-1] > 330)) or ((P_deg[i-1] < 30) and (P_deg[i] > 330))):
            P_deg[i] = np.nan

    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_throttlecmd'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    
    fig5 = plt.figure('OpenLoop_throttlecmd_MarginPlot')
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
    plt.plot(+np.array([-Criteria['openloop_throttlecmd_phasemargin']['target'] , 0 , +Criteria['openloop_throttlecmd_phasemargin']['target'], 0 , -Criteria['openloop_throttlecmd_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_throttlecmd_gainmargin']['target'], 0 , -Criteria['openloop_throttlecmd_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-80,100))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop ThrottleCmd Margin')
    fig5.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='lower left',ncol=1)
    plt.show()
    plt.savefig('LinearResponse_SpeedController_Bode_OpenLoop_throttlecmd.pdf', dpi=fig5.dpi)
    
    # %% OPEN LOOP VZ MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_VZ'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 45) and (P_deg[i-1] > 315)) or ((P_deg[i-1] < 45) and (P_deg[i] > 315))):
            P_deg[i] = np.nan

    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_VZ'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    

    fig6 = plt.figure('OpenLoop_VZ_MarginPlot')
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
    plt.plot(+np.array([-Criteria['openloop_VZ_phasemargin']['target'] , 0 , +Criteria['openloop_VZ_phasemargin']['target'], 0 , -Criteria['openloop_VZ_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_VZ_gainmargin']['target'], 0 , -Criteria['openloop_VZ_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-90,90))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop VZ Margin')
    fig6.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='best',ncol=1)
    plt.savefig('LinearResponse_AltitudeController_Bode_OpenLoop_VZ.pdf', dpi=fig6.dpi)
    plt.show()
    
    # %% OPEN LOOP Z MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_Z'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 45) and (P_deg[i-1] > 315)) or ((P_deg[i-1] < 45) and (P_deg[i] > 315))):
            P_deg[i] = np.nan
    
    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_Z'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    
    fig7 = plt.figure('OpenLoop_Z_MarginPlot')
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
    plt.plot(+np.array([-Criteria['openloop_Z_phasemargin']['target'] , 0 , +Criteria['openloop_Z_phasemargin']['target'], 0 , -Criteria['openloop_Z_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_Z_gainmargin']['target'], 0 , -Criteria['openloop_Z_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-90,90))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop Z Margin')
    fig7.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='best',ncol=1)
    plt.savefig('LinearResponse_AltitudeController_Bode_OpenLoop_Z.pdf', dpi=fig7.dpi)
    plt.show()
    
    # %% OPEN LOOP VX MARGIN PLOT
    
    G_adm,P_rad,w_radps = ct.bode(ClosedLoops['OpenLoop_VX'],omega = 10**np.linspace(BodePlot_minFreq_10e_radps,BodePlot_maxFreq_10e_radps,num=200),plot = False)
    P_deg = np.rad2deg(P_rad) 
    P_deg = P_deg%(360)
    for i in range(1,len(P_deg)):
        if (((P_deg[i] < 45) and (P_deg[i-1] > 315)) or ((P_deg[i-1] < 45) and (P_deg[i] > 315))):
            P_deg[i] = np.nan

    P_deg = P_deg-360
    G_db = 20*np.log10(G_adm)
    
    margins_res = ct.stability_margins(ClosedLoops['OpenLoop_VX'], returnall = True);
    GM_dB = 20*np.log10(margins_res[0])
    

    fig8 = plt.figure('OpenLoop_VX_MarginPlot')
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
    plt.plot(+np.array([-Criteria['openloop_VX_phasemargin']['target'] , 0 , +Criteria['openloop_VX_phasemargin']['target'], 0 , -Criteria['openloop_VX_phasemargin']['target'] ]) , 
              np.array([0 , +Criteria['openloop_VX_gainmargin']['target'], 0 , -Criteria['openloop_VX_gainmargin']['target'] , 0]) ,
                        'k--', linewidth = 2) 
    plt.xlim((-90,90))
    plt.ylim((-40,20))
    plt.suptitle('Open Loop VX Margin')
    fig8.set_size_inches(8, 6)
    plt.tight_layout(w_pad=1, h_pad=0.1,rect = (0,0,1,0.95))
    if plot_legend:
        plt.legend(loc='best',ncol=1)
    plt.savefig('LinearResponse_SpeedController_Bode_OpenLoop_VX.pdf', dpi=fig8.dpi)
    plt.show()