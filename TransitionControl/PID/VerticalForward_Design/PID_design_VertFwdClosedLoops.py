import numpy as np
import control as ct
from PID_design_VertFwdFunctions import gen_Gain

def gen_ClosedLoops(AircraftPitch , AltController, SpeedController, EngActuator, Sensor_vx , Sensor_vz , Sensor_z, Sensor_ax, Sensor_az):

    minus_one_gain = gen_Gain(-1,gain_name = 'minus_one_gain')

    ClosedLoops = {}
    
    
    ClosedLoops['Complete'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['AltController.VZ_ref_mps', 'AltController.Z_ref_m', 'SpeedController.VX_ref_mps'],
                                inputs  = ['VZ_ref_mps'              , 'Z_ref_m'              , 'VX_ref_mps'                ],
                                outlist = ['AircraftPitch.Z_m', 'AircraftPitch.VZ_mps', 'AircraftPitch.VX_mps', 'AircraftPitch.Theta_deg', 'AircraftPitch.Elevator_u', 
                                            'AltController.VZ_cmd_mps', 'AltController.VZ_err_int_m' , 'AltController.ThetaCmd_deg',
                                            'SpeedController.VX_err_mps', 'SpeedController.VX_err_int_m' , 'SpeedController.ThrottleCmd_u'],
                                outputs = ['Z_m', 'VZ_mps' , 'VX_mps' , 'Theta_deg', 'Elevator_u', 
                                            'VZ_cmd_mps' , 'VZ_err_int_m', 'ThetaCmd_deg',
                                            'VX_err_mps' , 'VX_err_int_m', 'ThrottleCmd_u'])
    
    ClosedLoops['NoZFeedback'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['AltController.VZ_ref_mps', 'AltController.Z_ref_m', 'SpeedController.VX_ref_mps'],
                                inputs  = ['VZ_ref_mps'              , 'Z_ref_m'              , 'VX_ref_mps'                ],
                                outlist = ['AircraftPitch.Z_m', 'AircraftPitch.VZ_mps', 'AircraftPitch.VX_mps', 'AircraftPitch.Theta_deg', 'AircraftPitch.Elevator_u', 
                                            'AltController.VZ_cmd_mps', 'AltController.VZ_err_int_m' , 'AltController.ThetaCmd_deg',
                                            'SpeedController.VX_err_mps', 'SpeedController.VX_err_int_m' , 'SpeedController.ThrottleCmd_u'],
                                outputs = ['Z_m', 'VZ_mps' , 'VX_mps' , 'Theta_deg', 'Elevator_u', 
                                            'VZ_cmd_mps' , 'VZ_err_int_m', 'ThetaCmd_deg',
                                            'VX_err_mps' , 'VX_err_int_m', 'ThrottleCmd_u'])
    
    
    ClosedLoops['Z2Z'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['AltController.Z_ref_m'],
                                inputs  = ['Z_ref_m'              ],
                                outlist = ['AircraftPitch.Z_m'],
                                outputs = ['Z_m'])
    
    ClosedLoops['VZ2VZ'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['AltController.VZ_ref_mps'],
                                inputs  = ['VZ_ref_mps'              ],
                                outlist = ['AircraftPitch.VZ_mps'],
                                outputs = ['VZ_mps'])
    
    ClosedLoops['VX2VX'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['SpeedController.VX_ref_mps'],
                                inputs  = ['VX_ref_mps'                ],
                                outlist = ['AircraftPitch.VX_mps'],
                                outputs = ['VX_mps'])
    
   
    ClosedLoops['OpenLoop_thetacmd'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS'], minus_one_gain['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['minus_one_gain.inp_name'    , 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['AircraftPitch.Theta_ref_deg'],
                                inputs  = ['ThetaCmd_deg_in'            ],
                                outlist = ['minus_one_gain.out_name'    ],
                                outputs = ['ThetaCmd_deg_out'           ])

    
    ClosedLoops['OpenLoop_throttlecmd'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS'], minus_one_gain['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['minus_one_gain.inp_name'    , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['EngActuator.ThrottleCmd_u'],
                                inputs  = ['ThrottleCmd_u_in'            ],
                                outlist = ['minus_one_gain.out_name'    ],
                                outputs = ['ThrottleCmd_u_out'           ])
    
    ClosedLoops['OpenLoop_VZ'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS'], minus_one_gain['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['minus_one_gain.inp_name'    , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['AltController.VZ_mps'],
                                inputs  = ['VZ_mps_in'            ],
                                outlist = ['minus_one_gain.out_name'    ],
                                outputs = ['VZ_mps_out'           ])
    
    ClosedLoops['OpenLoop_Z'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS'], minus_one_gain['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['minus_one_gain.inp_name'    , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['AltController.Z_m'],
                                inputs  = ['Z_m_in'            ],
                                outlist = ['minus_one_gain.out_name'    ],
                                outputs = ['Z_m_out'           ])
    
    ClosedLoops['OpenLoop_VX'] = ct.interconnect(
                                [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS'], minus_one_gain['SS']],
                                connections=[
                                              ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
                                              ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
                                              ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
                                              ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
                                              ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
                                              ['minus_one_gain.inp_name'    , 'Sensor_vx.VX_sen_mps'],
                                              ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
                                              ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
                                              ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
                                              ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
                                              ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
                                              ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
                                              ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
                                name = 'ClosedLoop' , 
                                inplist = ['SpeedController.VX_mps'],
                                inputs  = ['VX_mps_in'            ],
                                outlist = ['minus_one_gain.out_name'    ],
                                outputs = ['VX_mps_out'           ])
    
    # ClosedLoops['Complete'] = ct.interconnect(
    #                             [AircraftPitch['SS'], AltController['SS'] , SpeedController['SS'] , EngActuator['SS'], Sensor_vx['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_ax['SS'], Sensor_az['SS']],
    #                             connections=[
    #                                           ['Sensor_vx.VX_mps'           , 'AircraftPitch.VX_mps'],
    #                                           ['Sensor_vz.VZ_mps'           , 'AircraftPitch.VZ_mps'],
    #                                           ['Sensor_z.Z_m'               , 'AircraftPitch.Z_m'],
    #                                           ['Sensor_ax.AXi_mps2'         , 'AircraftPitch.Nxi_mps2'],
    #                                           ['Sensor_az.AZi_mps2'         , 'AircraftPitch.Nzi_mps2'],
    #                                           ['SpeedController.VX_mps'     , 'Sensor_vx.VX_sen_mps'],
    #                                           ['SpeedController.AXi_mps2'   , 'Sensor_ax.AXi_sen_mps2'],
    #                                           ['AltController.VZ_mps'       , 'Sensor_vz.VZ_sen_mps'],
    #                                           ['AltController.Z_m'          , 'Sensor_z.Z_sen_m'],
    #                                           ['AltController.AZi_mps2'     , 'Sensor_az.AZi_sen_mps2'],
    #                                           ['EngActuator.ThrottleCmd_u'  , 'SpeedController.ThrottleCmd_u'],
    #                                           ['AircraftPitch.Throttle_inp' , 'EngActuator.Throttle_u'],
    #                                           ['AircraftPitch.Theta_ref_deg', 'AltController.ThetaCmd_deg']],
    #                             name = 'ClosedLoop' , 
    #                             inplist = ['AircraftPitch.Theta_ref_deg'],
    #                             inputs  = ['ThetaCmd_deg_in'            ],
    #                             outlist = ['minus_one_gain.out_name'    ],
    #                             outputs = ['ThetaCmd_deg_out'           ])


    
    return ClosedLoops
