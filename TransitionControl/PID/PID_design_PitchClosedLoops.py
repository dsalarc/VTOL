import numpy as np
import control as ct
from PID_design_PitchFunctions import gen_Gain

def PitchClosedLoops(Aircraft , Control, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation):
    
    minus_one_gain = gen_Gain(-1,gain_name = 'minus_one_gain')
    
    ClosedLoops = {}
    ClosedLoops['AltitudeIncluded'] = ct.interconnect(
                                [Aircraft['AltitudeIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS']  , ElevActuator['SS']  , ControlAllocation['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['Control.Theta_deg'  , 'Sensor_t.Theta_sen_deg'],
                                             ['ControlAllocation.PitchCmd' , 'Control.PitchCmd'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u']],
                                name = 'ClosedLoop' , 
                                inplist = ['Control.Q_ref_degps', 'Control.Theta_ref_deg', 'Control.PitchCmd_inp', 'Aircraft.Throttle'],
                                inputs = ['Q_ref_degps', 'Theta_ref_deg', 'PitchCmd_inp', 'Throttle_inp'],
                                outlist = ['Aircraft.Theta_deg', 'Aircraft.Q_degps', 
                                           'Control.Q_cmd_degps', 'Control.Q_err_int_deg' , 
                                           'Control.PitchCmd', 
                                           'ControlAllocation.ThrottlePitch' , 'ControlAllocation.Elevator_u',
                                           'Aircraft.Z_m'],
                                outputs = ['Theta_deg', 'Q_degps' , 
                                           'Q_cmd_degps' , 'Q_err_int_deg', 
                                           'PitchCmd_u' , 
                                           'ThrottlePitchCmd_u', 'ElevatorCmd_u', 'Z_m'])
    
    ClosedLoops['AltitudeNotIncluded'] = ct.interconnect(
                                [Aircraft['AltitudeNotIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS']  , ElevActuator['SS']  , ControlAllocation['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['Control.Theta_deg'  , 'Sensor_t.Theta_sen_deg'],
                                             ['ControlAllocation.PitchCmd' , 'Control.PitchCmd'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u']],
                                name = 'ClosedLoop' , 
                                inplist = ['Control.Q_ref_degps', 'Control.Theta_ref_deg', 'Control.PitchCmd_inp'],
                                inputs = ['Q_ref_degps', 'Theta_ref_deg', 'PitchCmd_inp'],
                                outlist = ['Aircraft.Theta_deg', 'Aircraft.Q_degps', 
                                           'Control.Q_cmd_degps', 'Control.Q_err_int_deg' , 
                                           'Control.PitchCmd', 
                                           'ControlAllocation.ThrottlePitch' , 'ControlAllocation.Elevator_u'],
                                outputs = ['Theta_deg', 'Q_degps' , 
                                           'Q_cmd_degps' , 'Q_err_int_deg', 
                                           'PitchCmd_u' , 
                                           'ThrottlePitchCmd_u', 'ElevatorCmd_u'])
        
    ClosedLoops['OpenLoop_pitchcmd'] = ct.interconnect(
                                [Aircraft['AltitudeNotIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS']  , ElevActuator['SS']  , ControlAllocation['SS'] , minus_one_gain['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['Control.Theta_deg'  , 'Sensor_t.Theta_sen_deg'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u'] , 
                                             ['minus_one_gain.inp_name'   , 'Control.PitchCmd']],
                                name = 'OpenLoop_pitchcmd' , 
                                inplist = ['ControlAllocation.PitchCmd'],
                                inputs = ['PitchCmd_in'],
                                outlist = ['minus_one_gain.out_name'],
                                outputs = ['PitchCmd_out'])
    
    ClosedLoops['OpenLoop_q'] = ct.interconnect(
                                [Aircraft['AltitudeNotIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS']  , ElevActuator['SS']  , ControlAllocation['SS'] , minus_one_gain['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['Control.Theta_deg'  , 'Sensor_t.Theta_sen_deg'],
                                             ['ControlAllocation.PitchCmd' , 'Control.PitchCmd'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u'],
                                             ['minus_one_gain.inp_name'   , 'Sensor_q.Q_sen_degps']],
                                name = 'OpenLoop_q' , 
                                inplist = ['Control.Q_degps'],
                                inputs = ['Q_degps_in'],
                                outlist = ['minus_one_gain.out_name'],
                                outputs = ['Q_degps_out'])
    
    ClosedLoops['OpenLoop_theta'] = ct.interconnect(
                                [Aircraft['AltitudeNotIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS']  , ElevActuator['SS']  , ControlAllocation['SS'] , minus_one_gain['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['ControlAllocation.PitchCmd' , 'Control.PitchCmd'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u'],
                                             ['minus_one_gain.inp_name'   , 'Sensor_t.Theta_sen_deg']],
                                name = 'OpenLoop_theta' , 
                                inplist = ['Control.Theta_deg'],
                                inputs = ['Theta_deg_in'],
                                outlist = ['minus_one_gain.out_name'],
                                outputs = ['Theta_deg_out'])
    
    ClosedLoops['T2T'] = ct.interconnect(
                                [Aircraft['AltitudeNotIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS'] , ElevActuator['SS']  , ControlAllocation['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['Control.Theta_deg'  , 'Sensor_t.Theta_sen_deg'],
                                             ['ControlAllocation.PitchCmd' , 'Control.PitchCmd'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u']],
                                name = 'ClosedLoop_only_q' , 
                                inplist = ['Control.Theta_ref_deg'],
                                inputs = ['Theta_ref_deg'],
                                outlist = ['Aircraft.Theta_deg'],
                                outputs = ['Theta_deg'])
    
    ClosedLoops['Q2Q']  = ct.interconnect(
                                [Aircraft['AltitudeNotIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS'] , ElevActuator['SS']  , ControlAllocation['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['ControlAllocation.PitchCmd' , 'Control.PitchCmd'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u']],
                                name = 'ClosedLoop_only_q' , 
                                inplist = ['Control.Q_ref_degps'],
                                inputs = ['Q_ref_degps'],
                                outlist = ['Aircraft.Q_degps'],
                                outputs = ['Q_degps'])
    
    ClosedLoops['NoThetaFeedback']  = ct.interconnect(
                                [Aircraft['AltitudeNotIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS'] , ElevActuator['SS']  , ControlAllocation['SS'] ],
                                connections=[
                                             ['Sensor_q.Q_degps'   , 'Aircraft.Q_degps'],
                                             ['Control.Q_degps'    , 'Sensor_q.Q_sen_degps'],
                                             ['Sensor_t.Theta_deg' , 'Aircraft.Theta_deg'],
                                             ['ControlAllocation.PitchCmd' , 'Control.PitchCmd'],
                                             ['EngActuator.PitchCmd_u'     , 'ControlAllocation.ThrottlePitch'],
                                             ['ElevActuator.ElevatorCmd_u'    , 'ControlAllocation.Elevator_u'],
                                             ['Aircraft.PitchThrottle'     , 'EngActuator.PitchThrottle_u'],
                                             ['Aircraft.W2_Elevator'       , 'ElevActuator.Elevator_u']],
                                name = 'ClosedLoop_theta_open' , 
                                inplist = ['Control.Q_ref_degps', 'Control.Theta_ref_deg', 'Control.PitchCmd_inp'],
                                inputs = ['Q_ref_degps', 'Theta_ref_deg', 'PitchCmd_inp'],
                                outlist = ['Aircraft.Theta_deg', 'Aircraft.Q_degps', 'Control.PitchCmd', 'Control.Q_cmd_degps', 'Control.Q_err_int_deg'],
                                outputs = ['Theta_deg', 'Q_degps' , 'PitchCmd_u' , 'Q_cmd_degps' , 'Q_err_int_deg'])
    
    return ClosedLoops
