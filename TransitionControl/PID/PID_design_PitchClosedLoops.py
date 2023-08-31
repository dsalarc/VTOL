import numpy as np
import control as ct


def PitchClosedLoops(Aircraft , Control, Sensor_q , Sensor_t , EngActuator, ElevActuator, ControlAllocation):
    ClosedLoops = {}
    ClosedLoops['Complete'] = ct.interconnect(
                                [Aircraft['SpeedIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS']  , ElevActuator['SS']  , ControlAllocation['SS'] ],
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
    
    ClosedLoops['T2T'] = ct.interconnect(
                                [Aircraft['SpeedIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS'] , ElevActuator['SS']  , ControlAllocation['SS'] ],
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
    
    ClosedLoops['OnlyQ']  = ct.interconnect(
                                [Aircraft['SpeedIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS'] , ElevActuator['SS']  , ControlAllocation['SS'] ],
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
    
    ClosedLoops['ThetaOpen']  = ct.interconnect(
                                [Aircraft['SpeedIncluded']['SS'], Control['SS'] , Sensor_q['SS'] , Sensor_t['SS'] , EngActuator['SS'] , ElevActuator['SS']  , ControlAllocation['SS'] ],
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
