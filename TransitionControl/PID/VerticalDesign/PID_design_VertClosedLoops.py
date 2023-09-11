import numpy as np
import control as ct


def gen_ClosedLoops(Aircraft , Controller, Sensor_vz , Sensor_z , Sensor_az , EngActuator):
    ClosedLoops = {}
    ClosedLoops['Complete'] = ct.interconnect(
                                [Aircraft['PitchNotIncluded']['SS'], Controller['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_az['SS'] , EngActuator['SS']],
                                connections=[
                                             ['Sensor_vz.VZ_mps'          , 'Aircraft.VZ_mps'],
                                             ['Sensor_z.Z_m'              , 'Aircraft.Z_m'],
                                             ['Sensor_az.AZi_mps2'        , 'Aircraft.Nzi_mps2'],
                                             ['Controller.VZ_mps'         , 'Sensor_vz.VZ_sen_mps'],
                                             ['Controller.Z_m'            , 'Sensor_z.Z_sen_m'],
                                             ['Controller.AZi_mps2'       , 'Sensor_az.AZi_sen_mps2'],
                                             ['EngActuator.ThrottleCmd_u' , 'Controller.ThrottleCmd'],
                                             ['Aircraft.Throttle'         , 'EngActuator.Throttle_u']],
                                name = 'ClosedLoop' , 
                                inplist = ['Controller.VZ_ref_mps', 'Controller.Z_ref_m', 'Controller.ThrottleCmd_inp'],
                                inputs = ['VZ_ref_mps', 'Z_ref_m', 'ThrottleCmd_inp'],
                                outlist = ['Aircraft.Z_m', 'Aircraft.VZ_mps', 
                                           'Controller.VZ_cmd_mps', 'Controller.VZ_err_int_m' , 
                                           'Controller.ThrottleCmd'],
                                outputs = ['Z_m', 'VZ_mps' , 
                                           'VZ_cmd_mps' , 'VZ_err_int_m', 
                                           'ThrottleCmd_u'])
    
    ClosedLoops['Z2Z'] = ct.interconnect(
                                [Aircraft['PitchNotIncluded']['SS'], Controller['SS'] , Sensor_vz['SS'] , Sensor_z['SS'] , Sensor_az['SS'] , EngActuator['SS']],
                                connections=[
                                             ['Sensor_vz.VZ_mps'          , 'Aircraft.VZ_mps'],
                                             ['Sensor_z.Z_m'              , 'Aircraft.Z_m'],
                                             ['Sensor_az.AZi_mps2'        , 'Aircraft.Nzi_mps2'],
                                             ['Controller.VZ_mps'         , 'Sensor_vz.VZ_sen_mps'],
                                             ['Controller.Z_m'            , 'Sensor_z.Z_sen_m'],
                                             ['Controller.AZi_mps2'       , 'Sensor_az.AZi_sen_mps2'],
                                             ['EngActuator.ThrottleCmd_u' , 'Controller.ThrottleCmd'],
                                             ['Aircraft.Throttle'         , 'EngActuator.Throttle_u']],
                                name = 'ClosedLoop_Z2Z' , 
                                inplist = ['Controller.Z_ref_m'],
                                inputs = ['Z_ref_m'],
                                outlist = ['Aircraft.Z_m'],
                                outputs = ['Z_m'])
    
    ClosedLoops['Only_VZ'] = ct.interconnect(
                                [Aircraft['PitchNotIncluded']['SS'], Controller['SS'] , Sensor_vz['SS'] , Sensor_az['SS'] , EngActuator['SS']],
                                connections=[
                                             ['Sensor_vz.VZ_mps'          , 'Aircraft.VZ_mps'],
                                             ['Sensor_az.AZi_mps2'        , 'Aircraft.Nzi_mps2'],
                                             ['Controller.VZ_mps'         , 'Sensor_vz.VZ_sen_mps'],
                                             ['Controller.AZi_mps2'       , 'Sensor_az.AZi_sen_mps2'],
                                             ['EngActuator.ThrottleCmd_u' , 'Controller.ThrottleCmd'],
                                             ['Aircraft.Throttle'         , 'EngActuator.Throttle_u']],
                                name = 'ClosedLoop_only_VZ' , 
                                inplist = ['Controller.VZ_ref_mps'],
                                inputs = ['VZ_ref_mps'],
                                outlist = ['Aircraft.VZ_mps'],
                                outputs = ['VZ_mps'])
    
    ClosedLoops['Open_Z'] = ct.interconnect(
                                [Aircraft['PitchNotIncluded']['SS'], Controller['SS'] , Sensor_vz['SS'] , Sensor_az['SS'] , EngActuator['SS']],
                                connections=[
                                             ['Sensor_vz.VZ_mps'          , 'Aircraft.VZ_mps'],
                                             ['Sensor_az.AZi_mps2'        , 'Aircraft.Nzi_mps2'],
                                             ['Controller.VZ_mps'         , 'Sensor_vz.VZ_sen_mps'],
                                             ['Controller.AZi_mps2'       , 'Sensor_az.AZi_sen_mps2'],
                                             ['EngActuator.ThrottleCmd_u' , 'Controller.ThrottleCmd'],
                                             ['Aircraft.Throttle'         , 'EngActuator.Throttle_u']],
                                name = 'ClosedLoop_open_Z' , 
                                inplist = ['Controller.VZ_ref_mps', 'Controller.Z_ref_m', 'Controller.ThrottleCmd_inp'],
                                inputs = ['VZ_ref_mps', 'Z_ref_m', 'ThrottleCmd_inp'],
                                outlist = ['Aircraft.Z_m', 'Aircraft.VZ_mps', 
                                           'Controller.VZ_cmd_mps', 'Controller.VZ_err_int_m' , 
                                           'Controller.ThrottleCmd'],
                                outputs = ['Z_m', 'VZ_mps' , 
                                           'VZ_cmd_mps' , 'VZ_err_int_m', 
                                           'ThrottleCmd_u'])

    
    return ClosedLoops
