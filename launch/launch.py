#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_dataserver2'

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')
    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            namespace=data['generic_prop']['namespace'],
            executable="sub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "msg_filter" : data['topic_monitor']['msg_filter'], 
                    "scan_period_ms" : data['topic_monitor']['scan_period_ms'], 
                    "sampling_period_ms" : data['msg_record']['sampling_period_ms'], 
                    "dump_period_s" : data['msg_record']['dump_period_s'], 
                    "record_duration_s" : data['msg_record']['record_duration_s'], 
                    "img_threads" : data['msg_record']['img_threads'], 
                    "gnd_threads" : data['msg_record']['gnd_threads'], 
                    "dump_path" : data['msg_record']['dump_path'], 
                    "enable_control" : data['msg_record']['enable_control'], 

                    "control_enable_monitor" : data['control_setting']['enable_monitor'], 
                    "control_enable_record" : data['control_setting']['enable_record'], 
                    "control_scan_period_ms" : data['control_setting']['scan_period_ms'], 
                    "control_sampling_period_ms" : data['control_setting']['sampling_period_ms'], 
                    "control_dump_period_s" : data['control_setting']['dump_period_s'], 
                    "control_record_duration_s" : data['control_setting']['record_duration_s'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : serviceData['devInfoService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "devMultiNode" : serviceData['devMultiNode'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])