#!/usr/bin/env python3

import os, sys
from ruamel.yaml import YAML

def create_human_yaml(pkg_path, l_upper_trunk, l_upper_arm, l_forearm, l_hand):
    # pkg_path = '/home/gizem/catkin_ws/src/dh_game/'
    print(pkg_path)
    yaml_filename = '/config/human.yaml'
    with open(pkg_path+yaml_filename, 'w') as fp:
        fp.truncate()
        fp.write('l_upper_trunk: '+str(l_upper_trunk)+os.linesep)
        fp.write('l_upper_arm: '+str(l_upper_arm)+os.linesep)
        fp.write('l_forearm: '+str(l_forearm)+os.linesep)
        fp.write('l_hand: '+str(l_hand)+os.linesep)
    fp.close()


def create_sensors_yaml(pkg_path, chest, l_upper_arm, l_forearm, l_hand, r_upper_arm, r_forearm, emg):
    yaml_filename = '/config/sensors.yaml'
    with open(pkg_path+yaml_filename, 'w') as fp:
        fp.truncate()
        fp.write('chest: '+str(chest)+os.linesep)
        fp.write('l_upper_arm: '+str(l_upper_arm)+os.linesep)
        fp.write('l_forearm: '+str(l_forearm)+os.linesep)
        fp.write('l_hand: '+str(l_hand)+os.linesep)
        fp.write('r_upper_arm: '+str(r_upper_arm)+os.linesep)
        fp.write('r_forearm: '+str(r_forearm)+os.linesep)
        fp.write('emg: '+str(emg)+os.linesep)
    fp.close()
    
    


if __name__ == "__main__":
    # create_human_yaml()
    pass