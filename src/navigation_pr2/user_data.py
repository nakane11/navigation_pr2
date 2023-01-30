#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import smach

def load_user_data(self):
    # jsonを読み込み
    with open(self.input_path, 'r') as f:
        json_load = json.load(f)
    ud = smach.UserData()
    for key in json_load.keys():
        ud[key] = json_load[key]
    self.userdata.update(ud)

def save_user_data(self)
    with open(self.output_path, 'w') as f:
        json.dump(self.userdata, f, indent=4)

smach.StateMachine.load_user_data = load_user_data
smach.StateMachine.save_user_data = save_user_data
        
