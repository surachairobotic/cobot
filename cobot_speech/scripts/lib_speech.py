#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time


###################

voice_dict = [
  {
    'value': 'save point',
    'keywords': ['set point', 'setpoint', 'safeway', 'checkpoint', 'save point', 'set my', 'set appointment']
  },
  {
    'value': 'learn mode',
    'keywords': ['learn', 'land', 'limo', 'lattimore', 'lemo', 'lyn', 'linn', 'lemon', 'longmo', 'lenmo', 'walmart', 'landmark', 'led mart', 'land mart']
  },
  {
    'value': 'pick box number one',
    'keywords': ['หยิบกล่องที่หนึ่ง', 'ที่หนึ่ง', 'ที่ 1', 'กล่องหนึ่ง', 'กล่อง 1']
  },
  {
    'value': 'pick box number two',
    'keywords': ['หยิบกล่องที่สอง', 'ที่สอง', 'ที่ 2', 'กล่องสอง', 'กล่อง 2']
  },
  {
    'value': 'pick box number one',
    'keywords': ['number one', 'number 1']
  },
  {
    'value': 'pick box number two',
    'keywords': ['number two', 'number 2', 'number to']
  },
]

SEND_INTERVAL = 5.0
t_last_send = time.time() - SEND_INTERVAL*2

def process_sentence(sen, pub):
  global voice_dict, t_last_send, SEND_INTERVAL
  
  if time.time() - SEND_INTERVAL < t_last_send:
    return False
  sen = sen.lower()
  for v in voice_dict:
    for s in v['keywords']:
      if sen.find(s)>=0:
        pub.publish(v['value'])
        print('**** FOUND : '+sen)
        t_last_send = time.time()
        return True

  return False


