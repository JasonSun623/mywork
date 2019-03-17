#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 16 21:50:51 2019

@author: dongho
"""

import logging
logger = logging.getLogger('myapp')
hdlr = logging.FileHandler('log_line/log.txt')
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
hdlr.setFormatter(formatter)
logger.addHandler(hdlr) 
logger.setLevel(logging.WARNING)
while True:
    logger.info('While this is just chatty')