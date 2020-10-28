#!/usr/bin/env python3
import os
from shutil import copy2
from os import makedirs
from os.path import exists, dirname

def copyOver(src, dst):
	drn = dirname(dst)
	#print('Checking: ', drn)
	if not exists(drn):
		#print('Creating: ', drn)
		makedirs(drn)
	if exists(drn):
		#print("Directory exists! Copying...")
		copy2(src, dst)
		#if exists(dst):
			#print("Success!")
		#else:
			#print("Copy failed! WHY?!")
	#else:
		#print("Directory still doesn't exist for some reason.")

os.system("sudo systemctl stop openvpn-restart@actor2.service")
copyOver("openvpn@.service", "/etc/systemd/system/openvpn-restart@.service")
copyOver("PleuneNET.conf", "/etc/openvpn/actor2.conf")
os.system("sudo systemctl enable openvpn-restart@actor2.service")
os.system("sudo systemctl start openvpn-restart@actor2.service")
