# -*- coding: utf-8 -*-
"""
Created on Thu Aug 23 21:53:09 2018

@author: Administrator
"""
import math

head = ['# .PCD v0.7 - Point Cloud Data file format\n', 
        'VERSION 0.7\n', 'FIELDS x y z\n', 
        'SIZE 4 4 4\n', 'TYPE F F F\n', 
        'COUNT 1 1 1\n', 
        'WIDTH 513\n', 
        'HEIGHT 1\n', 
        'VIEWPOINT 0 0 0 1 0 0 0\n', 
        'POINTS 513\n', 
        'DATA ascii\n']

fileindex = 0
new_file = open('1.pcd', 'w')        
with open('LaserData.txt', 'r') as file:
    for line in file.readlines():
        if(line == 'Laser Data\n'):
            new_file.close()
            fileindex += 1
            new_file = open(str(fileindex)+'.pcd', 'w')
            new_file.writelines(head)
        else:
            [theta, dist] = line.split()
            theta = float(theta)
            dist = float(dist)
            if(dist == 0):
                continue
            x = dist * math.cos(math.pi * theta / 180.0)
            y = dist * math.sin(math.pi * theta / 180.0)
            z = 0.0
            new_line = str(x) + '\t' + str(y) + '\t' + str(z) + '\n'
            new_file.writelines(new_line)
new_file.close()