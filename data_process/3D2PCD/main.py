# -*- coding: utf-8 -*-
"""
Created on Fri Jul 27 22:03:00 2018

@author: Administrator
"""

#Zimport os
head = ['# .PCD v0.7 - Point Cloud Data file format\n', 
        'VERSION 0.7\n', 'FIELDS x y z\n', 
        'SIZE 4 4 4\n', 'TYPE F F F\n', 
        'COUNT 1 1 1\n', 
        'WIDTH 10000\n', 
        'HEIGHT 1\n', 
        'VIEWPOINT 0 0 0 1 0 0 0\n', 
        'POINTS 10000\n', 
        'DATA ascii\n']

class Point(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        
points = []
re_points = []

filename = 'zhanban'
#读取pcd文件,从pcd的第12行开始是三维点
with open(filename+'.pcd', 'r') as f:
    for line in f.readlines()[11:len(f.readlines())-1]:
        strs = line.split(' ')
        points.append(Point(round(float(strs[0])/1000.0,6) ,round(float(strs[1])/1000.0,6),round(float(strs[2])/1000.0,6)))
    
with open(filename+'_2.pcd','w') as fw:
    for line in head:
        fw.writelines(line)
    for point in points:
        line = str(point.x) + " " + str(point.y) + " " + str(point.z) + "\n"
        fw.writelines(line)