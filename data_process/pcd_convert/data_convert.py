# -*- coding: utf-8 -*-
"""
Created on Fri Jul 27 16:35:42 2018

@author: Administrator
"""
import os
head = ['# .PCD v0.7 - Point Cloud Data file format\n', 
        'VERSION 0.7\n', 'FIELDS x y z\n', 
        'SIZE 4 4 4\n', 'TYPE F F F\n', 
        'COUNT 1 1 1\n', 
        'WIDTH 13704\n', 
        'HEIGHT 1\n', 
        'VIEWPOINT 0 0 0 1 0 0 0\n', 
        'POINTS 13704\n', 
        'DATA ascii\n']

class Point(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        
points = []
re_points = []

filename = 'data'
#读取pcd文件,从pcd的第12行开始是三维点
with open(filename+'.pcd', 'r') as f:
    for line in f.readlines()[11:len(f.readlines())-1]:
        strs = line.split(' ')
        points.append(Point(strs[0],strs[1],strs[2].strip()))

with open(filename+'.txt','w') as fw:
    for i in range(len(points)):
         linev = points[i].x+" "+points[i].y+" "+points[i].z+"\n"
         fw.writelines(linev)
         
with open(filename+'.txt','r') as fr:
    for line in fr.readlines():
        tmp = [float(i) for i in line.split()]
        re_points.append(Point(tmp[0], tmp[1], tmp[2]))
        
with open(filename+'_2.pcd','w') as fw:
    for line in head:
        fw.writelines(line)
    for point in re_points:
        line = str(point.x) + " " + str(point.y) + " " + str(point.z) + "\n"
        fw.writelines(line)