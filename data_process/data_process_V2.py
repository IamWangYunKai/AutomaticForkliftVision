# -*- coding: utf-8 -*-
"""
Created on Fri Jul 27 20:16:01 2018
Process raw data from o3d303 camera
@author: Wang Yunkai

"""

import plotly
import plotly.graph_objs as go


#data information
dist = []
amp = []
x = []
y = []
z = []

def process_data(filename):
    with open(filename,'r') as file:
        for i in range(11):
            file.readline() #not use
        while True:
            line = file.readline()
            if line:
                tmp = [float(i) for i in line.split()]
                x.append(tmp[0])
                y.append(tmp[1])
                z.append(tmp[2])
                dist.append(tmp[3])
                amp.append(tmp[4])
            else:
                break
        # read data over
        for i in range(len(dist)):
            if z[i] > 2.0:
                x[i] = y[i] = z[i] = 0
    
def show_img():
    trace = go.Scatter3d(
        x=x,
        y=y,
        z=z,
        mode='markers',
        marker=dict(
            size=3,
            color=z,                # set color to an array/list of desired values
            colorscale='Viridis',   # choose a colorscale
            opacity=0.6,            #不透明度
            showscale =True
        )
    )

    data = [trace]
    layout = go.Layout(
        margin=dict(
            l=0,
            r=0,
            b=0,
            t=0
        )
    )
    fig = go.Figure(data=data, layout=layout)
    plotly.offline.plot(fig, filename='3D-Point-Cloud-V1.html')
    
if __name__ == '__main__':
    filename = 'data.pcd'
    process_data(filename)
    show_img()