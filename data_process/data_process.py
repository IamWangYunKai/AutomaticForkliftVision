# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 13:46:38 2018
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
        imgWidth = int(file.readline()[9:-1])
        imgHeight = int(file.readline()[10:-1])
        print('imgWidth: ', imgWidth, '\t imgHeight: ', imgHeight)
        file.readline() #not use
        while True:
            line = file.readline()
            if line:
                tmp = [float(i) for i in line.split()]
                dist.append(tmp[0])
                amp.append(tmp[1])
                x.append(tmp[2])
                y.append(tmp[3])
                z.append(tmp[4])
            else:
                break
        # read data over
        pass
    
    
if __name__ == '__main__':
    filename = 'data.txt'
    process_data(filename)
    trace = go.Scatter3d(
        x=x,
        y=y,
        z=z,
        mode='markers',
        marker=dict(
            size=2,
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
    plotly.offline.plot(fig, filename='3D-Point-Cloud.html')