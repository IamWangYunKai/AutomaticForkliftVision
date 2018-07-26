# PCS
Point Cloud Segmentation for O3D303 Camera

Use cmake 3.10.0 & VS2015 32bit 

# Quick Start

Use CMake GUI,

Choose your source code path:  PCS\SDK\code

Choose your build path:  PCS\SDK\build

Click "Configure" , choose VS2015 32 bit compiler (Not 64 bit ! ) and choose only "O#D#XX_SAMPLE_CODES_BUILD_ALL"

Then generate it and open this project in VS2015

Find sample.cpp in SampleO3D3xxCamera and change your camera's Ip adrdess, for example:

```C
#define SOURCE_PARAM "169.254.130.223:80:50010"
```

Compile all projects and find PCS\SDK\build\bin\Release\LightVis\LightVis.exe

Change your camera IP in lv.cfg (in the same path with LightVis.exe) and then just open LightVis.exe !