#Introduction

In this repository, we want to automate forklifts .

We use our industrial control board (with Windows 7 system) and RBK framework(with SLAM and multi-agent scheduling) to let them move automatically. 

We use IFM O3D303 Camera and PCL 1.8.0 (32 bit, with Boost 1.63) to identify pallets and use single line laser to identify shelves. Forklifts will avoid the shelves, and find the pallets and lift them to some designated places.

In this repository, we just pay our attention to the ***identification module*** !

# Quick Start

## Identify Pallets

```bash
cd cmake_projects/merge_independent_x86
build.bat
```



##Identify Shelves

```bash
cd cmake_projects/MeanShift_wash
make
.\MeanShift.exe
```

