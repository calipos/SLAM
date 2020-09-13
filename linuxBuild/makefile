#$(warning $(ORIGIN)) 
#export LD_LIBRARY_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))/$(LIB_ROOT):$LD_LIBRARY_PATH
CC = g++
FLAG= -w -std=c++11 -Wall -O3   -fPIC  -Wl,-rpath,./ 
NVCC = /usr/local/cuda/bin/nvcc
NVCCFLAGS  =   -Xcompiler -fPIC    

INC = ../src/NvCodec
CUDA_INC_DIR = /usr/local/cuda/include/
CUDA_LIB1_DIR = /usr/local/cuda/lib64/ 
CUDA_LIB2_DIR = /usr/local/cuda/targets/x86_64-linux/lib/ 
NVDECLIB_DIR = ../src/Lib/linux/stubs/x86_64  
FFMPEG_INC_DIR = ../3rdpart/ffmpeg-4.1.3/install/include
FFMPEG_LIB_DIR = ../3rdpart/ffmpeg-4.1.3/install/lib
LIBs = -lcuda -lcudart -lnvcuvid -lavcodec -lavformat -lavutil -lswresample
CUDA_ARCH := -gencode arch=compute_35,code=sm_35 -gencode arch=compute_50,code=sm_50 -gencode arch=compute_52,code=sm_52 -gencode arch=compute_61,code=sm_61
DECTAGET = ./NvDecoder.o
SPLITAGET = ./splitMp4.o
LIBTAGET = ./unreDec.so
COLORCVTTAGET = ./colorCvt.o
#$(warning $(LIBs)) 

VITUAL_LIDAR = ./vitual_lidar
VelodyneUnpack = ./velodyne_unpack.a

all : $(VITUAL_LIDAR) $(VelodyneUnpack) 
$(VITUAL_LIDAR):../VelodyneLib/virtualLidar_exe.cpp  
	rm -f $@
	$(CC) $(FLAG)   ../VelodyneLib/virtualLidar_exe.cpp   -o $@ 
$(VelodyneUnpack): ../VelodyneUnpack/calibration.cpp ../VelodyneUnpack/rawdata.cpp
	rm -f $@
	$(CC) $(FLAG)   ../VelodyneLib/virtualLidar_exe.cpp   -o $@ 
clean:
	rm -f vitual_lidar
	rm -f *.o
	rm -f *.so
