#$(warning $(ORIGIN)) 
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
$(warning $(LIBs)) 



all : $(DECTAGET) $(SPLITAGET) $(LIBTAGET) $(COLORCVTTAGET)
	#export LD_LIBRARY_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))/$(LIB_ROOT):$LD_LIBRARY_PATH
$(DECTAGET):../src/NvCodec/NvDecoder/NvDecoder.cpp  
	rm -f $@
	$(CC) $(FLAG)   -c ../src/NvCodec/NvDecoder/NvDecoder.cpp -o $@  -I$(INC) -I$(CUDA_INC_DIR) -I$(FFMPEG_INC_DIR) -L$(CUDA_LIB1_DIR) -L$(CUDA_LIB2_DIR) -L$(NVDECLIB_DIR) -L$(FFMPEG_LIB_DIR) $(LIBs) 
$(SPLITAGET):../src/AppDecode/AppDec/splitMp4.cpp  
	rm -f $@
	$(CC) $(FLAG)   -c ../src/AppDecode/AppDec/splitMp4.cpp -o $@  -I$(INC) -I$(CUDA_INC_DIR) -I$(FFMPEG_INC_DIR) -L$(CUDA_LIB1_DIR) -L$(CUDA_LIB2_DIR) -L$(NVDECLIB_DIR) -L$(FFMPEG_LIB_DIR) $(LIBs)
$(COLORCVTTAGET):../src/Utils/ColorSpace.cu  
	rm -f $@
	$(NVCC) $(NVCCFLAGS) $(CUDA_ARCH)  -c ../src/Utils/ColorSpace.cu  -o $@  -I$(INC) -I$(CUDA_INC_DIR) -I$(FFMPEG_INC_DIR) -L$(CUDA_LIB1_DIR) -L$(CUDA_LIB2_DIR) -L$(NVDECLIB_DIR) -L$(FFMPEG_LIB_DIR) $(LIBs)
$(LIBTAGET):../src/AppDecode/AppDec/AppDec.cpp $(DECTAGET)  $(SPLITAGET) $(COLORCVTTAGET)
	rm -f $@
	$(CC) $(FLAG) -shared  ../src/AppDecode/AppDec/AppDec.cpp $(DECTAGET)  $(SPLITAGET) $(COLORCVTTAGET) -o $@  -I$(INC) -I$(CUDA_INC_DIR) -I$(FFMPEG_INC_DIR) -L$(CUDA_LIB1_DIR) -L$(CUDA_LIB2_DIR) -L$(NVDECLIB_DIR) -L$(FFMPEG_LIB_DIR) $(LIBs)
clean:
	rm -f *.o
	rm -f *.so
