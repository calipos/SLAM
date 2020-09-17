#ifndef _DATA_PIPE_H_
#define _DATA_PIPE_H_
#include<iostream>
#include<string>
#include<vector>
#include<mutex>
#include "../logger/logger.h"

template<class DataStruct>
class DataPipe
{
private:
	std::mutex pushPopLck;
	std::vector<uint8_t> emptyFlag;
	int pushPos;
	int popPos;
public:
	int queueLength_;
public:
	DataPipe()
	{
		pushPos = 0;
		popPos = 0;  
	};
	DataPipe(const int&queueLength) 
	{
		CHECK(queueLength > 0);
		pushPos = 0;
		popPos = 0;
		queueLength_ = queueLength;
		queue=std::vector<DataStruct>(queueLength, DataStruct()); 
		emptyFlag = std::vector<uint8_t>(queueLength_, 0);
	};
	~DataPipe() {}
	template<class Source>
	int initSource(Source&src) { return 0; }
	template<class Source>
	int getData(Source& src) { return 0; }
	int pushData(const DataStruct&data)
	{
		std::lock_guard<std::mutex> lck(this->pushPopLck);
		if (emptyFlag[pushPos]==0)
		{
			data.copyTo(queue[pushPos]);
			emptyFlag[pushPos] = 1;
			pushPos++;
			pushPos %= queueLength_;
			return 0;
		}
		else
		{
			return -1;
		}
	}
	template<class Fun,class Arg>
	int pushData(const Fun&f, Arg& param)
	{
		std::lock_guard<std::mutex> lck(this->pushPopLck);
		if (emptyFlag[pushPos] == 0)
		{
			DataStruct* mem = &queue[pushPos];
			int ret = f(param, mem);
			if (ret == 0)
			{
				emptyFlag[pushPos] = 1;
				pushPos++;
				pushPos %= queueLength_;
				return 0;
			}
			else return -2;
		}
		else
		{
			return -1;
		}
	}	
	template<class Source>
	int setQueueAttr( Source&anotherParam)
	{
		return 0;
	}
	int pop(DataStruct*mem)
	{
		std::lock_guard<std::mutex> lck(this->pushPopLck);
		if (emptyFlag[popPos] > 0&& mem!=NULL)
		{
			queue[popPos].copyTo(*mem);
			emptyFlag[popPos] = 0;
			popPos++; 
			popPos %= queueLength_;
		}
		else
		{
			return -1;
		}
	}
	//template<class Source, class Outtype>
	//int pullData(const Source& src, Outtype&out) { return 0; }
	template<class Source>
	int destroySource(const Source& src) { return 0; }
	std::vector<DataStruct> queue;
private:
	int setQueueLength(const int& queueLength)
	{
		CHECK(queueLength > 0);
		pushPos = 0;
		popPos = 0;
		queueLength_ = queueLength;
		queue = std::vector<DataStruct>(queueLength, DataStruct());
		emptyFlag = std::vector<uint8_t>(queueLength_, 0);
		return 0;
	}
	
};


#endif // !_DATA_PIPE_H_
