#ifndef _DATA_PIPE_H_
#define _DATA_PIPE_H_
#include<iostream>
#include<string>
#include<vector>


template<class DataStruct>
class DataPipe
{
public:
	int queueLength_;
	int totalSize;
	int cuurentPosFlag; 
public:
	DataPipe()
	{
		cuurentPosFlag = 0;
		totalSize = -1;
	};
	DataPipe(const int&queueLength) 
	{
		cuurentPosFlag = 0;
		totalSize = -1;
		queueLength_ = queueLength;
		queue=std::vector<DataStruct>(queueLength, DataStruct()); 
	};
	~DataPipe() {}
	template<class Source>
	int initSource(Source&src) { return 0; }
	template<class Source>
	int getData(Source& src) { return 0; }
	int pushData(const DataStruct&data)
	{
		data.copyTo(queue[cuurentPosFlag]);
		if (totalSize<0)
		{
			totalSize = 2;
		}
		else
		{
			totalSize++;
		}		
		cuurentPosFlag++;
		cuurentPosFlag %= queue.size();
	}
	DataStruct& pushData2()
	{
		if (totalSize < 0)
		{
			totalSize = 2;
		}
		else
		{
			totalSize++;
		}
		cuurentPosFlag++;
		cuurentPosFlag %= queue.size(); 
		return queue[cuurentPosFlag];
	}
	template<class Source>
	int setQueueAttr( Source&anotherParam)
	{
		return 0;
	}
	DataStruct& pop()
	{
		if (totalSize >0)
		{
			if (cuurentPosFlag == 0)cuurentPosFlag = queue.size() - 1;
			else cuurentPosFlag--;
			totalSize--;
		}
		else 
		{
			cuurentPosFlag = 0;
			totalSize = -1;
		}
		return queue[0];
	}
	//template<class Source, class Outtype>
	//int pullData(const Source& src, Outtype&out) { return 0; }
	template<class Source>
	int destroySource(const Source& src) { return 0; }
	std::vector<DataStruct> queue;
private:
	int setQueueLength(const int& queueLength)
	{
		cuurentPosFlag = 0;
		totalSize = -1;
		queueLength_ = queueLength;
		queue = std::vector<DataStruct>(queueLength, DataStruct()); 
		return 0;
	}
	
};


#endif // !_DATA_PIPE_H_
