
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "datapipe.h"
std::mutex mtx; // ȫ���໥�ų���.
std::condition_variable cv; // ȫ����������.
bool ready = true; // ȫ�ֱ�־λ.


struct test_source
{};
struct test_data
{
	std::vector<char> data;
	test_data()
	{
		data.resize(1208);
	}
};

template<>
template<>
int DataPipe<test_data>::initSource<test_source>(test_source&src)
{
	return 0;
}
template<>
template<>
int DataPipe<test_data>::getData<test_source>(test_source& src)
{
	for (int i = 0;  ; i++)
	{
		{
			std::unique_lock <std::mutex> lck(mtx);
			while (!ready) // �����־λ��Ϊ true, ��ȴ�...
				cv.wait(lck); // 
		}
		std::cout<<i<<std::endl;
		if (i>1e6)
		{
			return 0;
		}
	}
	return 0;
}
int ThreadProc1()
{
	DataPipe<test_data> test_pipe(10);
	test_source testSrc;
	test_pipe.initSource(testSrc);
	test_pipe.getData(testSrc);
	return 0;
}

int main()
{
	std::thread t1(ThreadProc1); 

	std::this_thread::sleep_for(std::chrono::seconds(1));
	{
		std::unique_lock <std::mutex> lck(mtx);
		ready = false; // ����ȫ�ֱ�־λΪ true.
		//cv.notify_all(); // ����ȫ���߳�.
	}
	std::this_thread::sleep_for(std::chrono::seconds(1));
	{
		std::unique_lock <std::mutex> lck(mtx);
		ready = true; // ����ȫ�ֱ�־λΪ true.
		cv.notify_all(); // ����ȫ���߳�.
	}
	std::this_thread::sleep_for(std::chrono::seconds(1));
	{
		std::unique_lock <std::mutex> lck(mtx);
		ready = false; // ����ȫ�ֱ�־λΪ true.
		//cv.notify_all(); // ����ȫ���߳�.
	}
	std::this_thread::sleep_for(std::chrono::seconds(1));
	{
		std::unique_lock <std::mutex> lck(mtx);
		ready = true; // ����ȫ�ֱ�־λΪ true.
		cv.notify_all(); // ����ȫ���߳�.
	}
	std::this_thread::sleep_for(std::chrono::seconds(3));
	t1.join(); 

	system("pause");
	return 0;
}
