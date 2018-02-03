#pragma once

#include <windows.h>

#include <vector>
#include <iostream>
#include <algorithm>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "Common.h"

/*check error code of cudaMalloc and print out if needed*/
#define safe_cuda(CODE)\
 {\
  cudaError_t err = CODE;\
  if(err != cudaSuccess) {\
    std::cout<<"CUDA error:"<<cudaGetErrorString(err)<<std::endl;\
 }\
}


template<class Key>
class SortAssistant
{
public:
	SortAssistant(const std::vector<Key> &keys) : _keys(keys) { }

	bool operator()(const unsigned int &lhs, const unsigned int &rhs)
	{
		return _keys[lhs] < _keys[rhs];
	}
private:
	const std::vector<Key> &_keys;
};

// 容器必须是vector等支持随机访问的容器
template<class Containers>
void indices_sort(const Containers &keys, std::vector<unsigned int> &indices)
{
	indices.resize(keys.size());
	for (int i = 0; i < keys.size(); ++i)
	{
		indices[i] = i;
	}
	std::sort(indices.begin(), indices.end(), SortAssistant<typename Containers::value_type>(keys));

}

// 为支持更通用的情况，由用户确保indices的索引不超过keys的大小
template<class Containers>
void remove_redundant(const Containers &keys, std::vector<unsigned int> &indices)
{
	if (indices.empty())
	{
		return;
	}

	unsigned int n = 1;
	typename Containers::value_type current = keys[indices[0]];
	for (int i = 1; i < indices.size(); ++i)
	{
		if (keys[indices[i]] != current)
		{
			indices[n++] = indices[i];
			current = keys[indices[i]];
		}
	}

	indices.resize(n);
}

// 为支持更通用的情况，由用户确保indices的索引不超过keys的大小
template<class Containers>
void filter(const Containers &input, const std::vector<unsigned int> &indices, Containers &ouput)
{
	ouput.resize(indices.size()); 
	for (int i = 0; i < indices.size(); ++i)
	{
		unsigned int index = indices[i];
		ouput[i] = input[indices[i]];
	}
}


/**
 *alloc a memory on gpu and copy data from cpu to gpu.
*/
inline void copyFromCPUtoGPU(void* *dst, void *src, int size)
{
	cudaMalloc(dst, size);
	safe_cuda(cudaMemcpy(*dst, src, size, cudaMemcpyHostToDevice));
}

/**
 *alloc a memory on cpu and copy data from gpu to cpu.
*/
inline void copyFromGPUtoCPU(void* *dst, void *src, int size)
{
	*dst = malloc(size);
	safe_cuda(cudaMemcpy(*dst, src, size, cudaMemcpyDeviceToHost));
}


class RefObject
{
public:
	RefObject()
	{
		ref_create();
	}
	RefObject(const RefObject &rhs) : _reference(rhs._reference)
	{
		++(*_reference);
	}
	RefObject &operator=(const RefObject &rhs)
	{
		ref_release();

		_reference = rhs._reference;
		++(*_reference);

		return *this;
	}

protected:
	virtual void ref_auto_clean() = 0;

	void ref_create()
	{
		_reference = new int;
		*_reference = 1;
	}

	void ref_release()
	{
		if (--(*_reference) == 0)
		{
			ref_auto_clean();
			delete _reference;
		}

		_reference = NULL;
	}

	void ref_renew()
	{
		ref_release();
		ref_create(); 
	}

private:
	int *_reference;
};


class StopWatch
{
public:
	StopWatch() : _elapsed(0)
	{
		QueryPerformanceFrequency(&_freq);
	}
	~StopWatch() { }
public:
	void start()
	{
		QueryPerformanceCounter(&_start);
	}
	void stop()
	{
		LARGE_INTEGER end;
		QueryPerformanceCounter(&end);
		_elapsed += (end.QuadPart - _start.QuadPart) * 1000000 / _freq.QuadPart;
	}
	void restart()
	{
		_elapsed = 0;
		start();
	}
	//微秒
	double elapsed()
	{
		return double(_elapsed);
	}
	//毫秒
	double elapsed_ms()
	{
		return double(_elapsed) / 1000.0;
	}
	//秒
	double elapsed_second()
	{
		return double(_elapsed) / 1000000.0;
	}

private:
	LARGE_INTEGER _freq;
	LARGE_INTEGER _start;
	sint64 _elapsed;
};

void getFPS();
