#ifndef _DRONESIM_SHM_OBJECT_
#define _DRONESIM_SHM_OBJECT_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

struct trace_queue
{
	enum { BUFFERSIZE = 1228800/*640*480*4*/ };

	/*trace_queue()
	  :  reading(false)
	  ,  p_end(false)
	{}*/

	//Mutex to protect access to the queue
	boost::interprocess::interprocess_mutex      mutex;

	boost::interprocess::interprocess_condition  cond;

	//Items to fill
	float buffer[BUFFERSIZE];

	//Is there any message
	bool reading;
	bool connected;
	bool p_end;
};

class ShmWriter
{
public:
	boost::interprocess::shared_memory_object shm;
    boost::interprocess::mapped_region region;
    trace_queue* data;

    ShmWriter();
    ~ShmWriter();
    void write(const float* pcd);
};

class ShmReader
{
public:
	boost::interprocess::shared_memory_object shm;
    boost::interprocess::mapped_region region;
    trace_queue* data;

    ShmReader();
    ~ShmReader();
    void read();
    void releaseMem();
};


#endif