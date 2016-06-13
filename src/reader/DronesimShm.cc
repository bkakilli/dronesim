
#include "DronesimShm.hh"
#define SHARED_MEMORY_UNIQUE_ID "DroneSimSharedMemory"

using namespace boost::interprocess;

ShmWriter::ShmWriter() {

	shared_memory_object::remove(SHARED_MEMORY_UNIQUE_ID);
	try{

		shm = shared_memory_object(
			create_only               //only create
			,SHARED_MEMORY_UNIQUE_ID           //name
			,read_write                //read-write mode
		);  
		//Set size
		shm.truncate(sizeof(trace_queue));

		//Map the whole shared memory in this process
		region = mapped_region(
			shm         //What to map
			,read_write  //Map it as read-write
		);

		//Construct the shared structure in memory
		//data = s_trace( (trace_queue*) region.get_address() );
		data = new (region.get_address()) trace_queue;

		data->reading = false;
		data->p_end = false;
		data->connected = false;

	}
	catch(interprocess_exception &ex){
		std::cout << ex.what() << std::endl;
		shared_memory_object::remove(SHARED_MEMORY_UNIQUE_ID); 
	}
	
}

ShmWriter::~ShmWriter() {
	shared_memory_object::remove(SHARED_MEMORY_UNIQUE_ID);
}

void ShmWriter::write(const float* pcd) {

  if(!data->reading){
    memcpy ( data->buffer, pcd, trace_queue::BUFFERSIZE*4 );
    data->cond.notify_all();
  }
	
}

ShmReader::ShmReader() {

	try{
		//Create a shared memory object.
		shm = shared_memory_object(
			open_only                    //only create
			,SHARED_MEMORY_UNIQUE_ID              //name
			,read_write                   //read-write mode
		);

		//Map the whole shared memory in this process
		region = mapped_region(
			shm                       //What to map
			,read_write //Map it as read-write
		);

		//Obtain a pointer to the shared structure
		data = static_cast<trace_queue*>(region.get_address());
	}
	catch(interprocess_exception &ex){
		std::cout << ex.what() << std::endl;
	}
	
}

ShmReader::~ShmReader() {
	
}

void ShmReader::read() {

	scoped_lock<interprocess_mutex> lock(data->mutex);
	data->cond.wait(lock);
	data->reading = true;

}

void ShmReader::releaseMem() {
	data->reading = false;
}