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