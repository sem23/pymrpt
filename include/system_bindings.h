#ifndef __SYSTEM_BINDINGS_H__
#define __SYSTEM_BINDINGS_H__

/* MRPT */
#include <mrpt/system/datetime.h>

/* BOOST */
#include <boost/python.hpp>

using namespace boost::python;
using namespace mrpt::system;

// time conversion
object TTimeStamp_to_ROS_Time(long_ timestamp);
long_ TTimeStamp_from_ROS_Time(object ros_time);

// exporter
void export_system();

#endif
