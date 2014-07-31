#include "system_bindings.h"

object system_timestampToParts(long_ timestamp)
{
    // import TTimeParts
    dict locals;
    exec("from mrpt.system import TTimeParts\n"
         "ttimeparts = TTimeParts()\n",
         object(), locals);
    object ttimeparts = locals["ttimeparts"];
    long t = extract<long>(timestamp);
    TTimeParts p;
    timestampToParts(t, p);
    ttimeparts.attr("year") = p.year;
    ttimeparts.attr("month") = p.month;
    ttimeparts.attr("day") = p.day;
    ttimeparts.attr("hour") = p.hour;
    ttimeparts.attr("minute") = p.minute;
    ttimeparts.attr("second") = p.second;
    ttimeparts.attr("day_of_week") = p.day_of_week;
    ttimeparts.attr("daylight_saving") = p.daylight_saving;
    return ttimeparts;
}

long_ system_time_tToTimestamp(const double &t)
{
    return long_(time_tToTimestamp(t));
}

object TTimeStamp_to_ROS_Time(long_ timestamp)
{
    double secs = timestampTotime_t(extract<long>(timestamp));
    // import rospy.Time
    dict locals;
    locals["secs"] = secs;
    exec("import rospy\n"
         "time = rospy.Time.from_sec(secs)\n",
         object(), locals);
    return locals["time"];
}

long_ TTimeStamp_from_ROS_Time(object ros_time)
{
    return system_time_tToTimestamp(extract<double>(ros_time.attr("to_sec")()));
}

long_ mrpt_system_now()
{
    return long_(mrpt::system::getCurrentTime());
}

void export_system()
{
    // map namespace to be submodule of mrpt package
    object system_module(handle<>(borrowed(PyImport_AddModule("mrpt.system"))));
    scope().attr("system") = system_module;
    scope system_scope = system_module;

    {
        class_<TTimeParts>("TTimeParts", init<>())
            .def_readwrite("year", &TTimeParts::year)
            .def_readwrite("month", &TTimeParts::month)
            .def_readwrite("day", &TTimeParts::day)
            .def_readwrite("hour", &TTimeParts::hour)
            .def_readwrite("minute", &TTimeParts::minute)
            .def_readwrite("second", &TTimeParts::second)
            .def_readwrite("day_of_week", &TTimeParts::day_of_week)
            .def_readwrite("daylight_saving", &TTimeParts::daylight_saving)
        ;
    }

    def("timestampToParts", &system_timestampToParts);
    def("time_tToTimestamp", &system_time_tToTimestamp);
    def("now", &mrpt_system_now);
    def("TTimeStamp_from_ROS_Time", &TTimeStamp_from_ROS_Time);
    def("TTimeStamp_to_ROS_Time", &TTimeStamp_to_ROS_Time);
}
