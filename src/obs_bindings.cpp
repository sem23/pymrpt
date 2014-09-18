/* MRPT */
#include <mrpt/slam/CAction.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CActionRobotMovement3D.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationRange.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>

#include <mrpt/slam/CSimpleMap.h>


/* BOOST */
#include <boost/python.hpp>

#include "bindings.h"
#include "system_bindings.h"
#include "utils_bindings.h"
#include "obs_bindings.h"

/* STD */
#include <stdint.h>

using namespace boost::python;

using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::utils;


// CActionCollection
void CActionCollection_insert1(CActionCollection &self, CActionRobotMovement2D &action)
{
    self.insert(action);
}

void CActionCollection_insert2(CActionCollection &self, CActionRobotMovement3D &action)
{
    self.insert(action);
}
// end of CActionCollection


// CAction
struct CActionWrap : CAction, wrapper<CAction>
{
    CObject *duplicate() const
    {
        return this->get_override("duplicate")();
    }
};

// CObservation
struct CObservationWrap : CObservation, wrapper<CObservation>
{
    CObject *duplicate() const
    {
        return this->get_override("duplicate")();
    }

    void writeToStream(mrpt::utils::CStream& stream, int32_t* pos) const
    {
        this->get_override("writeToStream")(stream, pos);
    }

    void readFromStream(mrpt::utils::CStream& stream, int32_t pos)
    {
        this->get_override("readFromStream")(stream, pos);
    }

    void getSensorPose(CPose3D &out_sensorPose) const
    {
        this->get_override("getSensorPose")(out_sensorPose);
    }

    void setSensorPose(const CPose3D &newSensorPose)
    {
        this->get_override("setSensorPose")(newSensorPose);
    }
};

long_ CObservation_get_timestamp(CObservation &self)
{
    return long_(self.timestamp);
}

void CObservation_set_timestamp(CObservation &self, long_ timestamp)
{
    self.timestamp = extract<uint64_t>(timestamp);
}
// end of CObservation

// CObservationOdometry
object CObservationOdometry_to_ROS_RawOdometry_msg(CObservationOdometry &self, str frame_id)
{
    // import msg
    dict locals;
    exec("from pymrpt.msg import RawOdometry\n"
         "raw_odometry_msg = RawOdometry()\n",
         object(), locals);
    object raw_odometry_msg = locals["raw_odometry_msg"];
    // set info
    raw_odometry_msg.attr("header").attr("frame_id") = frame_id;
    raw_odometry_msg.attr("header").attr("stamp") = TTimeStamp_to_ROS_Time(long_(self.timestamp));
    raw_odometry_msg.attr("has_encoders_info") = self.hasEncodersInfo;
    raw_odometry_msg.attr("has_velocities") = self.hasVelocities;
    // set data
    raw_odometry_msg.attr("encoder_left_ticks") = self.encoderLeftTicks;
    raw_odometry_msg.attr("encoder_right_ticks") = self.encoderRightTicks;
    raw_odometry_msg.attr("velocity_lin") = self.velocityLin;
    raw_odometry_msg.attr("velocity_ang") = self.velocityAng;
    return raw_odometry_msg;
}

void CObservationOdometry_from_ROS_RawOdometry_msg(CObservationOdometry &self, object raw_odometry_msg)
{
    // set info
    self.sensorLabel = extract<std::string>(raw_odometry_msg.attr("header").attr("frame_id"));
    self.timestamp = extract<uint64_t>(TTimeStamp_from_ROS_Time(raw_odometry_msg.attr("header").attr("stamp")));
    self.hasEncodersInfo = extract<bool>(raw_odometry_msg.attr("has_encoders_info"));
    self.hasVelocities = extract<bool>(raw_odometry_msg.attr("has_velocities"));
    // set data
    self.encoderLeftTicks = extract<int>(raw_odometry_msg.attr("encoder_left_ticks"));
    self.encoderRightTicks = extract<int>(raw_odometry_msg.attr("encoder_right_ticks"));
    self.velocityLin = extract<float>(raw_odometry_msg.attr("velocity_lin"));
    self.velocityAng = extract<float>(raw_odometry_msg.attr("velocity_ang"));
}
// end of CObservationOdometry


// CObservationRange
object CObservationRange_to_ROS_Range_msg(CObservationRange &self, str frame_id)
{
    // import msg
    dict locals;
    exec("from sensor_msgs.msg import Range\n"
         "range_msg = Range()\n",
         object(), locals);
    object range_msg = locals["range_msg"];
    // set info
    range_msg.attr("header").attr("frame_id") = frame_id;
    range_msg.attr("header").attr("stamp") = TTimeStamp_to_ROS_Time(long_(self.timestamp));
    range_msg.attr("min_range") = self.minSensorDistance;
    range_msg.attr("max_range") = self.maxSensorDistance;
    range_msg.attr("field_of_view") = self.sensorConeApperture;
    // set range
    range_msg.attr("range") = self.sensedData[0].sensedDistance;
    return range_msg;
}

void CObservationRange_from_ROS_Range_msg(CObservationRange &self, object range_msg)
{
    // set info
    self.sensorLabel = extract<std::string>(range_msg.attr("header").attr("frame_id"));
    self.timestamp = extract<uint64_t>(TTimeStamp_from_ROS_Time(range_msg.attr("header").attr("stamp")));
    self.minSensorDistance = extract<float>(range_msg.attr("min_range"));
    self.maxSensorDistance = extract<float>(range_msg.attr("max_range"));
    self.sensorConeApperture = extract<float>(range_msg.attr("field_of_view"));
    // set range
    CObservationRange::TMeasurement data;
    data.sensedDistance = extract<float>(range_msg.attr("range"));
    self.sensedData.clear();
    self.sensedData.push_back(data);
}
// end of CObservationRange

// CObservation2DRangeScan
object CObservation2DRangeScan_to_ROS_LaserScan_msg(CObservation2DRangeScan &self, str frame_id)
{
    // import msg
    dict locals;
    exec("from sensor_msgs.msg import LaserScan\n"
         "scan_msg = LaserScan()\n",
         object(), locals);
    object scan_msg = locals["scan_msg"];
    // set info
    scan_msg.attr("header").attr("frame_id") = frame_id;
    scan_msg.attr("header").attr("stamp") = TTimeStamp_to_ROS_Time(long_(self.timestamp));
    scan_msg.attr("range_min") = 0.0;
    scan_msg.attr("range_max") = self.maxRange;
    scan_msg.attr("angle_min") = -self.aperture / 2.0;
    scan_msg.attr("angle_max") = self.aperture / 2.0;
    scan_msg.attr("angle_increment") = self.aperture / self.scan.size();
    // set ranges (no intensities given in mrpt)
    list ranges;
    for (int i = 0; i < self.scan.size(); ++i) { ranges.append(self.scan[i]); }
    scan_msg.attr("ranges") = ranges;
    return scan_msg;
}

void CObservation2DRangeScan_from_ROS_LaserScan_msg(CObservation2DRangeScan &self, object scan_msg, CPose3D pose)
{
    // set info
    self.sensorLabel = extract<std::string>(scan_msg.attr("header").attr("frame_id"));
    self.timestamp = extract<uint64_t>(TTimeStamp_from_ROS_Time(scan_msg.attr("header").attr("stamp")));
    self.maxRange = extract<float>(scan_msg.attr("range_max"));
    self.aperture = extract<float>(scan_msg.attr("angle_max")) - extract<float>(scan_msg.attr("angle_min"));
    self.beamAperture = extract<float>(scan_msg.attr("angle_increment"));
    self.sensorPose = pose;
    // set ranges
    self.scan.clear();
    self.validRange.clear();
    list ranges = extract<list>(scan_msg.attr("ranges"));
    for (int i = 0; i < len(ranges); ++i) {
        float range = extract<float>(ranges[i]);
        self.scan.push_back(range);
        if (range < self.maxRange - 0.01)
          self.validRange.push_back('\x01');
        else
          self.validRange.push_back('\x00');
    }
}
// end of CObservation2DRangeScan

// CSimpleMap
void CSimpleMap_insert(CSimpleMap &self, CPose3DPDF &in_posePDF, CSensoryFrame &in_SF)
{
    // create smart pointers
    CPose3DPDFPtr in_posePDFPtr = (CPose3DPDFPtr) in_posePDF.duplicateGetSmartPtr();
    CSensoryFramePtr in_SFPtr = (CSensoryFramePtr) in_SF.duplicateGetSmartPtr();
    // insert smart pointers
    self.insert(in_posePDFPtr, in_SFPtr);
}
// end of CSimpleMap


// CSensoryFrame
void CSensoryFrame_insert1(CSensoryFrame &self, CObservation2DRangeScan &obs)
{
    CObservationPtr obsPtr = (CObservationPtr) obs.duplicateGetSmartPtr();
    self.insert(obsPtr);
}

void CSensoryFrame_insert2(CSensoryFrame &self, CObservation3DRangeScan &obs)
{
    CObservationPtr obsPtr = (CObservationPtr) obs.duplicateGetSmartPtr();
    self.insert(obsPtr);
}
// end of CSensoryFrame


//
// exporter
//
void export_obs()
{
    // map namespace to be submodule of mrpt package
    object obs_module(handle<>(borrowed(PyImport_AddModule("mrpt.obs"))));
    scope().attr("obs") = obs_module;
    scope obs_scope = obs_module;

    // CAction
    {
        class_<CActionWrap, boost::noncopyable>("CAction", no_init)
            .def_readwrite("timestamp", &CAction::timestamp)
        ;
    }

    // CActionCollection
    {
        class_<CActionCollection>("CActionCollection", init<>())
            .def("clear", &CActionCollection::clear, "Erase all actions from the list.")
            .def("insert", &CActionCollection::insert, "Add a new object to the list.")
            .def("size", &CActionCollection::size, "Returns the actions count in the collection.")
            .def("eraseByIndex", &CActionCollection::eraseByIndex, "Remove an action from the list by its index.")
        ;
    }

    // CActionRobotMovement2D
    {
        scope s = class_<CActionRobotMovement2D, bases<CAction> >("CActionRobotMovement2D", init<>())
            .def_readwrite("rawOdometryIncrementReading", &CActionRobotMovement2D::rawOdometryIncrementReading)
            .def_readwrite("estimationMethod", &CActionRobotMovement2D::estimationMethod)
            .def_readwrite("hasEncodersInfo", &CActionRobotMovement2D::hasEncodersInfo)
            .def_readwrite("encoderLeftTicks", &CActionRobotMovement2D::encoderLeftTicks)
            .def_readwrite("encoderRightTicks", &CActionRobotMovement2D::encoderRightTicks)
            .def_readwrite("velocityLin", &CActionRobotMovement2D::velocityLin)
            .def_readwrite("velocityAng", &CActionRobotMovement2D::velocityAng)
            .def_readwrite("motionModelConfiguration", &CActionRobotMovement2D::motionModelConfiguration)
            .def("computeFromOdometry", &CActionRobotMovement2D::computeFromOdometry, "Computes the PDF of the pose increment from an odometry reading and according to the given motion model (speed and encoder ticks information is not modified).")
            .def("computeFromEncoders", &CActionRobotMovement2D::computeFromEncoders, "If \"hasEncodersInfo\"=true, this method updates the pose estimation according to the ticks from both encoders and the passed parameters, which is passed internally to the method \"computeFromOdometry\" with the last used PDF options (or the defualt ones if not explicitly called by the user).")
        ;

        // TEstimationMethod
        enum_<CActionRobotMovement2D::TEstimationMethod>("TEstimationMethod")
            .value("emOdometry", CActionRobotMovement2D::emOdometry)
            .value("emScan2DMatching", CActionRobotMovement2D::emScan2DMatching)
        ;

        // TDrawSampleMotionModel
        enum_<CActionRobotMovement2D::TDrawSampleMotionModel>("TDrawSampleMotionModel")
            .value("mmGaussian", CActionRobotMovement2D::mmGaussian)
            .value("mmThrun", CActionRobotMovement2D::mmThrun)
        ;

        {
            // TMotionModelOptions
            scope s = class_<CActionRobotMovement2D::TMotionModelOptions>("TMotionModelOptions", init<>())
                .def_readwrite("modelSelection", &CActionRobotMovement2D::TMotionModelOptions::modelSelection)
                .def_readwrite("gausianModel", &CActionRobotMovement2D::TMotionModelOptions::gausianModel)
                .def_readwrite("thrunModel", &CActionRobotMovement2D::TMotionModelOptions::thrunModel)
            ;

            // TOptions_GaussianModel
            class_<CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel>("TOptions_GaussianModel", init<>())
                .def_readwrite("a1", &CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a1)
                .def_readwrite("a2", &CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a2)
                .def_readwrite("a3", &CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a3)
                .def_readwrite("a4", &CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::a4)
                .def_readwrite("minStdXY", &CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::minStdXY)
                .def_readwrite("minStdPHI", &CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel::minStdPHI)
            ;

            // TOptions_ThrunModel
            class_<CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel>("TOptions_ThrunModel", init<>())
                .def_readwrite("nParticlesCount", &CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::nParticlesCount)
                .def_readwrite("alfa1_rot_rot", &CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa1_rot_rot)
                .def_readwrite("alfa2_rot_trans", &CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa2_rot_trans)
                .def_readwrite("alfa3_trans_trans", &CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa3_trans_trans)
                .def_readwrite("alfa4_trans_rot", &CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::alfa4_trans_rot)
                .def_readwrite("additional_std_XY", &CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::additional_std_XY)
                .def_readwrite("additional_std_phi", &CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel::additional_std_phi)
            ;
        }
    }

    // CObservation
    {
        class_<CObservationWrap, boost::noncopyable>("CObservation", no_init)
            .add_property("timestamp", &CObservation_get_timestamp, &CObservation_set_timestamp)
            .def_readwrite("sensorLabel", &CObservation::timestamp)
            .def("getSensorPose", &CObservationWrap::getSensorPose, "A general method to retrieve the sensor pose on the robot.")
            .def("setSensorPose", &CObservationWrap::setSensorPose, "A general method to change the sensor pose on the robot.")
        ;
    }

    // CObservationOdometry
    {
        scope s = class_<CObservationOdometry, bases<CObservation> >("CObservationOdometry", init<>())
            .def_readwrite("odometry", &CObservationOdometry::odometry)
            .def_readwrite("hasEncodersInfo", &CObservationOdometry::hasEncodersInfo)
            .def_readwrite("encoderLeftTicks", &CObservationOdometry::encoderLeftTicks)
            .def_readwrite("encoderRightTicks", &CObservationOdometry::encoderRightTicks)
            .def_readwrite("hasVelocities", &CObservationOdometry::hasVelocities)
            .def_readwrite("velocityLin", &CObservationOdometry::velocityLin)
            .def_readwrite("velocityAng", &CObservationOdometry::velocityAng)
            .def("to_ROS_RawOdometry_msg", &CObservationOdometry_to_ROS_RawOdometry_msg, "Convert to ROS pymrpt_msgs/RawOdometry.")
            .def("from_ROS_RawOdometry_msg", &CObservationOdometry_from_ROS_RawOdometry_msg, "Convert from ROS pymrpt_msgs/RawOdometry.")
        ;
    }
    // CObservationRange
    {
        scope s = class_<CObservationRange, bases<CObservation> >("CObservationRange", init<>())
            .def_readwrite("minSensorDistance", &CObservationRange::minSensorDistance)
            .def_readwrite("maxSensorDistance", &CObservationRange::maxSensorDistance)
            .def_readwrite("sensorConeApperture", &CObservationRange::sensorConeApperture)
            .def_readwrite("sensedData", &CObservationRange::sensedData)
            .def("to_ROS_Range_msg", &CObservationRange_to_ROS_Range_msg, "Convert to ROS sensor_msgs/Range.")
            .def("from_ROS_Range_msg", &CObservationRange_from_ROS_Range_msg, "Convert from ROS sensor_msgs/Range.")
        ;

        // TMeasurement
        class_<CObservationRange::TMeasurement>("TMeasurement", init<>())
            .def_readwrite("sensorID", &CObservationRange::TMeasurement::sensorID)
            .def_readwrite("sensorPose", &CObservationRange::TMeasurement::sensorPose)
            .def_readwrite("sensedDistance", &CObservationRange::TMeasurement::sensedDistance)
        ;

        // TMeasurementList
        class_<CObservationRange::TMeasurementList>("TMeasurementList", init<>())
            .def("__len__", &CObservationRange::TMeasurementList::size)
            .def("clear", &CObservationRange::TMeasurementList::clear)
            .def("append", &stl_deque<CObservationRange::TMeasurementList>::add, with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &stl_deque<CObservationRange::TMeasurementList>::get, return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &stl_deque<CObservationRange::TMeasurementList>::set, with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &stl_deque<CObservationRange::TMeasurementList>::del)
        ;
    }

    // CObservation2DRangeScan
    {
        class_<CObservation2DRangeScan, bases<CObservation> >("CObservation2DRangeScan", init<>())
            .def_readwrite("scan", &CObservation2DRangeScan::scan)
            .def_readwrite("validRange", &CObservation2DRangeScan::validRange)
            .def_readwrite("aperture", &CObservation2DRangeScan::aperture)
            .def_readwrite("sensorPose", &CObservation2DRangeScan::sensorPose)
            .def_readwrite("maxRange", &CObservation2DRangeScan::maxRange)
            .def_readwrite("stdError", &CObservation2DRangeScan::stdError)
            .def_readwrite("beamAperture", &CObservation2DRangeScan::beamAperture)
            .def_readwrite("deltaPitch", &CObservation2DRangeScan::deltaPitch)
            .def("to_ROS_LaserScan_msg", &CObservation2DRangeScan_to_ROS_LaserScan_msg, "Convert to ROS sensor_msgs/LaserScan.")
            .def("from_ROS_LaserScan_msg", &CObservation2DRangeScan_from_ROS_LaserScan_msg, "Convert to ROS sensor_msgs/LaserScan.")
        ;
    }

    // CRawlog
    {
        class_<CRawlog>("CRawlog", init<>())
            .def("clear", &CRawlog::clear, "Clear the sequence of actions/observations, freeing the memory of all the objects in the list.")
            .def("addAction", &CRawlog::addAction, "Add an action to the sequence: a collection of just one element is created. The object is duplicated, so the original one can be free if desired.")
            .def("addActions", &CRawlog::addActions, "Add a set of actions to the sequence; the object is duplicated, so the original one can be free if desired.")
            .def("addObservations", &CRawlog::addObservations, "Add a set of observations to the sequence; the object is duplicated, so the original one can be free if desired.")
            .def("loadFromRawLogFile", &CRawlog::loadFromRawLogFile, "Load the contents from a file containing either CRawLog objects or directly Action/Observation object pairs.")
            .def("saveToRawLogFile", &CRawlog::saveToRawLogFile, "Saves the contents to a rawlog-file, compatible with RawlogViewer (As the sequence of internal objects).")
        ;
    }

    // CSensoryFrame
    {
        class_<CSensoryFrame>("CSensoryFrame", init<>())
            .def("clear", &CSensoryFrame::clear, "Clear all current observations.")
            .def("insert", &CSensoryFrame_insert1, "Inserts a new observation to the list.")
            .def("insert", &CSensoryFrame_insert2, "Inserts a new observation to the list.")
            .def("size", &CSensoryFrame::size, "Returns the number of observations in the list")
            .def("eraseByIndex", &CSensoryFrame::eraseByIndex, "Removes the i'th observation in the list (0=first).")
        ;
    }

    // CSimpleMap
    {
        class_<CSimpleMap>("CSimpleMap", init<>())
            .def("saveToFile", &CSimpleMap::saveToFile, "Save this object to a .simplemap binary file (compressed with gzip)")
            .def("loadFromFile", &CSimpleMap::loadFromFile, "Load the contents of this object from a .simplemap binary file (possibly compressed with gzip).")
            .def("insert", &CSimpleMap_insert, "Add a new pair to the sequence. The objects are copied, so original ones can be free if desired after insertion.")
            .def_readwrite("size", &CSimpleMap::size)
        ;
    }
}
