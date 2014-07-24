/* MRPT */
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservationRange.h>
#include <mrpt/slam/CObservation2DRangeScan.h>

#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/COccupancyGridMap2D.h>

#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/CPathPlanningCircularRobot.h>

#include <mrpt/poses/CPosePDFGaussian.h>

#include <mrpt/system/datetime.h>

/* BOOST */
#include <boost/python.hpp>

#include "maps_bindings.h"
#include "slam_bindings.h"
#include "system_bindings.h"

using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::slam;

struct CMetricMapWrap;

// COccupancyGridMap2D
bool COccupancyGridMap2D_insertObservation(COccupancyGridMap2D &self, CObservation &obs, CPose3D &pose)
{
    return self.insertObservation(&obs, &pose);
}

object COccupancyGridMap2D_to_ROS_OccupancyGrid_msg1(COccupancyGridMap2D &self, str frame_id)
{
    // import msg
    dict locals;
    exec("from nav_msgs.msg import OccupancyGrid\n"
         "occupancy_grid_msg = OccupancyGrid()\n",
         object(), locals);
    object occupancy_grid_msg = locals["occupancy_grid_msg"];
    // set info
    int width = self.getSizeX();
    int height = self.getSizeY();
    occupancy_grid_msg.attr("header").attr("frame_id") = frame_id;
    occupancy_grid_msg.attr("header").attr("stamp") = TTimeStamp_to_ROS_Time(long_(mrpt::system::now()));
    occupancy_grid_msg.attr("info").attr("width") = width;
    occupancy_grid_msg.attr("info").attr("height") = height;
    occupancy_grid_msg.attr("info").attr("resolution") = self.getResolution();
    occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("x") = self.getXMin();
    occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("y") = self.getYMin();
    // set data
    boost::python::list data;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float occupancy = self.getCell(x,y);
            if (occupancy < 0.45) {
                data.append(100);
            } else if (occupancy > 0.55){
                data.append(0);
            } else {
                data.append(-1);
            }
        }
    }
    occupancy_grid_msg.attr("data") = data;
    return occupancy_grid_msg;
}

object COccupancyGridMap2D_to_ROS_OccupancyGrid_msg2(COccupancyGridMap2D &self)
{
    return COccupancyGridMap2D_to_ROS_OccupancyGrid_msg1(self, str("map"));
}

void COccupancyGridMap2D_from_ROS_OccupancyGrid_msg(COccupancyGridMap2D &self, object occupancy_grid_msg)
{
    // set info
    float x_min = extract<float>(occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("x"));
    float y_min = extract<float>(occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("y"));
    float resolution = extract<float>(occupancy_grid_msg.attr("info").attr("resolution"));
    int width = extract<int>(occupancy_grid_msg.attr("info").attr("width"));
    int height = extract<int>(occupancy_grid_msg.attr("info").attr("height"));
    float x_max = x_min + width * resolution;
    float y_max = y_min + height * resolution;
    self.setSize(x_min, x_max, y_min, y_max, resolution);
    // set data
    int idx = 0;
    boost::python::list data = extract<boost::python::list>(occupancy_grid_msg.attr("data"));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int occupancy = extract<int>(data[idx]);
            if (occupancy >= 0) {
                self.setCell(x, y, (100 - occupancy) / 100.0);
            } else {
                self.setCell(x, y, 0.5);
            }
            idx++;
        }
    }
}
// end of COccupancyGridMap2D


// CPointsMap
struct CPointsMapWrap : CPointsMap, wrapper<CPointsMap>
{
    CObject *duplicate() const
    {
        return this->get_override("duplicate")();
    }

    void PLY_import_set_vertex_count(size_t count)
    {
        this->get_override("PLY_import_set_vertex_count")(count);
    }

    void reserve(size_t newLength)
    {
        this->get_override("reserve")(newLength);
    }

    void resize(size_t newLength)
    {
        this->get_override("resize")(newLength);
    }

    void setSize(size_t newLength)
    {
        this->get_override("setSize")(newLength);
    }

    void setPointFast(size_t index, float x, float y, float z)
    {
        this->get_override("setPointFast")(index, x, y, z);
    }

    void insertPointFast(float x, float y, float z)
    {
        this->get_override("insertPointFast")(x, y, z);
    }

    void copyFrom(const CPointsMap &obj)
    {
        this->get_override("copyFrom")(obj);
    }

    void getPointAllFieldsFast(const size_t index, std::vector<float> &point_data)
    {
        this->get_override("getPointAllFieldsFast")(index, point_data);
    }

    void setPointAllFieldsFast(const size_t index, const std::vector<float> &point_data)
    {
        this->get_override("setPointAllFieldsFast")(index, point_data);
    }

};
// end of CPointsMap

// CSimplePointsMap
void CSimplePointsMap_loadFromRangeScan1(CSimplePointsMap &self, const CObservation2DRangeScan &rangeScan)
{
    self.loadFromRangeScan(rangeScan);
}

void CSimplePointsMap_loadFromRangeScan2(CSimplePointsMap &self, const CObservation2DRangeScan &rangeScan, const CPose3D &robotPose)
{
    self.loadFromRangeScan(rangeScan, &robotPose);
}
// end of CSimplePointsMap

void export_maps()
{
    // map namespace to be submodule of package
    object maps_module(handle<>(borrowed(PyImport_AddModule("mrpt.maps"))));
    scope().attr("maps") = maps_module;
    scope maps_scope = maps_module;

    // COccupancyGridMap2D
    {
        scope s = class_<COccupancyGridMap2D>("COccupancyGridMap2D", init<optional<float, float, float, float, float> >())
            .def("insertObservation", &COccupancyGridMap2D_insertObservation)
            .def("saveAsBitmapFile", &COccupancyGridMap2D::saveAsBitmapFile)
            .def("loadFromBitmapFile", &COccupancyGridMap2D::loadFromBitmapFile)
            .def("getSizeX", &COccupancyGridMap2D::getSizeX)
            .def("getSizeY", &COccupancyGridMap2D::getSizeY)
            .def("getXMin", &COccupancyGridMap2D::getXMin)
            .def("getXMax", &COccupancyGridMap2D::getXMax)
            .def("getYMin", &COccupancyGridMap2D::getYMin)
            .def("getYMax", &COccupancyGridMap2D::getYMax)
            .def("getResolution", &COccupancyGridMap2D::getResolution)
            .def("setCell", &COccupancyGridMap2D::setCell)
            .def("getCell", &COccupancyGridMap2D::getCell)
            .def("laserScanSimulator", &COccupancyGridMap2D::laserScanSimulator)
            .def_readwrite("insertionOptions", &COccupancyGridMap2D::insertionOptions)
            .def_readwrite("likelihoodOptions", &COccupancyGridMap2D::likelihoodOptions)
            .def("to_ROS_OccupancyGrid_msg", &COccupancyGridMap2D_to_ROS_OccupancyGrid_msg1)
            .def("to_ROS_OccupancyGrid_msg", &COccupancyGridMap2D_to_ROS_OccupancyGrid_msg2)
            .def("from_ROS_OccupancyGrid_msg", &COccupancyGridMap2D_from_ROS_OccupancyGrid_msg)
        ;

        // TInsertionOptions
        class_<COccupancyGridMap2D::TInsertionOptions, bases<CLoadableOptions> >("TInsertionOptions", init<>())
            .def_readwrite("mapAltitude", &COccupancyGridMap2D::TInsertionOptions::mapAltitude)
            .def_readwrite("useMapAltitude", &COccupancyGridMap2D::TInsertionOptions::useMapAltitude)
            .def_readwrite("maxDistanceInsertion", &COccupancyGridMap2D::TInsertionOptions::maxDistanceInsertion)
            .def_readwrite("maxOccupancyUpdateCertainty", &COccupancyGridMap2D::TInsertionOptions::maxOccupancyUpdateCertainty)
            .def_readwrite("considerInvalidRangesAsFreeSpace", &COccupancyGridMap2D::TInsertionOptions::considerInvalidRangesAsFreeSpace)
            .def_readwrite("decimation", &COccupancyGridMap2D::TInsertionOptions::decimation)
            .def_readwrite("horizontalTolerance", &COccupancyGridMap2D::TInsertionOptions::horizontalTolerance)
            .def_readwrite("CFD_features_gaussian_size", &COccupancyGridMap2D::TInsertionOptions::CFD_features_gaussian_size)
            .def_readwrite("CFD_features_median_size", &COccupancyGridMap2D::TInsertionOptions::CFD_features_median_size)
            .def_readwrite("wideningBeamsWithDistance", &COccupancyGridMap2D::TInsertionOptions::wideningBeamsWithDistance)
        ;

        // TLikelihoodOptions
        class_<COccupancyGridMap2D::TLikelihoodOptions, bases<CLoadableOptions> >("TLikelihoodOptions", init<>())
            .def_readwrite("likelihoodMethod", &COccupancyGridMap2D::TLikelihoodOptions::likelihoodMethod)
            .def_readwrite("LF_stdHit", &COccupancyGridMap2D::TLikelihoodOptions::LF_stdHit)
            .def_readwrite("LF_zHit", &COccupancyGridMap2D::TLikelihoodOptions::LF_zHit)
            .def_readwrite("LF_zRandom", &COccupancyGridMap2D::TLikelihoodOptions::LF_zRandom)
            .def_readwrite("LF_maxRange", &COccupancyGridMap2D::TLikelihoodOptions::LF_maxRange)
            .def_readwrite("LF_decimation", &COccupancyGridMap2D::TLikelihoodOptions::LF_decimation)
            .def_readwrite("LF_maxCorrsDistance", &COccupancyGridMap2D::TLikelihoodOptions::LF_maxCorrsDistance)
            .def_readwrite("LF_alternateAverageMethod", &COccupancyGridMap2D::TLikelihoodOptions::LF_alternateAverageMethod)
            .def_readwrite("MI_exponent", &COccupancyGridMap2D::TLikelihoodOptions::MI_exponent)
            .def_readwrite("MI_skip_rays", &COccupancyGridMap2D::TLikelihoodOptions::MI_skip_rays)
            .def_readwrite("MI_ratio_max_distance", &COccupancyGridMap2D::TLikelihoodOptions::MI_ratio_max_distance)
            .def_readwrite("rayTracing_useDistanceFilter", &COccupancyGridMap2D::TLikelihoodOptions::rayTracing_useDistanceFilter)
            .def_readwrite("rayTracing_decimation", &COccupancyGridMap2D::TLikelihoodOptions::rayTracing_decimation)
            .def_readwrite("rayTracing_stdHit", &COccupancyGridMap2D::TLikelihoodOptions::rayTracing_stdHit)
            .def_readwrite("consensus_takeEachRange", &COccupancyGridMap2D::TLikelihoodOptions::consensus_takeEachRange)
            .def_readwrite("consensus_pow", &COccupancyGridMap2D::TLikelihoodOptions::consensus_pow)
            .def_readwrite("OWA_weights", &COccupancyGridMap2D::TLikelihoodOptions::OWA_weights)
            .def_readwrite("enableLikelihoodCache", &COccupancyGridMap2D::TLikelihoodOptions::enableLikelihoodCache)
        ;
    }

    // CPointsMap
    {
        scope s = class_<CPointsMapWrap, boost::noncopyable>("CPointsMap", no_init)
            .def("reserve", &CPointsMapWrap::reserve)
            .def("resize", &CPointsMapWrap::resize)
            .def("setSize", &CPointsMapWrap::setSize)
            .def("setPointFast", &CPointsMapWrap::setPointFast)
            .def("insertPointFast", &CPointsMapWrap::insertPointFast)
            .def("copyFrom", &CPointsMapWrap::copyFrom)
            .def("getPointAllFieldsFast", &CPointsMapWrap::getPointAllFieldsFast)
            .def("setPointAllFieldsFast", &CPointsMapWrap::setPointAllFieldsFast)
        ;
    }

    // CSimplePointsMap
    {
        scope s = class_<CSimplePointsMap, bases<CPointsMap> >("CSimplePointsMap", init<>())
            .def("loadFromRangeScan", &CSimplePointsMap_loadFromRangeScan1)
            .def("loadFromRangeScan", &CSimplePointsMap_loadFromRangeScan2)
        ;
    }

}
