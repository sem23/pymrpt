#ifndef __SLAM_BINDINGS_H__
#define __SLAM_BINDINGS_H__

/* MRPT */
#include <mrpt/slam/CMetricMap.h>

/* BOOST */
#include <boost/python.hpp>


using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::slam;

// CMetricMap
struct CMetricMapWrap : CMetricMap, wrapper<CMetricMap>
{
    void internal_clear();
    bool isEmpty();
    double computeObservationLikelihood(const CObservation *obs, const CPose3D &takenFrom);
    void saveMetricMapRepresentationToFile(const std::string &filNamePrefix);
    void getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj);
};

void export_slam();

#endif
