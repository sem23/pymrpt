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

/* BOOST */
#include <boost/python.hpp>

#include "slam_bindings.h"
#include "poses_bindings.h"

using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::slam;

// TMetricMapInitializer
void TMetricMapInitializer_set_COccupancyGridMap2D(TMetricMapInitializer &self, float min_x, float max_x, float min_y, float max_y, float resolution)
{
    self.metricMapClassType = CLASS_ID(COccupancyGridMap2D);
    self.m_disableSaveAs3DObject = true;
    self.occupancyGridMap2D_options.min_x = min_x;
    self.occupancyGridMap2D_options.max_x = max_x;
    self.occupancyGridMap2D_options.min_y = min_y;
    self.occupancyGridMap2D_options.max_y = max_y;
    self.occupancyGridMap2D_options.resolution = resolution;
}

// CICP
tuple CICP_AlignPDF(CICP &self, COccupancyGridMap2D &m1, CSimplePointsMap &m2, CPosePDFGaussian &initialEstimationPDF)
{
    CPosePDFGaussian posePDF;
    float runningTime;
    CICP::TReturnInfo info;

    CPosePDFPtr posePDFPtr = self.AlignPDF(&m1, &m2, initialEstimationPDF, &runningTime, &info);
    posePDF.copyFrom(*posePDFPtr);

    boost::python::list ret_val;
    ret_val.append(posePDF);
    ret_val.append(runningTime);
    ret_val.append(info);
    return tuple(ret_val);
}
// end of CICP

// CMetricMap
void CMetricMapWrap::internal_clear()
{
    this->get_override("internal_clear")();
}

bool CMetricMapWrap::isEmpty()
{
    return this->get_override("isEmpty")();
}

double CMetricMapWrap::computeObservationLikelihood(const CObservation *obs, const CPose3D &takenFrom)
{
    return this->get_override("computeObservationLikelihood")(obs, takenFrom);
}

void CMetricMapWrap::saveMetricMapRepresentationToFile(const std::string &filNamePrefix)
{
    this->get_override("saveMetricMapRepresentationToFile")(filNamePrefix);
}

void CMetricMapWrap::getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj)
{
    this->get_override("getAs3DObject")(outObj);
}

// CMetricMapBuilder
struct CMetricMapBuilderWrap : CMetricMapBuilder, wrapper<CMetricMapBuilder>
{
    void initialize(const CSimpleMap &initialMap = CSimpleMap(), CPosePDF *x0 = NULL)
    {
        this->get_override("initialize")(initialMap, x0);
    }

    CPose3DPDFPtr getCurrentPoseEstimation() const
    {
        return this->get_override("getCurrentPoseEstimation")();
    }

    void processActionObservation(CActionCollection &action, CSensoryFrame &observations)
    {
        this->get_override("processActionObservation")(action, observations);
    }

    void getCurrentlyBuiltMap(CSimpleMap &out_map) const
    {
        this->get_override("getCurrentlyBuiltMap")(out_map);
    }

    void saveCurrentEstimationToImage(const std::string &file, bool formatEMF_BMP = true)
    {
        this->get_override("saveCurrentEstimationToImage")(file, formatEMF_BMP);
    }
};


void export_slam()
{
    // map namespace to be submodule of mrpt package
    object slam_module(handle<>(borrowed(PyImport_AddModule("mrpt.slam"))));
    scope().attr("slam") = slam_module;
    scope slam_scope = slam_module;

    // CMetricMap
    {
        scope s = class_<CMetricMapWrap, boost::noncopyable>("CMetricMap", no_init)
        ;
    }

    // CPathPlanningCircularRobot
    {
        class_<CPathPlanningCircularRobot>("CPathPlanningCircularRobot", init<>())
            .def("computePath", &CPathPlanningCircularRobot::computePath)
            .def_readwrite("robotRadius", &CPathPlanningCircularRobot::robotRadius)
        ;
    }

    // TMetricMapInitializer
    {
        scope s = class_<TMetricMapInitializer>("TMetricMapInitializer", init<>())
            .def("set_COccupancyGridMap2D", &TMetricMapInitializer_set_COccupancyGridMap2D)
            .def_readwrite("occupancyGridMap2D_options", &TMetricMapInitializer::occupancyGridMap2D_options)
        ;

        // TOccGridMap2DOptions
        class_<TMetricMapInitializer::TOccGridMap2DOptions>("TOccGridMap2DOptions", init<>())
            .def_readwrite("min_x", &TMetricMapInitializer::TOccGridMap2DOptions::min_x)
            .def_readwrite("max_x", &TMetricMapInitializer::TOccGridMap2DOptions::max_x)
            .def_readwrite("min_y", &TMetricMapInitializer::TOccGridMap2DOptions::min_y)
            .def_readwrite("max_y", &TMetricMapInitializer::TOccGridMap2DOptions::max_y)
            .def_readwrite("resolution", &TMetricMapInitializer::TOccGridMap2DOptions::resolution)
            .def_readwrite("insertionOpts", &TMetricMapInitializer::TOccGridMap2DOptions::insertionOpts)
            .def_readwrite("likelihoodOpts", &TMetricMapInitializer::TOccGridMap2DOptions::likelihoodOpts)
        ;
    }

    // TSetOfMetricMapInitializers
    {
        class_<TSetOfMetricMapInitializers>("TSetOfMetricMapInitializers", init<>())
            .def("size", &TSetOfMetricMapInitializers::size)
            .def("push_back", &TSetOfMetricMapInitializers::push_back)
            .def("clear", &TSetOfMetricMapInitializers::clear)
            .def_readwrite("options", &TSetOfMetricMapInitializers::options)
        ;
    }

    // CMultiMetricMap
    {
        scope s = class_<CMultiMetricMap>("CMultiMetricMap", init<optional<TSetOfMetricMapInitializers*, CMultiMetricMap::TOptions*> >())
        ;
        // TOptions
        class_<CMultiMetricMap::TOptions, bases<CLoadableOptions> >("TOptions", init<>())
        ;
    }

    // CMultiMetricMapPDF
    {
        scope s = class_<CMultiMetricMapPDF>("CMultiMetricMapPDF", init<>())
            .def_readwrite("options", &CMultiMetricMapPDF::options)
        ;

        // TConfigParams
        class_<CMultiMetricMapPDF::TPredictionParams, bases<CLoadableOptions> >("TPredictionParams", init<>())
            .def_readwrite("pfOptimalProposal_mapSelection", &CMultiMetricMapPDF::TPredictionParams::pfOptimalProposal_mapSelection)
            .def_readwrite("ICPGlobalAlign_MinQuality", &CMultiMetricMapPDF::TPredictionParams::ICPGlobalAlign_MinQuality)
            .def_readwrite("update_gridMapLikelihoodOptions", &CMultiMetricMapPDF::TPredictionParams::update_gridMapLikelihoodOptions)
            .def_readwrite("KLD_params", &CMultiMetricMapPDF::TPredictionParams::KLD_params)
            .def_readwrite("icp_params", &CMultiMetricMapPDF::TPredictionParams::icp_params)
        ;
    }

    // CICP
    {
        scope s = class_<CICP>("CICP", init<CICP::TConfigParams>())
            .def_readwrite("options", &CICP::options)
            .def("AlignPDF", &CICP_AlignPDF)
        ;

        class_<CICP::TConfigParams, bases<CLoadableOptions> >("TConfigParams", init<>())
            .def_readwrite("ICP_algorithm", &CICP::TConfigParams::ICP_algorithm)
            .def_readwrite("onlyClosestCorrespondences", &CICP::TConfigParams::onlyClosestCorrespondences)
            .def_readwrite("onlyUniqueRobust", &CICP::TConfigParams::onlyUniqueRobust)
            .def_readwrite("maxIterations", &CICP::TConfigParams::maxIterations)
            .def_readwrite("minAbsStep_trans", &CICP::TConfigParams::minAbsStep_trans)
            .def_readwrite("minAbsStep_rot", &CICP::TConfigParams::minAbsStep_rot)
            .def_readwrite("thresholdDist", &CICP::TConfigParams::thresholdDist)
            .def_readwrite("thresholdAng", &CICP::TConfigParams::thresholdAng)
            .def_readwrite("ALFA", &CICP::TConfigParams::ALFA)
            .def_readwrite("smallestThresholdDist", &CICP::TConfigParams::smallestThresholdDist)
            .def_readwrite("covariance_varPoints", &CICP::TConfigParams::covariance_varPoints)
            .def_readwrite("doRANSAC", &CICP::TConfigParams::doRANSAC)
            .def_readwrite("ransac_minSetSize", &CICP::TConfigParams::ransac_minSetSize)
            .def_readwrite("ransac_maxSetSize", &CICP::TConfigParams::ransac_maxSetSize)
            .def_readwrite("ransac_nSimulations", &CICP::TConfigParams::ransac_nSimulations)
            .def_readwrite("ransac_mahalanobisDistanceThreshold", &CICP::TConfigParams::ransac_mahalanobisDistanceThreshold)
            .def_readwrite("normalizationStd", &CICP::TConfigParams::normalizationStd)
            .def_readwrite("ransac_fuseByCorrsMatch", &CICP::TConfigParams::ransac_fuseByCorrsMatch)
            .def_readwrite("ransac_fuseMaxDiffXY", &CICP::TConfigParams::ransac_fuseMaxDiffXY)
            .def_readwrite("ransac_fuseMaxDiffPhi", &CICP::TConfigParams::ransac_fuseMaxDiffPhi)
            .def_readwrite("kernel_rho", &CICP::TConfigParams::kernel_rho)
            .def_readwrite("use_kernel", &CICP::TConfigParams::use_kernel)
            .def_readwrite("Axy_aprox_derivatives", &CICP::TConfigParams::Axy_aprox_derivatives)
            .def_readwrite("LM_initial_lambda", &CICP::TConfigParams::LM_initial_lambda)
            .def_readwrite("skip_cov_calculation", &CICP::TConfigParams::skip_cov_calculation)
            .def_readwrite("skip_quality_calculation", &CICP::TConfigParams::skip_quality_calculation)
            .def_readwrite("corresponding_points_decimation", &CICP::TConfigParams::corresponding_points_decimation)
        ;

        class_<CICP::TReturnInfo>("TReturnInfo", init<>())
            .def_readwrite("cbSize", &CICP::TReturnInfo::cbSize)
            .def_readwrite("nIterations", &CICP::TReturnInfo::nIterations)
            .def_readwrite("goodness", &CICP::TReturnInfo::goodness)
            .def_readwrite("quality", &CICP::TReturnInfo::quality)
        ;
    }
    // CMetricMapBuilder
    {
        scope s = class_<CMetricMapBuilderWrap, boost::noncopyable>("CMetricMapBuilder", no_init)
            .def("initialize", &CMetricMapBuilder::initialize)
            .def("clear", &CMetricMapBuilder::clear)
            .def("getCurrentPoseEstimation", &CMetricMapBuilder::getCurrentPoseEstimation)
            .def("processActionObservation", &CMetricMapBuilder::processActionObservation)
            .def("getCurrentlyBuiltMap", &CMetricMapBuilder::getCurrentlyBuiltMap)
            .def("getCurrentlyBuiltMapSize", &CMetricMapBuilder::getCurrentlyBuiltMapSize)
            .def("saveCurrentEstimationToImage", &CMetricMapBuilder::saveCurrentEstimationToImage)
            .def("enableMapUpdating", &CMetricMapBuilder::enableMapUpdating)
            .def("loadCurrentMapFromFile", &CMetricMapBuilder::loadCurrentMapFromFile)
            .def("saveCurrentMapToFile", &CMetricMapBuilder::saveCurrentMapToFile)
        ;
    }

    // CMetricMapBuilderICP
    {
        scope s = class_<CMetricMapBuilderICP, bases<CMetricMapBuilder> >("CMetricMapBuilderICP", init<>())
            .def_readwrite("ICP_options", &CMetricMapBuilderICP::ICP_options)
            .def_readwrite("ICP_params", &CMetricMapBuilderICP::ICP_params)
        ;

        // TConfigParams
        class_<CMetricMapBuilderICP::TConfigParams, bases<CLoadableOptions> >("TConfigParams", init<>())
            .def_readwrite("matchAgainstTheGrid", &CMetricMapBuilderICP::TConfigParams::matchAgainstTheGrid)
            .def_readwrite("insertionLinDistance", &CMetricMapBuilderICP::TConfigParams::insertionLinDistance)
            .def_readwrite("insertionAngDistance", &CMetricMapBuilderICP::TConfigParams::insertionAngDistance)
            .def_readwrite("localizationLinDistance", &CMetricMapBuilderICP::TConfigParams::localizationLinDistance)
            .def_readwrite("localizationAngDistance", &CMetricMapBuilderICP::TConfigParams::localizationAngDistance)
            .def_readwrite("minICPgoodnessToAccept", &CMetricMapBuilderICP::TConfigParams::minICPgoodnessToAccept)
            .def_readwrite("mapInitializers", &CMetricMapBuilderICP::TConfigParams::mapInitializers)
        ;
    }

    // CMetricMapBuilderRBPF
    {
        scope s = class_<CMetricMapBuilderRBPF, bases<CMetricMapBuilder> >("CMetricMapBuilderRBPF", init<CMetricMapBuilderRBPF::TConstructionOptions>())
            .def("drawCurrentEstimationToImage", &CMetricMapBuilderRBPF::drawCurrentEstimationToImage)
            .def("saveCurrentPathEstimationToTextFile", &CMetricMapBuilderRBPF::saveCurrentPathEstimationToTextFile)
            .def("getCurrentJointEntropy", &CMetricMapBuilderRBPF::getCurrentJointEntropy)
            .def_readwrite("mapPDF", &CMetricMapBuilderRBPF::mapPDF)
        ;

        // TConstructionOptions
        class_<CMetricMapBuilderRBPF::TConstructionOptions, bases<CLoadableOptions> >("TConstructionOptions", init<>())
            .def_readwrite("insertionLinDistance", &CMetricMapBuilderRBPF::TConstructionOptions::insertionLinDistance)
            .def_readwrite("insertionAngDistance", &CMetricMapBuilderRBPF::TConstructionOptions::insertionAngDistance)
            .def_readwrite("localizeLinDistance", &CMetricMapBuilderRBPF::TConstructionOptions::localizeLinDistance)
            .def_readwrite("localizeAngDistance", &CMetricMapBuilderRBPF::TConstructionOptions::localizeAngDistance)
            .def_readwrite("PF_options", &CMetricMapBuilderRBPF::TConstructionOptions::PF_options)
            .def_readwrite("mapsInitializers", &CMetricMapBuilderRBPF::TConstructionOptions::mapsInitializers)
            .def_readwrite("predictionOptions", &CMetricMapBuilderRBPF::TConstructionOptions::predictionOptions)
        ;
    }
}
