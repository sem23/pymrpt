/* MRPT */
//#include <mrpt/slam.h>
//#include <mrpt/utils.h>
//#include <mrpt/poses.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFParticles.h>

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

/* BOOST */
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "bindings.h"
#include "obs_bindings.h"
#include "maps_bindings.h"
#include "slam_bindings.h"
#include "utils_bindings.h"
#include "poses_bindings.h"
#include "system_bindings.h"

using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::bayes;


//
// Helpers
//
void IndexError() { PyErr_SetString(PyExc_IndexError, "Index out of range"); }





//
// Overloads
//


// CPosePDF
// static void (CPosePDF::*CPosePDF_jacobiansPoseComposition1)(const mrpt::poses::CPose2D &, const mrpt::poses::CPose2D &, mrpt::math::CMatrixDouble33 &, mrpt::math::CMatrixDouble33 &, const bool, const bool) = &CPosePDF::jacobiansPoseComposition;
// static void (CPosePDF::*CPosePDF_jacobiansPoseComposition2)(const mrpt::poses::CPosePDFGaussian &, const mrpt::poses::CPosePDFGaussian &, mrpt::math::CMatrixDouble33 &, mrpt::math::CMatrixDouble33 &) = &CPosePDF::jacobiansPoseComposition;


//
// The Module
//
BOOST_PYTHON_MODULE(pymrpt)
{
    // define this module to be a package
    object package = scope();
    package.attr("__path__") = "pymrpt";

    // STL
    class_<std::vector<float> >("FloatVec")
            .def(vector_indexing_suite<std::vector<float> >());
    class_<std::vector<double> >("DoubleVec")
            .def(vector_indexing_suite<std::vector<double> >());
    class_<std::vector<char> >("CharVec")
            .def(vector_indexing_suite<std::vector<char> >());

////////////
// system //
////////////
    export_system();

///////////
// utils //
///////////
    export_utils();

///////////
// poses //
///////////
    export_poses();

/////////
// obs //
/////////
    export_obs();

//////////
// maps //
//////////
    export_maps();

//////////
// slam //
//////////
    export_slam();

///////////
// bayes //
///////////
    // CParticleFilter
    {
        scope s = class_<CParticleFilter>("CParticleFilter", init<CParticleFilter>())
            .def("executeOn", &CParticleFilter::executeOn)
            .def_readwrite("m_options", &CParticleFilter::m_options)
        ;

        // TParticleFilterAlgorithm
        enum_<CParticleFilter::TParticleFilterAlgorithm>("TParticleFilterAlgorithm")
            .value("pfStandardProposal", CParticleFilter::pfStandardProposal)
            .value("pfAuxiliaryPFStandard", CParticleFilter::pfAuxiliaryPFStandard)
            .value("pfOptimalProposal", CParticleFilter::pfOptimalProposal)
            .value("pfAuxiliaryPFOptimal", CParticleFilter::pfAuxiliaryPFOptimal)
        ;

        // TParticleResamplingAlgorithm
        enum_<CParticleFilter::TParticleResamplingAlgorithm>("TParticleResamplingAlgorithm")
            .value("prMultinomial", CParticleFilter::prMultinomial)
            .value("prResidual", CParticleFilter::prResidual)
            .value("prStratified", CParticleFilter::prStratified)
            .value("prSystematic", CParticleFilter::prSystematic)
        ;

        // TParticleFilterOptions
        class_<CParticleFilter::TParticleFilterOptions, bases<CLoadableOptions> >("TParticleFilterOptions", init<>())
            .def_readwrite("adaptiveSampleSize", &CParticleFilter::TParticleFilterOptions::adaptiveSampleSize)
            .def_readwrite("BETA", &CParticleFilter::TParticleFilterOptions::BETA)
            .def_readwrite("sampleSize", &CParticleFilter::TParticleFilterOptions::sampleSize)
            .def_readwrite("pfAuxFilterOptimal_MaximumSearchSamples", &CParticleFilter::TParticleFilterOptions::pfAuxFilterOptimal_MaximumSearchSamples)
            .def_readwrite("powFactor", &CParticleFilter::TParticleFilterOptions::powFactor)
            .def_readwrite("PF_algorithm", &CParticleFilter::TParticleFilterOptions::PF_algorithm)
            .def_readwrite("resamplingMethod", &CParticleFilter::TParticleFilterOptions::resamplingMethod)
            .def_readwrite("max_loglikelihood_dyn_range", &CParticleFilter::TParticleFilterOptions::max_loglikelihood_dyn_range)
            .def_readwrite("pfAuxFilterStandard_FirstStageWeightsMonteCarlo", &CParticleFilter::TParticleFilterOptions::pfAuxFilterStandard_FirstStageWeightsMonteCarlo)
            .def_readwrite("verbose", &CParticleFilter::TParticleFilterOptions::verbose)
            .def_readwrite("pfAuxFilterOptimal_MLE", &CParticleFilter::TParticleFilterOptions::pfAuxFilterOptimal_MLE)
        ;

        // TParticleFilterOptions
        class_<CParticleFilter::TParticleFilterStats>("TParticleFilterStats", init<>())
            .def_readwrite("ESS_beforeResample", &CParticleFilter::TParticleFilterStats::ESS_beforeResample)
            .def_readwrite("weightsVariance_beforeResample", &CParticleFilter::TParticleFilterStats::weightsVariance_beforeResample)
        ;
    }
}
