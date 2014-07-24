#ifndef __POSES_BINDINGS_H__
#define __POSES_BINDINGS_H__

/* BOOST */
#include <boost/python.hpp>

/* MRPT */
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils/CStream.h>

/* set up namespace (spamming is ok here)*/
using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::utils;

/* CPosePDF */
struct CPosePDFWrap : CPosePDF, wrapper<CPosePDF>
{
    // from inherited class CProbabilityDensityFunction
    CObject *duplicate() const;
    void writeToStream(CStream& stream, int* pos) const;
    void readFromStream(CStream& stream, int pos);
    void getMean(CPose2D &mean_point) const;
    void getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,3ul,3ul> &cov, CPose2D &mean_point) const;
    void saveToTextFile(const std::string &file) const;
    void drawSingleSample(CPose2D &outPart) const;
    void changeCoordinatesReference(const CPose3D &newReferenceBase);

    // from CPosePDF
    void copyFrom(const CPosePDF &o);
    void bayesianFusion(const CPosePDF &p1, const CPosePDF &p2, const double &minMahalanobisDistToDrop);
    void inverse(CPosePDF &o) const;
};


void export_poses();

#endif
