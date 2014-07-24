/* MRPT */
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFParticles.h>

#include <mrpt/utils/CStream.h>

/* BOOST */
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "poses_bindings.h"

using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::utils;


// CPose2D
double &(CPose2D::*CPose2D_get_x)()             = &CPose2D::x;
void    (CPose2D::*CPose2D_set_x)(double)       = &CPose2D::x;
double &(CPose2D::*CPose2D_get_y)()             = &CPose2D::y;
void    (CPose2D::*CPose2D_set_y)(double)       = &CPose2D::y;
double &(CPose2D::*CPose2D_get_phi)()           = &CPose2D::phi;
void    (CPose2D::*CPose2D_set_phi)(double)     = &CPose2D::phi;

// CPose3D
double &(CPose3D::*CPose3D_get_x)()             = &CPose3D::x;
void    (CPose3D::*CPose3D_set_x)(double)       = &CPose3D::x;
double &(CPose3D::*CPose3D_get_y)()             = &CPose3D::y;
void    (CPose3D::*CPose3D_set_y)(double)       = &CPose3D::y;
double &(CPose3D::*CPose3D_get_z)()             = &CPose3D::z;
void    (CPose3D::*CPose3D_set_z)(double)       = &CPose3D::z;


tuple CPose3D_getYawPitchRoll(CPose3D &self)
{
    list ret_val;
    double yaw, pitch, roll;
    self.getYawPitchRoll(yaw, pitch, roll);
    ret_val.append(yaw);
    ret_val.append(pitch);
    ret_val.append(roll);
    return tuple(ret_val);
}


// CPosePDF
CObject *CPosePDFWrap::duplicate() const
{
    return this->get_override("duplicate")();
}

void CPosePDFWrap::writeToStream(CStream& stream, int* pos) const
{
    this->get_override("writeToStream")(stream, pos);
}

void CPosePDFWrap::readFromStream(CStream& stream, int pos)
{
    this->get_override("readFromStream")(stream, pos);
}

void CPosePDFWrap::getMean(mrpt::poses::CPose2D &mean_point) const
{
    this->get_override("getMean")(mean_point);
}

void CPosePDFWrap::getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,3ul,3ul> &cov, mrpt::poses::CPose2D &mean_point) const
{
    this->get_override("getCovarianceAndMean")(cov, mean_point);
}

void CPosePDFWrap::saveToTextFile(const std::string &file) const
{
    this->get_override("saveToTextFile")(file);
}

void CPosePDFWrap::drawSingleSample(mrpt::poses::CPose2D &outPart) const
{
    this->get_override("drawSingleSample")(outPart);
}

void CPosePDFWrap::changeCoordinatesReference(const mrpt::poses::CPose3D &newReferenceBase)
{
    this->get_override("changeCoordinatesReference")(newReferenceBase);
}

void CPosePDFWrap::copyFrom(const CPosePDF &o)
{
    this->get_override("copyFrom")(o);
}

void CPosePDFWrap::bayesianFusion(const CPosePDF &p1, const CPosePDF &p2, const double &minMahalanobisDistToDrop)
{
    this->get_override("bayesianFusion")(p1, p2, minMahalanobisDistToDrop);
}

void CPosePDFWrap::inverse(CPosePDF &o) const
{
    this->get_override("inverse")(o);
}
// end of CPosePDF

// CPose3DPDF
struct CPose3DPDFWrap : CPose3DPDF, wrapper<CPose3DPDF>
{
    CObject *duplicate() const
    {
        return this->get_override("duplicate")();
    }

    void writeToStream(mrpt::utils::CStream& stream, int* pos) const
    {
        this->get_override("writeToStream")(stream, pos);
    }

    void readFromStream(mrpt::utils::CStream& stream, int pos)
    {
        this->get_override("readFromStream")(stream, pos);
    }

    void getMean(mrpt::poses::CPose3D &mean_point) const
    {
        this->get_override("getMean")(mean_point);
    }

    void getCovarianceAndMean(mrpt::math::CMatrixFixedNumeric<double,6ul,6ul> &cov, mrpt::poses::CPose3D &mean_point) const
    {
        this->get_override("getCovarianceAndMean")(cov, mean_point);
    }

    void saveToTextFile(const std::string &file) const
    {
        this->get_override("saveToTextFile")(file);
    }

    void changeCoordinatesReference(const mrpt::poses::CPose3D &newReferenceBase)
    {
        this->get_override("changeCoordinatesReference")(newReferenceBase);
    }

    void drawSingleSample(mrpt::poses::CPose3D &outPart) const
    {
        this->get_override("drawSingleSample")(outPart);
    }

    void copyFrom(const CPose3DPDF &o)
    {
        this->get_override("copyFrom")(o);
    }

    void bayesianFusion(const CPose3DPDF &p1, const CPose3DPDF &p2)
    {
        this->get_override("bayesianFusion")(p1, p2);
    }

    void inverse(CPose3DPDF &o) const
    {
        this->get_override("inverse")(o);
    }
};

void export_poses()
{
    // map namespace to be submodule of mrpt package
    object poses_module(handle<>(borrowed(PyImport_AddModule("mrpt.poses"))));
    scope().attr("poses") = poses_module;
    scope poses_scope = poses_module;

    // CPosePDF
    {
        class_<CPosePDFWrap, boost::noncopyable>("CPosePDF", no_init)
            .def("writeToStream", &CPosePDFWrap::writeToStream)
            .def("readFromStream", &CPosePDFWrap::readFromStream)
            .def("getMean", &CPosePDFWrap::getMean)
            .def("getCovarianceAndMean", &CPosePDFWrap::getCovarianceAndMean)
            .def("saveToTextFile", &CPosePDFWrap::saveToTextFile)
            .def("copyFrom", &CPosePDFWrap::copyFrom)
            .def("bayesianFusion", &CPosePDFWrap::bayesianFusion)
            .def("inverse", &CPosePDFWrap::inverse)
        ;
    }

    // CPosePDFGaussian
    {
        class_<CPosePDFGaussian, bases<CPosePDF> >("CPosePDFGaussian", init<optional<CPose2D> >())
            .def_readwrite("mean", &CPosePDFGaussian::mean)
            .def_readwrite("cov", &CPosePDFGaussian::cov)
        ;
    }

    // CPose2D
    {
        class_<CPose2D>("CPose2D", init<>())
            .def(init<CPose3D>())
            .def(init<double, double, double>())
            .add_property("x",
                make_function(CPose2D_get_x, return_value_policy<copy_non_const_reference>()),
                CPose2D_set_x
            )
            .add_property("y",
                make_function(CPose2D_get_y, return_value_policy<copy_non_const_reference>()),
                CPose2D_set_y
            )
            .add_property("phi",
                make_function(CPose2D_get_phi, return_value_policy<copy_non_const_reference>()),
                CPose2D_set_phi
            )
            .def(self + self)
            .def(self - self)
        ;
    }

    // CPose3D
    {
        class_<CPose3D>("CPose3D", init<optional<CPose2D> >())
            .add_property("x",
                make_function(CPose3D_get_x, return_value_policy<copy_non_const_reference>()),
                CPose3D_set_x
            )
            .add_property("y",
                make_function(CPose3D_get_y, return_value_policy<copy_non_const_reference>()),
                CPose3D_set_y
            )
            .add_property("z",
                make_function(CPose3D_get_z, return_value_policy<copy_non_const_reference>()),
                CPose3D_set_z
            )
            .def("setYawPitchRoll", &CPose3D::setYawPitchRoll)
            .def("getYawPitchRoll", &CPose3D_getYawPitchRoll)
            .def("setFromValues", &CPose3D::setFromValues)
        ;
    }

    // CPose3DPDF
    {
        class_<CPose3DPDFWrap, boost::noncopyable>("CPose3DPDF", no_init)
            .def("jacobiansPoseComposition", &CPose3DPDFWrap::jacobiansPoseComposition)
        ;
    }

    // CPose3DPDFParticles
    {
        class_<CPose3DPDFParticles, bases<CPose3DPDF> >("CPose3DPDFParticles", init<>())
            .def("getMean", &CPose3DPDFParticles::getMean)
        ;
    }
}