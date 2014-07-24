#ifndef __UTILS_BINDINGS_H__
#define __UTILS_BINDINGS_H__

/* MRPT */
#include <mrpt/utils/CObject.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CRobotSimulator.h>


/* BOOST */
#include <boost/python.hpp>

using namespace boost::python;
using namespace mrpt::utils;
using namespace mrpt::poses;

// CObject wrapper
struct CObjectWrap : CObject, wrapper<CObject>
{
    CObject *duplicate() const;
};

// CSerializable wrapper
struct CSerializableWrap : CSerializable, wrapper<CSerializable>
{
    CObject *duplicate() const;
    void writeToStream(mrpt::utils::CStream &out, int *getVersion) const;
    void readFromStream(mrpt::utils::CStream &in, int version);
};


// exporter
void export_utils();

#endif
