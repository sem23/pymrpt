#include "utils_bindings.h"

// CObject wrapper
CObject *CObjectWrap::duplicate() const
{
    return this->get_override("duplicate")();
}

// CSerializable wrapper
CObject *CSerializableWrap::duplicate() const
{
    return this->get_override("duplicate")();
}

void CSerializableWrap::writeToStream(mrpt::utils::CStream &out, int *getVersion) const
{
    this->get_override("writeToStream")(out, getVersion);
}

void CSerializableWrap::readFromStream(mrpt::utils::CStream &in, int version)
{
    this->get_override("readFromStream")(in, version);
}
// end of CSerializable wrapper

// CLoadableOptions wrapper
struct CLoadableOptionsWrap : CLoadableOptions, wrapper<CLoadableOptions>
{
    void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source, const std::string &section)
    {
        this->get_override("loadFromConfigFile")(source, section);
    }

    void saveToConfigFile(const mrpt::utils::CConfigFileBase &target, const std::string &section) const
    {
        this->get_override("saveToConfigFile")(target, section);
    }

    void dumpToTextStream(mrpt::utils::CStream& out) const
    {
        this->get_override("dumpToTextStream")(out);
    }
};

// CRobotSimulator
void CRobotSimulator_setDelayModelParams(CRobotSimulator self, float TAU, float DELAY)
{
    self.setDelayModelParams(TAU, DELAY);
}

void CRobotSimulator_setOdometryErrors(
    CRobotSimulator &self,
    bool enabled,
    double Ax_err_bias,
    double Ax_err_std,
    double Ay_err_bias,
    double Ay_err_std,
    double Aphi_err_bias,
    double Aphi_err_std)
{
    self.setOdometryErrors(
        enabled,
        Ax_err_bias,
        Ax_err_std,
        Ay_err_bias,
        Ay_err_std,
        Aphi_err_bias,
        Aphi_err_std
    );
}

CPose2D CRobotSimulator_getOdometry(CRobotSimulator &self)
{
    CPose2D pose;
    self.getOdometry(pose);
    return pose;
}

CPose2D CRobotSimulator_getRealPose(CRobotSimulator &self)
{
    CPose2D pose;
    self.getRealPose(pose);
    return pose;
}
// end of CRobotSimulator


// Utils
double mrpt_utils_DEG2RAD(double deg) { return mrpt::utils::DEG2RAD(deg); }
double mrpt_utils_RAD2DEG(double rad) { return mrpt::utils::RAD2DEG(rad); }
// end of Utils

// exporter
void export_utils()
{
    // map namespace to be submodule of mrpt package
    object utils_module(handle<>(borrowed(PyImport_AddModule("mrpt.utils"))));
    scope().attr("utils") = utils_module;
    scope utils_scope = utils_module;

    // CObject
    {
        class_<CObjectWrap, boost::noncopyable>("CObject", no_init)
            .def("duplicate", &CObjectWrap::duplicate, return_value_policy<manage_new_object>(), "Returns a copy of the object, indepently of its class.")
        ;
    }

    // CLoadableOptions
    {
        class_<CLoadableOptionsWrap, boost::noncopyable>("CLoadableOptions", no_init)
            .def("loadFromConfigFile", &CLoadableOptionsWrap::loadFromConfigFile)
            .def("loadFromConfigFileName", &CLoadableOptionsWrap::loadFromConfigFileName)
            .def("saveToConfigFile", &CLoadableOptionsWrap::saveToConfigFile)
            .def("saveToConfigFileName", &CLoadableOptionsWrap::saveToConfigFileName)
            .def("dumpToConsole", &CLoadableOptionsWrap::dumpToConsole)
            .def("dumpToTextStream", &CLoadableOptionsWrap::dumpToTextStream)
        ;
    }

    // CRobotSimulator
    {
        class_<CRobotSimulator>("CRobotSimulator", init<>())
            .def("setDelayModelParams", &CRobotSimulator_setDelayModelParams, "Change the model of delays used for the orders sent to the robot")
            .def("setOdometryErrors", &CRobotSimulator_setOdometryErrors, "Enable/Disable odometry errors")
            .def("setRealPose", &CRobotSimulator::setRealPose)
            .def("getX", &CRobotSimulator::getX)
            .def("getY", &CRobotSimulator::getY)
            .def("getPHI", &CRobotSimulator::getPHI)
            .def("getT", &CRobotSimulator::getT)
            .def("getV", &CRobotSimulator::getV)
            .def("getW", &CRobotSimulator::getW)
            .def("setV", &CRobotSimulator::setV)
            .def("setW", &CRobotSimulator::setW)
            .def("movementCommand", &CRobotSimulator::movementCommand)
            .def("resetStatus", &CRobotSimulator::resetStatus)
            .def("resetTime", &CRobotSimulator::resetTime)
            .def("resetOdometry", &CRobotSimulator::resetOdometry)
            .def("simulateInterval", &CRobotSimulator::simulateInterval)
            .def("getOdometry", &CRobotSimulator_getOdometry)
            .def("getRealPose", &CRobotSimulator_getRealPose)
        ;
    }

    // static module functions
    def("DEG2RAD", &mrpt_utils_DEG2RAD);
    def("RAD2DEG", &mrpt_utils_RAD2DEG);
}
