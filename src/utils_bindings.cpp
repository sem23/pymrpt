#include "utils_bindings.h"

/* STD */
#include <stdint.h>

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

void CSerializableWrap::writeToStream(mrpt::utils::CStream &out, int32_t *getVersion) const
{
    this->get_override("writeToStream")(out, getVersion);
}

void CSerializableWrap::readFromStream(mrpt::utils::CStream &in, int32_t version)
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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CRobotSimulator_setDelayModelParams_overloads, setDelayModelParams, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CRobotSimulator_setOdometryErrors_overloads, setOdometryErrors, 1, 7)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CRobotSimulator_resetOdometry_overloads, resetOdometry, 0, 1)

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
            .def("loadFromConfigFile", &CLoadableOptionsWrap::loadFromConfigFile, "This method load the options from a \".ini\"-like file or memory-stored string list.")
            .def("loadFromConfigFileName", &CLoadableOptionsWrap::loadFromConfigFileName, "Behaves like loadFromConfigFile, but you can pass directly a file name and a temporary CConfigFile object will be created automatically to load the file.")
            .def("saveToConfigFile", &CLoadableOptionsWrap::saveToConfigFile, "This method saves the options to a \".ini\"-like file or memory-stored string list.")
            .def("saveToConfigFileName", &CLoadableOptionsWrap::saveToConfigFileName, "Behaves like saveToConfigFile, but you can pass directly a file name and a temporary CConfigFile object will be created automatically to save the file.")
            .def("dumpToConsole", &CLoadableOptionsWrap::dumpToConsole, "Just like dumpToTextStream() but sending the text to the console (std::cout)")
            .def("dumpToTextStream", &CLoadableOptionsWrap::dumpToTextStream, "This method should clearly display all the contents of the structure in textual form, sending it to a CStream.")
        ;
    }

    // CRobotSimulator
    {
        class_<CRobotSimulator>("CRobotSimulator", "This class can be used to simulate the kinematics and dynamics of a differential driven planar mobile robot, including odometry errors and dynamics limitations.", init< optional<float, float> >(args("TAU", "DELAY"), "Constructor with default dynamic model-parameters"))
            .def("setDelayModelParams", &CRobotSimulator::setDelayModelParams, CRobotSimulator_setDelayModelParams_overloads(args("TAU_delay_sec=1.8f", "CMD_delay_sec=0.3f"), "Change the model of delays used for the orders sent to the robot"))
            .def("setOdometryErrors", &CRobotSimulator::setOdometryErrors, CRobotSimulator_setOdometryErrors_overloads( args("enabled", "Ax_err_bias", "Ax_err_std", "Ay_err_bias", "Ay_err_std", "Aphi_err_bias", "Aphi_err_std"), "Enable/Disable odometry errors. Errors in odometry are introduced per millisecond."))
            .def("setRealPose", &CRobotSimulator::setRealPose, args("pose"), "Reset actual robot pose (inmediately, without simulating the movement along time).")
            .def("getX", &CRobotSimulator::getX, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getY", &CRobotSimulator::getY, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getPHI", &CRobotSimulator::getPHI, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getT", &CRobotSimulator::getT, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getV", &CRobotSimulator::getV, "Read the instantaneous, error-free status of the simulated robot.")
            .def("getW", &CRobotSimulator::getW, "Read the instantaneous, error-free status of the simulated robot.")
            .def("setV", &CRobotSimulator::setV, args("v"), "Set actual robot velocity, error-free status of the simulated robot.")
            .def("setW", &CRobotSimulator::setW, args("w"), "Set actual robot turnrate, error-free status of the simulated robot.")
            .def("movementCommand", &CRobotSimulator::movementCommand, args("lin_vel", "ang_vel"), "Used to command the robot a desired movement (velocities).")
            .def("resetStatus", &CRobotSimulator::resetStatus, "Reset all the simulator variables to 0 (All but current simulator time).")
            .def("resetTime", &CRobotSimulator::resetTime, "Reset time counter.")
            .def("simulateInterval", &CRobotSimulator::simulateInterval, args("dt"), "This method must be called periodically to simulate discrete time intervals.")
            .def("resetOdometry", &CRobotSimulator::resetOdometry,  CRobotSimulator_resetOdometry_overloads(args("newOdo"), "Forces odometry to be set to a specified values."))
            .def("getOdometry", &CRobotSimulator_getOdometry, "Reads the simulated robot odometry (this is NOT equal to the actual error-free robot coordinates).")
            .def("getRealPose", &CRobotSimulator_getRealPose, "Reads the real robot pose.")
        ;
    }

    // static module functions
    def("DEG2RAD", &mrpt_utils_DEG2RAD, args("deg"), "Convert degrees to radiants.");
    def("RAD2DEG", &mrpt_utils_RAD2DEG, args("rad"), "Convert radiants to degrees.");
}
