#pragma once

#include <mobilerack-interface/QualisysClient.h>
#include <mobilerack-interface/SerialInterface.h>
#include "SoftTrunk_common.h"

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <thread>
#include <cmath>
#include <mutex>
#include <complex>

/**
 * @brief Calculates the PCC configuration of each soft arm segment based on motion track / internal sensor measurement.
 * 
 * Since the PCC model is just an approximation, there are several methods to calculate the curvature from the frame data from motion capture system.
 * For this class, curvature is calculated using the origin of the frame for tip of each segment, expressed in coordinate frames of the base of the segment.
 * The relation between theta and a in the image is unsolvable for x (a = tan(theta/2) * sin(theta) *L/theta where L is backbone length), so use a third order approximation instead (a = L (x/2 - x^3/24)) and solve with Wolfram Alpha.
 * @image html img/calculation_from_frames.jpg
 * @par Qualisys motion tracking sensor
 * the frames have to be set up properly in QTM (using Rigid Body) first.
 * Rigid Body label conventions: base of robot is 0, tip of first segment is 1, and so on...
 * Z axis of each frame is parallel to lengthwise direction of the arm, and origin of each frame is at center of tip of segment
 * @par Bend Labs sensor
 * @image html bendlabs_coordinates.jpg
 * @todo cannot gracefully deal with missed frames etc when there are occlusions.
 */
class CurvatureCalculator {
public:
    enum class SensorType{
        qualisys,
        bend_labs
    };
    enum class CalcMethod{
        position /** calculate curvature using relative xyz coordinates */,
        orientation /** calculate curvature using orientation of z axis */
    };

    /**
     * @brief Construct a new Curvature Calculator object
     * 
     * @param address IP address of QTM PC when using Qualisys, portname of Arduino when using Bend Labs.
     */
    CurvatureCalculator(CurvatureCalculator::SensorType sensor_type, std::string address);

    ~CurvatureCalculator();

    void get_curvature(Pose &pose);

    unsigned long long int get_timestamp(); /** @brief get timestamp (in microsecs) from Qualisys  */

    /** @brief get a single frame data from qualisys, only usable when CurvatureCalculator is set to use Qualisys. */
    Eigen::Transform<double, 3, Eigen::Affine> get_frame(int id);

private:
    SensorType sensor_type;

    std::unique_ptr<QualisysClient> optiTrackClient;
    /**
     * @brief recorded data from motion tracking system. Saves absolute transforms for each frame.
     */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> abs_transforms;
    
    std::unique_ptr<SerialInterface> serialInterface;
    std::vector<float> bendLab_data;

    const CalcMethod calcMethod = CalcMethod::orientation;

    const double L = 0.12; /** @brief backbone length in meters, this is assumed to be constant when bending. Used when calcMethod is position. */

    unsigned long long int timestamp = 0;
    
    std::thread calculatorThread;
    
    /**
     * @brief background process that calculates curvature
     */
    void calculator_loop();

    /**
     * @brief thread runs while this is true
     */
    bool run;
    const bool log = true;

    /**
     * @brief calculates q from the current frame values.
     */
    void calculateCurvature();

    VectorXd initial_q;
    std::mutex mtx;

    /** @brief PCC configuration of each segment of soft arm. depends on st_params::parametrization */
    Pose pose;
    
    void setupQualisys(std::string qtm_address);

    /**
     * @brief for future, if you want to use sensors embedded in arm.
     */
    void setupIntegratedSensor(std::string portname);
    
};
