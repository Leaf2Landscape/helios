#pragma once

#include "AbstractBeamDeflector.h"
#include <limits> // Required for std::numeric_limits

/**
 * @brief Class representing a polygon mirror beam deflector with three facets, Nadir, Forward, Backward
 */
class PolygonMirrorNFBBeamDeflector : public AbstractBeamDeflector {

protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Effective scan angle (radians)
     */
	double cfg_device_scanAngleEffective_rad = 0;
	/**
	 * @brief Maximum effective scan angle (radians)
	 */
	double cfg_device_scanAngleEffectiveMax_rad = 0;

  /**
   * @brief The forward/backward tilt angle calculated in the last doSimStep call (radians).
   */
  double state_currentForwardAngle_rad = 0.0;

  int state_currentLineCounter = 0;

  /**
   * @brief The calculated maximum effective forward angle based on settings (radians).
   */
  double cached_maxSetRollAngle_rad = std::numeric_limits<double>::quiet_NaN(); // NEW member variable, init to NaN

  /**
   * @brief The calculated maximum effective forward angle based on settings (radians).
   */
  double cached_maxSetForwardAngle_rad = std::numeric_limits<double>::quiet_NaN(); // NEW member variable, init to NaN

  ///**
  // * @brief Simulation time for tracking mirror cycle progress
  // */
  ////double state_simTime = 0.0;
    
private:
    ///**
    // * @brief Simulation time for tracking mirror cycle progress
    // */
    double parCoefficient = 0.0; // parameter coefficient for the parabolic curve
    double minTiltAngle_deg = 0.0; // minimum tilt in degrees
    //double pulseTime = 0.0;
    //double facetPulseTime = 0.0;
    //double pulsesPerScanline = 0.0;
    //double facetFreq = 0.0;
    //double facetPulseFreq = 0.0;
    //int pulseFreq = 0;
    //double effAngleRatio = 0.0;
    /**
     * @brief Relative emitter attitude
     */
    Rotation cached_emitterSetRelativeAttitude = Rotation(glm::dvec3(1, 0, 0), 0);

public:
    // ***  CONSTRUCTION  / DESTRUCTION  *** //
    // ************************************* //
    /**
     * @brief Constructor for polygon mirror beam deflector
     * @see PolygonMirrorChannelBeamDeflector::cfg_device_scanAngleEffectiveMax_rad
     * @see AbstractBeamDeflector::AbstractBeamDeflector(
     *  double, double, double)
     */
	PolygonMirrorNFBBeamDeflector(
		double _scanFreqMax_Hz,
		double _scanFreqMin_Hz,
		double _scanAngleMax_rad,
		double _scanAngleEffectiveMax_rad
    ):
    AbstractBeamDeflector(_scanAngleMax_rad, _scanFreqMax_Hz, _scanFreqMin_Hz)
{
    this->cfg_device_scanAngleEffectiveMax_rad = _scanAngleEffectiveMax_rad;
    this->cfg_device_scanAngleEffective_rad = _scanAngleEffectiveMax_rad;
}
    std::shared_ptr<AbstractBeamDeflector> clone() override;
    void _clone(std::shared_ptr<AbstractBeamDeflector> abd) override;

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @see AbstractBeamDeflector::applySettings
     */
    void applySettings(std::shared_ptr<ScannerSettings>) override;
    
    /**
     * @see AbstractBeamDeflector::doSimStep
     */
    void doSimStep() override;

    /**
	 * @see AbstractBeamDeflector::getOpticsType
	 */
    std::string getOpticsType() const override {
        return "POLYGON_NFB_MIRROR";
    }

	/**
	 * @see AbstractBeamDeflector::lastPulseLeftDevice
	 */
	bool lastPulseLeftDevice() override;

	/**
	 * @brief Obtain the maximum effective scan angle in radians.
	 * @see PolygonMirrorNFBBeamDeflector::cfg_device_scanAngleEffectiveMax_rad
	 */
	double getScanAngleEffectiveMax_rad() const {
    return cfg_device_scanAngleEffectiveMax_rad;
  }

  /**
   * @see AbstractBeamDeflector::getAcrossTrackAngle_rad
   */
  double getAcrossTrackAngle_rad() const override {
    return this->state_currentBeamAngle_rad;
  }
  /**
   * @see AbstractBeamDeflector::getDownTrackAngle_rad
   */
  double getDownTrackAngle_rad() const override {
    return this->state_currentForwardAngle_rad;
  }
};
