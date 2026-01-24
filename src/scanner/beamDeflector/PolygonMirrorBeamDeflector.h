#pragma once

#include "AbstractBeamDeflector.h"

/**
 * @brief Class representing a polygon mirror beam deflector
 */
class PolygonMirrorBeamDeflector : public AbstractBeamDeflector
{

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

public:
  // ***  CONSTRUCTION  / DESTRUCTION  *** //
  // ************************************* //
  /**
   * @brief Constructor for polygon mirror beam deflector
   * @see PolygonMirrorBeamDeflector::cfg_device_scanAngleEffectiveMax_rad
   * @see AbstractBeamDeflector::AbstractBeamDeflector(
   *  double, double, double)
   */
  PolygonMirrorBeamDeflector(double _scanFreqMax_Hz,
                             double _scanFreqMin_Hz,
                             double _scanAngleMax_rad,
                             double _scanAngleEffectiveMax_rad)
    : AbstractBeamDeflector(_scanAngleMax_rad, _scanFreqMax_Hz, _scanFreqMin_Hz)
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
  std::string getOpticsType() const override { return "POLYGON_MIRROR"; }
  /**
   * @see AbstractBeamDeflector::lastPulseLeftDevice
   */
  bool lastPulseLeftDevice() override;
  /**
   * @see AbstractBeamDeflector::getAcrossTrackAngle_rad
   */
  double getAcrossTrackAngle_rad() const override
  {
    return this->state_currentBeamAngle_rad;
  }
  /**
   * @see AbstractBeamDeflector::getDownTrackAngle_rad
   */
  double getDownTrackAngle_rad() const override
  {
    // This simple deflector model has no down-track motion.
    return 0.0;
  }
  /**
   * @brief Obtain the maximum effective scan angle in radians.
   * @see PolygonMirrorBeamDeflector::cfg_device_scanAngleEffectiveMax_rad
   */
  double getScanAngleEffectiveMax_rad() const
  {
    return cfg_device_scanAngleEffectiveMax_rad;
  }
};
