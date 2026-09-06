#pragma once

#include <maths/model/BaseEnergyModel.h>

#include <vector>

// ***  ARGUMENT CLASSES  *** //
// ************************** //
class NewReceivedPowerArgs : public ModelArg
{
public:
  double const targetRange;
  double const incidenceAngle_rad;
  Material const& material;
  int const subrayRadiusStep;
  NewReceivedPowerArgs(double const targetRange,
                       double const incidenceAngle_rad,
                       Material const& material,
                       int const subrayRadiusStep)
    : targetRange(targetRange)
    , incidenceAngle_rad(incidenceAngle_rad)
    , material(material)
    , subrayRadiusStep(subrayRadiusStep)
  {
  }
};

class NewEmittedPowerArgs : public ModelArg
{
public:
  int const subrayRadiusStep;
  NewEmittedPowerArgs(int const subrayRadiusStep)
    : subrayRadiusStep(subrayRadiusStep)
  {
  }
};

class NewTargetAreaArgs : public ModelArg
{
public:
  double const targetRangeSquared;
  int const subrayRadiusStep;
  NewTargetAreaArgs(double const targetRangeSquared,
                    int const subrayRadiusStep)
    : targetRangeSquared(targetRangeSquared)
    , subrayRadiusStep(subrayRadiusStep)
  {
  }
};

// ***  NEW ENERGY MODEL CLASS  *** //
// ******************************** //
/**
 * @brief Hybrid energy model combining ImprovedEnergyModel's exact
 *  per-ring/per-subray energy partitioning and elliptical-LUT
 *  compatibility with a purely far-field emitted-power formula, dropping
 *  the near-field term entirely instead of approximating it.
 *
 * Unlike BaseEnergyModel/ImprovedEnergyModel, this model has no
 * beamWaistRadius/wavelength_m/rangeMin dependency at all -- the near-field
 * term in both existing models (BaseEnergyModel's R0^2+R^2 additive term,
 * ImprovedEnergyModel's Omega0=1-R/R0 term) either converges to matching
 * the geometric ring cone only asymptotically (Base) or never converges at
 * all (Improved, a confirmed, non-vanishing ~17.5% mismatch at every
 * range). Dropping the term entirely makes the emitted-power Gaussian's
 * width exactly `w(R) = R * cached_halfDivergence_rad / beamQuality` --
 * matching the geometric ring cone exactly, at every range including
 * R=rangeMin, not just asymptotically, because both sides read the same
 * corrected half-angle quantity directly, with nothing else mixed in.
 * `lambda`, `w0`, and `rangeMin` drop out of the emitted-power computation
 * entirely -- they cancel algebraically regardless of beamQuality's value.
 *
 * This also sidesteps the (separately confirmed, but orthogonal) half/
 * full-angle beamDivergence_rad ambiguity: this model never separately
 * interprets what cached_halfDivergence_rad *means*, only that geometry
 * and energy read the identical number.
 */
class NewEnergyModel : public BaseEnergyModel
{
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Precomputed angular ring boundaries (in radians, not yet scaled
   *  by range) -- same construction as ImprovedEnergyModel::radii,
   *  including the BSQ-1 denominator and outer-boundary clamp (see
   *  ScanningDevice::prepareSimulation's own matching construction).
   */
  std::vector<double> radii;
  /**
   * @brief The values of NewEnergyModel::radii but squared.
   */
  std::vector<double> radiiSquared;
  /**
   * @brief Precompute part of the target area, same as
   *  ImprovedEnergyModel::targetAreaCache: \f$\pi/n_{sr}\f$.
   */
  std::vector<double> targetAreaCache;
  /**
   * @brief The range-independent coefficient such that
   *  \f$w^2(R) = R^2 \cdot \mathrm{wSquaredCoefficient}\f$:
   *
   * \f[
   *  \mathrm{wSquaredCoefficient} =
   *    \left(\frac{\varphi_{1/2}}{\mathrm{BQ}}\right)^2
   * \f]
   *
   * Where \f$\varphi_{1/2}\f$ is ScanningDevice::cached_halfDivergence_rad
   *  and \f$\mathrm{BQ}\f$ is the beam quality factor. No beam waist,
   *  wavelength, or rangeMin dependency at all -- unlike
   *  ImprovedEnergyModel's w0Squared/omegaCacheSquared pair, this is the
   *  one and only quantity the emitted-power formula needs beyond the
   *  ring geometry itself.
   */
  double const wSquaredCoefficient;
  /**
   * @brief Fully baked per-subray emitted power for each ring, computed
   *  once in the constructor -- since R^2 cancels completely between the
   *  (unscaled, angular) ring boundaries and
   *  \f$w^2(R) = R^2 \cdot \mathrm{wSquaredCoefficient}\f$, this is
   *  already the exact, final, range-independent value, not a prefactor
   *  needing a further per-call exponential difference the way
   *  ImprovedEnergyModel::deviceConstantExpression is.
   */
  std::vector<double> ringEmittedPower;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @see EnergyModel::EnergyModel
   */
  NewEnergyModel(ScanningDevice const& sd);

  // ***  METHODS  *** //
  // ***************** //
  /**
   * @see BaseEnergyModel::computeIntensity
   */
  double computeIntensity(double const incidenceAngle,
                          double const targetRange,
                          Material const& mat,
                          int const subrayRadiusStep
#if DATA_ANALYTICS >= 2
                          ,
                          std::vector<std::vector<double>>& calcIntensityRecords
#endif
                          ) override;
  /**
   * @see EnergyModel::computeReceivedPower
   * @see NewReceivedPowerArgs
   */
  double computeReceivedPower(
    ModelArg const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
    ) override;
  /**
   * @brief Compute the emitted power \f$P_e\f$. An O(1) lookup into
   *  NewEnergyModel::ringEmittedPower -- zero exp/trig calls per pulse.
   * @see EnergyModel::computeEmittedPower
   */
  double computeEmittedPower(ModelArg const& args) override;
  /**
   * @brief Compute the target area \f$A\f$. Byte-identical logic to
   *  ImprovedEnergyModel::computeTargetArea, reading NewEnergyModel's own
   *  radii/radiiSquared/targetAreaCache.
   * @see EnergyModel::computeTargetArea
   */
  double computeTargetArea(
    ModelArg const& args
#if DATA_ANALYTICS >= 2
    ,
    std::vector<std::vector<double>>& calcIntensityRecords
#endif
    ) override;
  /**
   * @brief Compute the intensity from a precomputed cross-section
   *  \f$\sigma\f$ (LadLut path). Unlike BaseEnergyModel's implementation
   *  (inherited by ImprovedEnergyModel unchanged), this references neither
   *  wavelength_m, beamWaistRadius, nor rangeMin anywhere -- a
   *  compiler-checked confirmation of this model's "no beam-waist
   *  dependency" design goal.
   * @see EnergyModel::computeIntensityFromSigma
   */
  double computeIntensityFromSigma(
    double const targetRange,
    double const sigma,
    int const subrayRadiusStep) override;
};
