#include <NewEnergyModel.h>
#include <maths/EnergyMaths.h>
#include <scanner/ScanningDevice.h>
#include <scanner/detector/AbstractDetector.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
NewEnergyModel::NewEnergyModel(ScanningDevice const& sd)
  : BaseEnergyModel(sd)
  , radii(sd.FWF_settings.beamSampleQuality + 1)
  , radiiSquared(sd.FWF_settings.beamSampleQuality + 1)
  , targetAreaCache(sd.FWF_settings.beamSampleQuality)
  , wSquaredCoefficient((sd.cached_halfDivergence_rad / sd.beamQuality) *
                        (sd.cached_halfDivergence_rad / sd.beamQuality))
  , ringEmittedPower(sd.FWF_settings.beamSampleQuality)
{
  // Cached radii. BSQ-1 denominator (ring-extent fix): makes the existing
  // outermost subray land exactly on cached_halfDivergence_rad instead of
  // falling short of it, at zero added subray cost. Guarded for BSQ=1 --
  // safe even at 0.0, since radii[1] is set directly below in that case.
  int const BSQ = sd.FWF_settings.beamSampleQuality;
  double const radiusStep_rad =
    (BSQ > 1) ? sd.cached_halfDivergence_rad / (BSQ - 1) : 0.0;
  radii[0] = 0.0;
  radiiSquared[0] = 0.0;
  for (int i = 0; i < BSQ; ++i) {
    int const subraysAtRing = (i == 0) ? 1 : (int)(i * PI_2);
    // Outer-boundary clamp: the last ring's outer boundary is clamped
    // directly to cached_halfDivergence_rad rather than left at
    // (i+0.5)*radiusStep_rad, which would overshoot the true edge (worse
    // at low BSQ) and make total captured energy vary with BSQ instead of
    // the fixed 1-exp(-2*beamQuality^2) it should be at every BSQ. This is
    // also the i=0 case at BSQ=1 (radii[1] set directly, not left at 0).
    radii[i + 1] = (i == BSQ - 1) ? sd.cached_halfDivergence_rad
                                  : (i + 0.5) * radiusStep_rad;
    radiiSquared[i + 1] = radii[i + 1] * radii[i + 1];
    targetAreaCache[i] = M_PI / ((double)subraysAtRing);

    // R^2 cancels completely between the (unscaled, angular) boundary
    // radii and wSquared(R) = R^2 * wSquaredCoefficient, so this exp-
    // difference is already the exact, final, range-independent
    // per-subray share of averagePower_w for this ring -- reuses the same
    // helper ImprovedEnergyModel calls per-pulse, but only once, here, at
    // construction time. Prefactor is the already-reduced form directly
    // (averagePower_w/subraysAtRing), matching the exact simplification
    // the elliptical-LUT builder's own comment (ScanningDevice.h) already
    // established -- no w0/totPower round-trip.
    ringEmittedPower[i] = EnergyMaths::calcSubrayWiseEmittedPowerFast(
      sd.averagePower_w / (double)subraysAtRing,
      wSquaredCoefficient,
      -2.0 * radiiSquared[i + 1], // outer boundary
      -2.0 * radiiSquared[i]);   // inner boundary
  }
}

// ***  METHODS  *** //
// ***************** //
double
NewEnergyModel::computeIntensity(
  double const incidenceAngle,
  double const targetRange,
  Material const& mat,
  int const subrayRadiusStep
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  NewReceivedPowerArgs args = NewReceivedPowerArgs(
    targetRange, incidenceAngle, mat, subrayRadiusStep);
  return computeReceivedPower(args
#if DATA_ANALYTICS >= 2
                              ,
                              calcIntensityRecords
#endif
  );
}

double
NewEnergyModel::computeReceivedPower(
  ModelArg const& _args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  NewReceivedPowerArgs const& args =
    static_cast<NewReceivedPowerArgs const&>(_args);
  // Pre-computations
  double const rangeSquared = args.targetRange * args.targetRange;
  // Emitted power (range-independent -- no range info passed at all)
  double const Pe = computeEmittedPower(
    NewEmittedPowerArgs{ args.subrayRadiusStep });
  // Target area
  double const targetArea = computeTargetArea(
    NewTargetAreaArgs{ rangeSquared, args.subrayRadiusStep }
#if DATA_ANALYTICS >= 2
    ,
    calcIntensityRecords
#endif
  );
  // Cross-section (inherited from BaseEnergyModel -- no wavelength/range
  // dependency to begin with)
  double const bdrf =
    EnergyMaths::computeBDRF(args.material, args.incidenceAngle_rad);
  double const sigma = computeCrossSection(
    BaseCrossSectionArgs{ args.material, bdrf, targetArea });
  // Received power
  double const atmosphericFactor = EnergyMaths::calcAtmosphericFactor(
    args.targetRange, sd.atmosphericExtinction);
  double const receivedPower =
    EnergyMaths::calcReceivedPowerImprovedFast(Pe,
                                               sd.cached_Dr2,
                                               16 * targetArea * rangeSquared,
                                               sd.efficiency,
                                               atmosphericFactor,
                                               sigma);
#if DATA_ANALYTICS >= 2
  std::vector<double>& calcIntensityRecord = calcIntensityRecords.back();
  calcIntensityRecord[3] = args.incidenceAngle_rad;
  calcIntensityRecord[4] = args.targetRange;
  calcIntensityRecord[5] = targetArea;
  calcIntensityRecord[7] = bdrf;
  calcIntensityRecord[8] = sigma;
  calcIntensityRecord[9] = receivedPower;
  calcIntensityRecord[10] = 0; // By default, assume the point isn't captured
  calcIntensityRecord[11] = Pe;
  calcIntensityRecord[12] = args.subrayRadiusStep;
#endif
  return receivedPower * 1e09;
}

double
NewEnergyModel::computeEmittedPower(ModelArg const& _args)
{
  NewEmittedPowerArgs const& args =
    static_cast<NewEmittedPowerArgs const&>(_args);
  // Fully precomputed at construction time -- O(1) lookup, zero trig/exp
  // work per call.
  return ringEmittedPower[args.subrayRadiusStep];
}

double
NewEnergyModel::computeTargetArea(
  ModelArg const& _args
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords
#endif
)
{
  // Byte-identical logic to ImprovedEnergyModel::computeTargetArea, just
  // reading NewEnergyModel's own radiiSquared/targetAreaCache.
  NewTargetAreaArgs const& args =
    static_cast<NewTargetAreaArgs const&>(_args);
  double targetArea;
  double radius_m_squared_for_record = 0.0;
  {
    double const prevRadiusSquared = radiiSquared[args.subrayRadiusStep];
    double const radiusSquared = radiiSquared[args.subrayRadiusStep + 1];
    double const radius_m_squared = radiusSquared * args.targetRangeSquared;
    double const prevRadius_m_squared =
      prevRadiusSquared * args.targetRangeSquared;
    targetArea = (radius_m_squared - prevRadius_m_squared) *
                 targetAreaCache[args.subrayRadiusStep];
#if DATA_ANALYTICS >= 2
    radius_m_squared_for_record = radius_m_squared;
#endif
  }
#if DATA_ANALYTICS >= 2
  std::vector<double> calcIntensityRecord(
    13, std::numeric_limits<double>::quiet_NaN());
  calcIntensityRecord[6] = std::sqrt(radius_m_squared_for_record);
  calcIntensityRecords.push_back(calcIntensityRecord);
#endif
  return targetArea;
}

double
NewEnergyModel::computeIntensityFromSigma(double const targetRange,
                                          double const sigma,
                                          int const subrayRadiusStep)
{
  double const rangeSquared = targetRange * targetRange;
  double const Pe = computeEmittedPower(
    NewEmittedPowerArgs{ subrayRadiusStep });
#if DATA_ANALYTICS >= 2
  std::vector<std::vector<double>> unusedRecords;
  double const targetArea = computeTargetArea(
    NewTargetAreaArgs{ rangeSquared, subrayRadiusStep },
    unusedRecords);
#else
  double const targetArea = computeTargetArea(
    NewTargetAreaArgs{ rangeSquared, subrayRadiusStep });
#endif
  double const atmosphericFactor =
    EnergyMaths::calcAtmosphericFactor(targetRange, sd.atmosphericExtinction);
  double const receivedPower = EnergyMaths::calcReceivedPowerImprovedFast(
    Pe, sd.cached_Dr2, 16 * targetArea * rangeSquared, sd.efficiency,
    atmosphericFactor, sigma);
  return receivedPower * 1e09;
}
