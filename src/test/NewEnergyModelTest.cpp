#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO

#include <ScanningDevice.h>
#include <SingleScanner.h>
#include <maths/EnergyMaths.h>
#include <maths/model/NewEnergyModel.h>
#include <scanner/detector/FullWaveformPulseDetector.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

// Covers the NewEnergyModel plan's test plan §"Test plan
// (src/test/NewEnergyModelTest.cpp)": energy conservation (fixed capture
// fraction across BSQ, including BSQ=1), exact match to the geometric cone
// at every range, divergence from ImprovedEnergyModel's near-field defect,
// no wavelength_m/beamWaistRadius dependency, and the RiPARAMETER-confirmed
// geometric footprint.

namespace {

// Mirrors EnergyModelsTest.cpp's own SingleScanner construction convention
// exactly (same constructor argument shape/order), parameterized over just
// what each test case below varies. Deliberately does NOT call
// prepareSimulation() itself -- callers need to choose the model-selection
// flag first.
std::shared_ptr<SingleScanner>
makeScanner(double const beamDiv_rad,
           double const beamQuality,
           double const averagePower_w,
           double const rangeMin_m,
           int const beamSampleQuality)
{
  auto scanner = std::make_shared<SingleScanner>(beamDiv_rad,
                                                 glm::dvec3(0, 0, 0),
                                                 Rotation(),
                                                 std::list<int>({ 100000 }),
                                                 5.0,
                                                 "scanDev0",
                                                 averagePower_w,
                                                 beamQuality,
                                                 0.99,
                                                 0.15,
                                                 9.07603791e-6,
                                                 1.064e-06,
                                                 false,
                                                 false,
                                                 false,
                                                 false,
                                                 false,
                                                 nullptr);
  auto detector =
    std::make_shared<FullWaveformPulseDetector>(scanner, 0.005, rangeMin_m);
  scanner->setDetector(detector, 0);
  scanner->setAtmosphericExtinction(9.07603791e-6, 0);

  ScanningDevice& scanDev = scanner->getScanningDevice(0);
  auto fwf = std::make_shared<FWFSettings>(scanDev.getFWFSettings());
  fwf->beamSampleQuality = beamSampleQuality;
  scanDev.setFWFSettings(fwf);

  return scanner;
}

// computeTargetArea carries the same DATA_ANALYTICS>=2-guarded extra
// parameter as computeIntensity/computeReceivedPower -- this keeps direct
// calls to it (needed here to inspect target area independent of the
// combined calcIntensity path) compiling at every DATA_ANALYTICS level.
double
computeTargetArea(std::shared_ptr<EnergyModel> const& energyModel,
                  ModelArg const& args)
{
#if DATA_ANALYTICS >= 2
  std::vector<std::vector<double>> unusedRecords;
  return energyModel->computeTargetArea(args, unusedRecords);
#else
  return energyModel->computeTargetArea(args);
#endif
}

Material
makeMaterial(double const reflectance)
{
  Material material;
  material.reflectance = reflectance;
  return material;
}

} // namespace

TEST_CASE("New Energy Model Test")
{
  double const beamDiv_rad = 0.0004; // VUX-120-equivalent full angle
  double const beamQuality = 1.0;
  double const averagePower_w = 4.0;
  double const rangeMin_m = 0.01;

  SECTION("Energy conservation: total captured fraction is a fixed "
         "constant across BSQ")
  {
    int const BSQ = GENERATE(3, 5, 10);

    auto scanner =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, BSQ);
    scanner->prepareSimulation(false, true);
    ScanningDevice& scanDev = scanner->getScanningDevice(0);

    auto energyModel = scanDev.getEnergyModel();
    REQUIRE(energyModel != nullptr);

    // Sum the actual per-subray emitted power over every real generated
    // subray (cached_subrayRadiusStep has one entry per subray, giving its
    // ring index) -- exactly what the real per-pulse loop would accumulate,
    // not a hand-reimplemented ring-count formula.
    double totalCaptured = 0.0;
    for (int const ring : scanDev.cached_subrayRadiusStep) {
      totalCaptured +=
        energyModel->computeEmittedPower(NewEmittedPowerArgs{ ring });
    }

    double const expected =
      averagePower_w * (1.0 - std::exp(-2.0 * beamQuality * beamQuality));
    REQUIRE(std::fabs(totalCaptured - expected) <= 1e-9 * expected);
  }

  SECTION("BSQ=1 follows the same rule as every other BSQ, not a special "
         "value")
  {
    auto scanner =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, 1);
    scanner->prepareSimulation(false, true);
    ScanningDevice& scanDev = scanner->getScanningDevice(0);

    REQUIRE(scanDev.cached_subrayRadiusStep.size() == 1);
    REQUIRE(scanDev.cached_subrayRadiusStep[0] == 0);

    Material material = makeMaterial(50.0);
    double const R = 100.0;
    double const intensity = scanDev.calcIntensity(0.0, R, material, 0);
    REQUIRE_FALSE(std::isnan(intensity));
    REQUIRE_FALSE(std::isinf(intensity));
    REQUIRE(intensity > 0.0);

    auto energyModel = scanDev.getEnergyModel();
    REQUIRE(energyModel != nullptr);

    // Target area matches the full nominal footprint: pi * halfDivergence^2
    // * R^2 -- BSQ=1's single ring spans the entire cone, boundary clamped
    // directly to cached_halfDivergence_rad.
    double const targetArea = computeTargetArea(
      energyModel, NewTargetAreaArgs{ R * R, 0 });
    double const expectedTargetArea = M_PI *
                                      scanDev.cached_halfDivergence_rad *
                                      scanDev.cached_halfDivergence_rad *
                                      R * R;
    REQUIRE(std::fabs(targetArea - expectedTargetArea) <=
           1e-9 * expectedTargetArea);

    // Emitted power equals averagePower_w*(1-e^-2) -- the same 86.47%
    // figure as every other BSQ, not a special-cased 100%.
    double const emittedPower =
      energyModel->computeEmittedPower(NewEmittedPowerArgs{ 0 });
    double const expectedEmittedPower =
      averagePower_w * (1.0 - std::exp(-2.0 * beamQuality * beamQuality));
    REQUIRE(std::fabs(emittedPower - expectedEmittedPower) <=
           1e-9 * expectedEmittedPower);
  }

  SECTION("Exact match to the geometric cone at every range, including "
         "R=rangeMin")
  {
    int const BSQ = 5;
    auto scanner =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, BSQ);
    scanner->prepareSimulation(false, true);
    ScanningDevice& scanDev = scanner->getScanningDevice(0);

    Material material = makeMaterial(50.0);
    double const incidenceAngle_rad = 0.0;
    int const ring = 1;

    // Emitted power has no range dependency by construction, and target
    // area's R^2 cancels completely against sigma's own R^2 dependency (sigma
    // = 4*pi*bdrf*targetArea) inside calcReceivedPowerImprovedFast's
    // denominator (16*targetArea*R^2) -- so intensity(R)*R^2/atmos(R) is an
    // exact constant, not merely a close approximation, at every R.
    std::vector<double> normalizedValues;
    for (double const R :
        { rangeMin_m, 10.0 * rangeMin_m, 1000.0 * rangeMin_m }) {
      double const intensity =
        scanDev.calcIntensity(incidenceAngle_rad, R, material, ring);
      double const atmos =
        EnergyMaths::calcAtmosphericFactor(R, 9.07603791e-6);
      normalizedValues.push_back(intensity * R * R / atmos);
    }

    REQUIRE(normalizedValues.front() > 0.0);
    for (size_t i = 1; i < normalizedValues.size(); ++i) {
      REQUIRE(std::fabs(normalizedValues[i] - normalizedValues[0]) <=
             1e-9 * std::max(std::fabs(normalizedValues[i]),
                            std::fabs(normalizedValues[0])));
    }
  }

  SECTION("ImprovedEnergyModel's near-field term collapses at realistic "
         "ranges; NewEnergyModel stays well-behaved")
  {
    // ImprovedEnergyModel's Omega0=1-R/rangeMin near-field term grows
    // unboundedly with R (it was never designed to plateau, unlike
    // BaseEnergyModel's R0^2+R^2 term), driving its emitted power toward
    // degenerate territory -- either vanishing toward zero or, confirmed by
    // actually running this exact scenario, outright NaN from the extreme
    // exponential arguments involved -- at ranges well beyond rangeMin.
    // NewEnergyModel has no such term and stays at its fixed, range-
    // independent captured fraction, so the two diverge sharply (not
    // converge) at realistic ALS-scale range/rangeMin ratios. This is
    // exactly the defect NewEnergyModel exists to avoid; ImprovedEnergyModel's
    // own near-field formula is unrelated pre-existing behavior, deliberately
    // left untouched by this effort (see the plan's Context section), so
    // this test only asserts that NewEnergyModel avoids it -- not any
    // specific value for ImprovedEnergyModel's own (broken) output.
    int const BSQ = 5;
    auto scannerImproved =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, BSQ);
    scannerImproved->prepareSimulation(false, false);
    auto scannerNew =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, BSQ);
    scannerNew->prepareSimulation(false, true);

    ScanningDevice& devImproved = scannerImproved->getScanningDevice(0);
    ScanningDevice& devNew = scannerNew->getScanningDevice(0);

    Material material = makeMaterial(50.0);
    double const incidenceAngle_rad = 0.0;
    int const ring = 1;
    double const R = 1000.0 * rangeMin_m;

    double const intensityImproved =
      devImproved.calcIntensity(incidenceAngle_rad, R, material, ring);
    double const intensityNew =
      devNew.calcIntensity(incidenceAngle_rad, R, material, ring);

    REQUIRE(intensityNew > 0.0);
    REQUIRE_FALSE(std::isnan(intensityNew));
    bool const improvedIsDegenerate =
      std::isnan(intensityImproved) || intensityImproved < intensityNew * 0.01;
    REQUIRE(improvedIsDegenerate);
  }

  SECTION("Regression guard: no wavelength_m/beamWaistRadius dependency")
  {
    int const BSQ = 5;
    auto scannerA =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, BSQ);
    scannerA->prepareSimulation(false, true);

    auto scannerB =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, BSQ);
    scannerB->prepareSimulation(false, true);
    // Perturb wavelength_m/beamWaistRadius post-prepareSimulation, without
    // calling configureBeam() afterward. NewEnergyModel's ring
    // geometry/emitted power were already fully baked
    // at construction time and never reference these fields again, so this
    // must have no effect on its output.
    scannerB->setWavelength(0.0, 0);
    scannerB->setBeamWaistRadius(1e12, 0);

    ScanningDevice& devA = scannerA->getScanningDevice(0);
    ScanningDevice& devB = scannerB->getScanningDevice(0);

    Material material = makeMaterial(50.0);
    double const incidenceAngle_rad = 0.1;
    double const targetRange_m = 75.0;
    int const ring = 1;

    double const intensityA =
      devA.calcIntensity(incidenceAngle_rad, targetRange_m, material, ring);
    double const intensityB =
      devB.calcIntensity(incidenceAngle_rad, targetRange_m, material, ring);

    REQUIRE(std::fabs(intensityA - intensityB) <=
           1e-9 * std::max(std::fabs(intensityA), std::fabs(intensityB)));
  }

  SECTION("Geometric footprint matches the RiPARAMETER-confirmed value "
         "exactly, at any BSQ>1")
  {
    // VUX-120-equivalent: beamDiv_rad=0.0004 (full angle), nadir, H=500m --
    // RiPARAMETER's own measured 1/e^2 footprint diameter at this height is
    // 0.20m, i.e. H*beamDiv_rad, confirmed empirically earlier in this
    // effort. Thanks to the ring-extent fix (BSQ-1 denominator), the
    // outermost subray lands exactly on cached_halfDivergence_rad for any
    // BSQ>1, not just asymptotically -- assert at more than one BSQ to
    // confirm the exactness, not mere convergence.
    int const BSQ = GENERATE(3, 10);
    double const H = 500.0;

    auto scanner =
      makeScanner(beamDiv_rad, beamQuality, averagePower_w, rangeMin_m, BSQ);
    scanner->prepareSimulation(false, true);
    ScanningDevice& scanDev = scanner->getScanningDevice(0);

    double const outermostAngle_rad =
      scanDev.cached_subrayDivergenceAngle_rad.back();
    double const diameter_m = 2.0 * outermostAngle_rad * H;
    double const expectedDiameter_m = H * beamDiv_rad;

    REQUIRE(std::fabs(diameter_m - expectedDiameter_m) <=
           1e-9 * expectedDiameter_m);
  }
}
