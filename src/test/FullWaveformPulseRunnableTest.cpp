#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <SingleScanner.h>
#include <Triangle.h>
#include <Vertex.h>
#include <maths/EnergyMaths.h>
#include <noise/UniformNoiseSource.h>
#include <scanner/SimulatedPulse.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>
#include <scene/Scene.h>

#include <algorithm>
#include <map>
#include <memory>
#include <vector>

namespace
{
// Test-only subclass exposing the protected computeSubrays for direct
// invocation, bypassing the full operator()/digestFullWaveform/
// exportOutput/Measurement pipeline -- this lets the test inspect the
// reflections map exactly as handleSubray populates it.
class TestableFullWaveformPulseRunnable : public FullWaveformPulseRunnable
{
public:
  TestableFullWaveformPulseRunnable(std::shared_ptr<Scanner> scanner,
                                    Scene& scene,
                                    SimulatedPulse const& pulse)
    : FullWaveformPulseRunnable(scanner, scene, pulse)
  {
    // Replicate the relevant part of AbstractPulseRunnable::initialize()
    // (normally invoked by operator()) since it is not called here.
    detector = scanner->getDetector(pulse.getDeviceIndex());
  }

  void runComputeSubrays(NoiseSource<double>& noiseSource,
                        std::map<double, double>& reflections,
                        std::vector<RaySceneIntersection>& intersects)
  {
    computeSubrays(noiseSource, reflections, intersects);
  }
};
}

TEST_CASE("FullWaveformPulseRunnable: same-distance subray accumulation")
{
  // Build a flat plane perpendicular to and centered on the boresight, far
  // enough away that the small footprint at that range fits comfortably
  // inside it. Subrays from different rings routinely land on the exact
  // same computed distance against such a plane (confirmed empirically:
  // 173 subrays collapse into only 12 distinct distance values), so this
  // scene reliably exercises the reflections map's same-distance handling.
  double const D = 50.0; // distance from scanner to plane (meters)
  double const R = 1.0;  // plane half-width (meters)
  Vertex v0, v1, v2, v3;
  v0.pos = glm::dvec3(-R, D, -R);
  v1.pos = glm::dvec3(R, D, -R);
  v2.pos = glm::dvec3(R, D, R);
  v3.pos = glm::dvec3(-R, D, R);
  auto material = std::make_shared<Material>();
  material->reflectance = 50.0;
  Triangle* t1 = new Triangle(v0, v1, v2);
  Triangle* t2 = new Triangle(v0, v2, v3);
  t1->material = material;
  t2->material = material;

  // A ScenePart must own the primitives -- Scene::buildKDGrove builds the
  // KD-Grove from Scene::parts (registered via Scene::registerParts, which
  // only picks up primitives with a non-null `part`), not from
  // Scene::primitives directly.
  std::shared_ptr<ScenePart> scenePart = std::make_shared<ScenePart>();
  scenePart->mId = "0";
  scenePart->mPrimitives.push_back(t1);
  scenePart->mPrimitives.push_back(t2);
  t1->part = scenePart;
  t2->part = scenePart;

  Scene scene;
  scene.primitives.push_back(t1);
  scene.primitives.push_back(t2);
  REQUIRE(scene.finalizeLoading());

  int const BSQ = 8;
  std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(
    0.0003,                             // beamDiv_rad
    glm::dvec3(0, 0, 0),                // beamOrigin
    Rotation(),                         // beamOrientation
    std::list<int>({ 100000 }),         // pulse freqs.
    5,                                  // pulseLength_ns
    "scanDev0",                         // id
    4.0,                                // averagePower_w
    1.0,                                // beamQuality
    0.99,                               // efficiency
    0.15,                               // receiverDiameter_m
    9.07603791e-6,                      // atmosphericVisibility_km
    1064,                               // wavelength_nm (constructor expects int nm)
    false,                              // Write waveform
    false,                              // Write pulse
    false,                              // Calc echowidth
    false,                              // Fullwave noise
    false,                              // Platform noise disabled
    nullptr                             // rangeErrExpr
  );
  std::shared_ptr<AbstractDetector> detector =
    std::make_shared<FullWaveformPulseDetector>(scanner,
                                                0.005, // accuracy_m
                                                0.01   // rangeMin_m
    );
  scanner->setDetector(detector);
  // SingleScanner's constructor truncates its integer wavelength_nm to
  // meters via /1e9; re-set it directly (in meters) to avoid precision
  // loss, matching the pattern used elsewhere in this test suite.
  scanner->setWavelength(1.064e-06);
  scanner->setAtmosphericExtinction(9.07603791e-6);
  ScanningDevice& scanDev = scanner->getScanningDevice(0);
  auto fwf = std::make_shared<FWFSettings>(scanDev.getFWFSettings());
  fwf->beamSampleQuality = BSQ;
  scanDev.setFWFSettings(fwf);
  scanner->prepareSimulation(false);

  // Pulse aimed straight at the plane's center (identity attitude points
  // along Directions::forward = +y). finalizeLoading() re-centers the scene
  // around its own centroid, so derive the origin from scene.getShift()
  // rather than assuming the plane lands at a fixed y.
  glm::dvec3 const shift = scene.getShift();
  SimulatedPulse pulse(glm::dvec3(0, -shift.y, 0), Rotation(), 0.0, 0, 0, 0);
  TestableFullWaveformPulseRunnable runnable(scanner, scene, pulse);

  UniformNoiseSource<double> noiseSource;
  std::map<double, double> reflections;
  std::vector<RaySceneIntersection> intersects;
  runnable.runComputeSubrays(noiseSource, reflections, intersects);

  int const totalSubrays = (int)scanDev.cached_subrayRotation.size();
  REQUIRE((int)intersects.size() == totalSubrays);

  // Distinct exact (bit-for-bit) distances are far fewer than the number of
  // subrays cast: multiple subrays -- from the same ring and, due to
  // floating-point rounding, sometimes across rings -- land on the exact
  // same computed distance against this flat perpendicular plane. This is
  // precisely the scenario handleSubray's reflections map must accumulate
  // rather than silently drop.
  REQUIRE(reflections.size() < intersects.size());

  // Total energy conservation: since every one of the totalSubrays subrays
  // hits the plane, the sum of all values ultimately stored in reflections
  // must equal the independently-computed sum of every individual subray's
  // emitted/received intensity, regardless of how they group by distance.
  // Before the fix (plain insert), colliding subrays were silently dropped
  // and this sum would come out too low.
  double expectedGrandTotal = 0.0;
  for (int i = 0; i < BSQ; ++i) {
    int const ringCount = (int)std::count(
      scanDev.cached_subrayRadiusStep.begin(),
      scanDev.cached_subrayRadiusStep.end(),
      i);
    double const angle = scanDev.cached_subrayDivergenceAngle_rad[i];
    double const dist = D / std::cos(angle);
    double const perSubrayIntensity =
      scanner->calcIntensity(0.0, dist, *material, i, 0);
    expectedGrandTotal += ringCount * perSubrayIntensity;
  }

  double actualGrandTotal = 0.0;
  for (auto const& [dist, intensity] : reflections) {
    actualGrandTotal += intensity;
  }

  double const absDiff = std::fabs(actualGrandTotal - expectedGrandTotal);
  REQUIRE(absDiff <= expectedGrandTotal * 1e-9);
}
