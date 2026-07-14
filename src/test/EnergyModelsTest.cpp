#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <SingleScanner.h>
#include <maths/EnergyMaths.h>
#include <scanner/detector/FullWaveformPulseRunnable.h>

TEST_CASE("Energy Models Test ")
{
  double const eps = 0.00001;

  SECTION("Test Emitted Received Power")
  {
    // Generate values for the tests
    // Format: I0, lambda, R, R0, r, w0, Dr2, Bt2, etaSys, ae, sigma
    auto [I0, lambda, R, R0, r, w0, Dr2, Bt2, etaSys, ae, sigma] = GENERATE(
      std::make_tuple(450., 0.3, 7.5, 5.0, 0.33, 0.9, 0.9, 0.7, 1.1, 0.0, 3.0),
      std::make_tuple(575., 0.6, 6.3, 9.0, 0.67, 0.6, 0.8, 0.6, 0.3, 0.05, 0.5),
      std::make_tuple(
        250., 0.0001, 1.0, 0.1, 0.09, 0.05, 0.1, 0.5, 0.6, 0.1, 0.4),
      std::make_tuple(75., 0.001, 5.0, 0.5, 13., 0.1, 0.2, 0.4, 0.9, 0.15, 0.3),
      std::make_tuple(
        370., 0.005, 10.0, 1.0, 0.55, 0.3, 0.7, 0.05, 1.2, 0.23, 0.6),
      std::make_tuple(
        40., 0.01, 15.0, 5.0, 0.26, 1.0, 0.3, 0.1, 1.0, 0.17, 0.7),
      std::make_tuple(
        30., 0.05, 20.0, 10.0, 0.19, 1.5, 0.6, 0.2, 0.5, 0.09, 1.5),
      std::make_tuple(
        900., 0.1, 30.0, 15.0, 7.4, 1.3, 0.5, 0.3, 0.1, 0.009, 2.0));

    // Compute tests
    // Test emitted power for i-th case
    double const PeNew =
      EnergyMaths::calcEmittedPower(I0, lambda, R, R0, r, w0);
    double const PeOld =
      EnergyMaths::calcEmittedPowerLegacy(I0, lambda, R, R0, r, w0);
    REQUIRE(std::fabs(PeNew - PeOld) <= eps);
    // Test received power for i-th case
    double const PrNew = EnergyMaths::calcReceivedPower(
      I0, lambda, R, R0, r, w0, Dr2, Bt2, etaSys, ae, sigma);
    double const PrOld = EnergyMaths::calcReceivedPowerLegacy(
      PeOld,
      Dr2,
      R,
      Bt2,
      etaSys,
      EnergyMaths::calcAtmosphericFactor(R, ae),
      sigma);
    REQUIRE(std::fabs(PrNew - PrOld) <= eps);
  }

  SECTION("Test Emitted Subray Wise Power")
  {
    // Values for the tests
    double const I0 = 499504.174245084;
    double const beamDiv = 0.0003;
    double const w = 0.01288227007085636;
    double const w0 = 0.0022578781259970223;
    int const BSQ = 8;
    // Representative target range used to turn the angular ring boundaries
    // below into physical radii (in meters), consistent with how
    // ImprovedEnergyModel::computeEmittedPower scales its cached angular
    // radii by the target range before comparing them against w (also in
    // meters).
    double const R = 100.0;

    auto [i, nsr, expectedPe] =
      GENERATE(std::make_tuple(0, 1, 0.16593573132947803),
               std::make_tuple(1, 6, 0.18370477237088903),
               std::make_tuple(2, 12, 0.11207803019801045),
               std::make_tuple(3, 18, 0.04917861329661144),
               std::make_tuple(4, 25, 0.014895218824846368),
               std::make_tuple(5, 31, 0.0034050951259272815),
               std::make_tuple(6, 37, 0.0005578025027892914),
               std::make_tuple(7, 43, 6.552178388162786e-05));

    // Ring boundaries are centered on the same angular step used to cast
    // each ring's subrays in ScanningDevice::prepareSimulation
    // (beamDiv / BSQ), then scaled to a physical radius via R.
    double const radiusStep_rad = beamDiv / BSQ;
    double const angle = (i + 0.5) * radiusStep_rad;
    double const prevAngle = (i == 0) ? 0.0 : (i - 0.5) * radiusStep_rad;
    double const radius = R * angle;
    double const prevRadius = R * prevAngle;
    double const Pe = EnergyMaths::calcSubrayWiseEmittedPower(
      I0, w0, w, radius, prevRadius, nsr);
    double const absDiff = std::fabs(Pe - expectedPe);
    REQUIRE(absDiff <= eps);
  }

  SECTION("Test Elliptical Footprint Energy")
  {
#if DATA_ANALYTICS < 1
    // Prepare fake scanner
    std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(
      0.0003,                             // beamDiv_rad
      glm::dvec3(0, 0, 0),                // beamOrigin,
      Rotation(),                         // beamOrientation
      std::list<int>({ 100000, 300000 }), // pulse freqs.
      5,                                  // pulseLength_ns
      "scanDev0",                         // id
      4.0,                                // averagePower_w
      1.0,                                // beamQuality
      0.98999999999999,                   // efficiency
      0.15,                               // receiverDiameter_m
      9.07603791e-6,                      // atmosphericExtinction
      1.064e-06,                          // wavelength_m
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
    // Configure scanner to match expected values
    scanner->setDetector(detector);
    scanner->setNumRays(37);
    scanner->setWavelength(1064e-06);
    scanner->setBeamWaistRadius(0.0011289390629985112);
    scanner->setAtmosphericExtinction(9.07603791e-6);
    scanner->prepareSimulation(false);
    // Get reference to scanning device
    ScanningDevice& scanDev = scanner->getScanningDevice(0);

    Material material;
    material.isGround = false;
    material.useVertexColors = false;
    material.map_Kd = "";
    material.reflectance = 0.5;
    material.specularity = 0;
    material.specularExponent = 96.078430999999;
    material.classification = 0;
    material.spectra = "shingle_red";
    float ka[4] = { 1, 1, 1, 0 };
    std::copy(std::begin(ka), std::end(ka), std::begin(material.ka));
    float kd[4] = { 0.639999986, 0.639999986, 0.639999986, 0 };
    std::copy(std::begin(kd), std::end(kd), std::begin(material.kd));
    float ks[4] = { 0, 0, 0, 0 };
    std::copy(std::begin(ks), std::end(ks), std::begin(material.ks));

    auto [raytracingRange, incidenceAngle, divergenceAngle, expectedIntensity] =
      GENERATE(
        std::make_tuple(0.1, 0.0, 0.0003, 0.0016944653501892728),
        std::make_tuple(0.5, 0.01, 0.00003, 5.7162595762529559e-05),
        std::make_tuple(1.0, 0.036, 0.000003, 1.3994704142630095e-05),
        std::make_tuple(1.3, 0.1, 0.0000003, 8.2082244537851853e-06),
        std::make_tuple(1.5, 0.12, 0.003, 6.1379499615213924e-06),
        std::make_tuple(2.1, 0.0, 0.0001, 3.1428056176348065e-06),
        std::make_tuple(3.7, 0.1, 0.00001, 1.0030793949400564e-06),
        std::make_tuple(5.0, 0.2, 0.000001, 5.4026593416462289e-07),
        std::make_tuple(10.0, 0.3, 0.0000001, 1.3138666190414419e-07),
        std::make_tuple(22.3, 0.4, 0.001, 2.5438162975634665e-08),
        std::make_tuple(28.8, 0.5, 0.0002, 1.4525665701644939e-08),
        std::make_tuple(30.9, 0.4, 0.00002, 1.3243046756709799e-08),
        std::make_tuple(51.7, 0.2, 0.000002, 5.0304190149318903e-09),
        std::make_tuple(123.2, 0.1, 0.0000002, 8.9793855544322227e-10),
        std::make_tuple(900.318, 0.0, 0.002, 1.6661917508151922e-11),
        std::make_tuple(1500.1, 1.5, 0.015, 4.1994847649407481e-13),
        std::make_tuple(1550., 1.3, 0.0015, 1.4861192718438113e-12),
        std::make_tuple(1999.0, 1.2, 0.00015, 1.2005148230934354e-12),
        std::make_tuple(1999.99, 1.0, 0.000015, 1.7882528400488737e-12),
        std::make_tuple(2900., 0.8, 0.15, 1.0789632615680178e-12));
    for (size_t j = 0; j < scanDev.cached_subrayDivergenceAngle_rad.size();
         ++j) {
      scanDev.cached_subrayDivergenceAngle_rad[j] = divergenceAngle;
    }
    double const intensity =
      scanner->calcIntensity(incidenceAngle, raytracingRange, material, 0, 0);
    REQUIRE(!isnan(intensity));
    double const absDiff = std::fabs(expectedIntensity - intensity) * 1e09;
    REQUIRE(absDiff <= eps);
#endif
  }

  SECTION("Test Ring Cast Angles Match Beam Sample Quality Step")
  {
    // ImprovedEnergyModel's ring boundaries are derived from
    // beamDivergence_rad / beamSampleQuality (see its constructor). This
    // checks that ScanningDevice::prepareSimulation actually casts each
    // ring's subrays at that same angular step, so the two cannot silently
    // drift apart again.
    int const beamSampleQuality = GENERATE(3, 8);
    double const beamDiv = 0.0003;
    std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(
      beamDiv,
      glm::dvec3(0, 0, 0),
      Rotation(),
      std::list<int>({ 100000 }),
      5,
      "scanDev0",
      4.0,
      1.0,
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
    std::shared_ptr<AbstractDetector> detector =
      std::make_shared<FullWaveformPulseDetector>(scanner, 0.005, 0.01);
    scanner->setDetector(detector);
    ScanningDevice& scanDev = scanner->getScanningDevice(0);
    auto fwf = std::make_shared<FWFSettings>(scanDev.getFWFSettings());
    fwf->beamSampleQuality = beamSampleQuality;
    scanDev.setFWFSettings(fwf);
    scanner->prepareSimulation(false);

    double const expectedStep = beamDiv / beamSampleQuality;
    REQUIRE(scanDev.cached_subrayDivergenceAngle_rad.size() ==
            (size_t)beamSampleQuality);
    for (int i = 0; i < beamSampleQuality; ++i) {
      double const expectedAngle = i * expectedStep;
      REQUIRE(std::fabs(scanDev.cached_subrayDivergenceAngle_rad[i] -
                        expectedAngle) <= eps);
    }
  }

  SECTION("Test Emitted Power Decreases Monotonically Beyond The Peak Ring")
  {
    // Regression check for the units/ring-boundary fixes in
    // ImprovedEnergyModel: with a physically correct Gaussian falloff, the
    // annulus nearest the peak of r*exp(-2r^2/w^2) (ring 1) may legitimately
    // carry more per-subray energy than the tiny central ring (ring 0), but
    // from there outward the exponential decay must dominate and per-subray
    // emitted power must decrease monotonically. Before the fix this was
    // not the case (it oscillated/increased outward instead).
    double const I0 = 499504.174245084;
    double const beamDiv = 0.0003;
    double const w = 0.01288227007085636;
    double const w0 = 0.0022578781259970223;
    int const BSQ = 8;
    double const R = 100.0;
    double const radiusStep_rad = beamDiv / BSQ;
    int const nsr[BSQ] = { 1, 6, 12, 18, 25, 31, 37, 43 };

    double Pe[BSQ];
    for (int i = 0; i < BSQ; ++i) {
      double const angle = (i + 0.5) * radiusStep_rad;
      double const prevAngle = (i == 0) ? 0.0 : (i - 0.5) * radiusStep_rad;
      Pe[i] = EnergyMaths::calcSubrayWiseEmittedPower(
        I0, w0, w, R * angle, R * prevAngle, nsr[i]);
    }
    for (int i = 2; i < BSQ; ++i) {
      REQUIRE(Pe[i] < Pe[i - 1]);
    }
  }

  SECTION("Test Number Of Generated Subrays Matches Reported Ray Count")
  {
    // ScanningDevice::calcRaysNumber() must report the same subray count
    // that ScanningDevice::prepareSimulation() actually generates.
    int const beamSampleQuality = GENERATE(1, 3, 8, 10);
    std::shared_ptr<Scanner> scanner = std::make_shared<SingleScanner>(
      0.0003,
      glm::dvec3(0, 0, 0),
      Rotation(),
      std::list<int>({ 100000 }),
      5,
      "scanDev0",
      4.0,
      1.0,
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
    std::shared_ptr<AbstractDetector> detector =
      std::make_shared<FullWaveformPulseDetector>(scanner, 0.005, 0.01);
    scanner->setDetector(detector);
    ScanningDevice& scanDev = scanner->getScanningDevice(0);
    auto fwf = std::make_shared<FWFSettings>(scanDev.getFWFSettings());
    fwf->beamSampleQuality = beamSampleQuality;
    scanDev.setFWFSettings(fwf);
    scanner->prepareSimulation(false);
    scanner->calcRaysNumber();

    REQUIRE(scanner->getNumRays() ==
            (int)scanDev.cached_subrayRotation.size());
  }
}
