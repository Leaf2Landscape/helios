// In FullWaveformPulseRunnable.cpp

void
FullWaveformPulseRunnable::digestFullWaveform(
  std::vector<Measurement>& pointsMeasurement,
  int& numReturns,
  std::vector<std::vector<double>>& apMatrix,
  std::vector<double> const& fullwave,
  vector<RaySceneIntersection> const& intersects,
  glm::dvec3 const& beamDir,
  double const nsPerBin,
  int const numFullwaveBins,
  int const peakIntensityIndex,
  double const minHitTime_ns
#if DATA_ANALYTICS >= 2
  ,
  std::vector<std::vector<double>>& calcIntensityRecords,
  std::vector<std::vector<int>>& calcIntensityIndices,
  std::shared_ptr<HDA_PulseRecorder> pulseRecorder
#endif
)
{
  // Extract points from waveform data via Gaussian decomposition
  size_t const devIdx = pulse.getDeviceIndex();
  numReturns = 0;
  int const win_size =
    (int)(scanner->getFWFSettings(pulse.getDeviceIndex()).winSize_ns /
          nsPerBin);
  
  // least-squares estimation
  MarquardtFitter fit = MarquardtFitter(apMatrix);
  fit.setData(fullwave);

#if DATA_ANALYTICS >= 2
  std::unordered_set<std::size_t> capturedIndices;
#endif

  // =========================================================================
  // === THIS ENTIRE LOOP REPLACES THE ORIGINAL LOOP IN THE FUNCTION        ===
  // =========================================================================
  for (int i = 0; i < numFullwaveBins; ++i) {
    
    // 1. Detect a potential raw peak at index 'i'
    double const eps = scanner->getReceivedEnergyMin(pulse.getDeviceIndex());
    if (fullwave[i] < eps || !detectPeak(i, win_size, fullwave, eps)) {
      continue; // This is not a valid local maximum in the window, so skip.
    }

    // 2. We have found a valid raw peak at index 'i'. Now, proceed to process it.
    double refined_peak_bin = (double)i; // Initialize with the raw peak index as a fallback.
    double echo_width = 0.0;
    bool fit_was_successful = false;

    // Use the existing flag to control whether to perform the advanced fitting.
    if (scanner->isCalcEchowidth()) {
        try {
            // Set the initial parameters for the fit using the raw peak info.
            fit.setParameters(vector<double>{0, fullwave[i], (double)i, 1.0});
            fit.fitData();

            // Extract the refined parameters from the successful fit.
            std::vector<double> params = fit.getParameters();
            double fitted_mean = params[2];
            double fitted_width = std::abs(params[3]);

            // Add a sanity check: if the fit is nonsensical, reject it.
            if (fitted_width * nsPerBin >= 0.1 && fitted_mean >= 0 && fitted_mean < numFullwaveBins) {
                 refined_peak_bin = fitted_mean; // THIS IS THE KEY: Use the fitted mean for the peak position.
                 echo_width = fitted_width * nsPerBin; // Use the fitted standard deviation for the width.
                 fit_was_successful = true;
            } else {
                 // The fit produced an invalid result (e.g., negative width, out of bounds mean).
                 // We will fall back to using the raw peak data below.
            }

        } catch (std::exception& e) {
            // If the fitter fails, log a warning and fall back to using the raw peak.
            std::stringstream ss;
            ss << "MarquardtFitter failed for pulse " << pulse.getPulseNumber() 
               << ". Falling back to raw peak detection. Details: " << e.what();
            logging::WARN(ss.str());
            // Fallback is already handled as refined_peak_bin is initialized to 'i'.
        }
    }
    
    // 3. Compute distance using the REFINED_PEAK_BIN (or the raw one if fitting was off/failed).
    double distance = SPEEDofLIGHT_mPerNanosec * (refined_peak_bin * nsPerBin + minHitTime_ns);

    // 4. The rest of the original function logic follows from here...
    
    // Build list of objects that produced this return
    double minDifference = numeric_limits<double>::max();
    shared_ptr<RaySceneIntersection> closestIntersection = nullptr;

#ifdef DATA_ANALYTICS
    size_t intersectionIdx = 0;
    size_t closestIntersectionIdx = 0;
#endif
    for (RaySceneIntersection intersect : intersects) {
      double const intersectDist =
        glm::distance(intersect.point, pulse.getOriginRef());
      double const absDistDiff = std::fabs(intersectDist - distance);

      if (absDistDiff < minDifference) {
        minDifference = absDistDiff;
        closestIntersection = make_shared<RaySceneIntersection>(intersect);
#ifdef DATA_ANALYTICS
        closestIntersectionIdx = intersectionIdx;
#endif
      }
#ifdef DATA_ANALYTICS
      ++intersectionIdx;
#endif
    }

    string hitObject;
    int classification = 0;
    if (closestIntersection != nullptr) {
      hitObject = closestIntersection->prim->part->mId;
      if (closestIntersection->prim->part->getType() ==
          ScenePart::DYN_MOVING_OBJECT) {
        hitObject = hitObject.substr(5);
      }
      classification = closestIntersection->prim->material->classification;
    }
    
    // Add distance error (mechanical range error)
    if (pulse.hasMechanicalError()) {
      distance += pulse.getMechanicalRangeError();
    }
    
    // Build measurement
    Measurement tmp;
    tmp.devIdx = devIdx;
    tmp.devId = scanner->getDeviceId(devIdx);
    tmp.beamOrigin = pulse.getOrigin();
    tmp.beamDirection = beamDir;
    tmp.distance = distance; 
    tmp.echo_width = echo_width; 
    tmp.intensity = fullwave.at(i); 
    tmp.fullwaveIndex = pulse.getPulseNumber();
    tmp.hitObjectId = hitObject;
    tmp.returnNumber = numReturns + 1;
    tmp.classification = classification;
    tmp.gpsTime = pulse.getTime();
    tmp.acrossTrackAngle_rad = pulse.getAcrossTrackAngle();
    tmp.downTrackAngle_rad = pulse.getDownTrackAngle();

    pointsMeasurement.push_back(tmp);
    ++numReturns;

#if DATA_ANALYTICS >= 2
    capturedIndices.insert(closestIntersectionIdx);
#endif

    // Check if maximum number of returns per pulse has been reached
    if (!scanner->checkMaxNOR(numReturns, devIdx))
      break;
    
    // To prevent re-detecting the same broadened peak, we can skip forward in the loop.
    if (fit_was_successful) {
      // If we used the advanced fitting, we can safely jump ahead.
      i += win_size; 
    }
    // If we did not use fitting, the loop's natural i++ is the correct behavior
    // to find the next distinct peak.
  }

#if DATA_ANALYTICS >= 2
  // ... (original code for recording non-captured points) ...
#endif
}