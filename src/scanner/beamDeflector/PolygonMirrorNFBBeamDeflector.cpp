#include "PolygonMirrorNFBBeamDeflector.h"

#include <iostream>
#include <sstream>
#include <logging.hpp>
#include "maths/Directions.h"
#include "RotationOrder.h"
#include <cmath> // For std::fabs, std::asin, std::isnan, std::pow
#include <glm/glm.hpp> // For dvec3 and clamp
#define _USE_MATH_DEFINES
#include <math.h>
#include "MathConverter.h"
#include <limits> // For std::numeric_limits

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
std::shared_ptr<AbstractBeamDeflector> PolygonMirrorNFBBeamDeflector::clone(){
    std::shared_ptr<AbstractBeamDeflector> pmnfbbd =
        std::make_shared<PolygonMirrorNFBBeamDeflector>(
            cfg_device_scanFreqMax_Hz,
            cfg_device_scanFreqMin_Hz,
            cfg_device_scanAngleMax_rad,
            cfg_device_scanAngleEffectiveMax_rad
        );
    _clone(pmnfbbd);
    return pmnfbbd;
};

void PolygonMirrorNFBBeamDeflector::_clone(
    std::shared_ptr<AbstractBeamDeflector> abd
){
    AbstractBeamDeflector::_clone(abd); // Calls base class clone first

    PolygonMirrorNFBBeamDeflector *pmnfbbd = dynamic_cast<PolygonMirrorNFBBeamDeflector *>(abd.get());
    if(pmnfbbd){ // Check if cast was successful
        pmnfbbd->cfg_device_scanAngleEffective_rad = this->cfg_device_scanAngleEffective_rad;
        pmnfbbd->cfg_device_scanAngleEffectiveMax_rad = this->cfg_device_scanAngleEffectiveMax_rad;
        //!!pmnfbbd->state_currentRollAngle_rad = this->state_currentRollAngle_rad;
        pmnfbbd->state_currentLineCounter = this->state_currentLineCounter;
        pmnfbbd->cached_maxSetForwardAngle_rad = this->cached_maxSetForwardAngle_rad;
        pmnfbbd->cached_maxSetRollAngle_rad = this->cached_maxSetRollAngle_rad;
        pmnfbbd->cached_emitterSetRelativeAttitude = this->cached_emitterSetRelativeAttitude;
        pmnfbbd->parCoefficient = this->parCoefficient;
        pmnfbbd->minTiltAngle_deg = this->minTiltAngle_deg;
    } else {
        // Handle error: cast failed - should not happen if clone() is correct
        logging::ERR("PolygonMirrorNFBBeamDeflector::_clone dynamic_cast failed!");
    }
};

// ***  M E T H O D S  *** //
// *********************** //

void PolygonMirrorNFBBeamDeflector::applySettings(std::shared_ptr<ScannerSettings> settings) {
    setScanAngle_rad(settings->scanAngle_rad);
    setScanFreq_Hz(settings->scanFreq_Hz);

    std::stringstream ss;

    // get the verticalAngle settings (suggested for TLS)
    cfg_setting_verticalAngleMin_rad = settings->verticalAngleMin_rad;
    cfg_setting_verticalAngleMax_rad = settings->verticalAngleMax_rad;
    
    ss  << "Applying settings for PolygonMirrorNFBBeamDeflector...\n";
    ss  << "Vertical angle min/max " << cfg_setting_verticalAngleMin_rad << "/"
        << cfg_setting_verticalAngleMax_rad;

    // if not set, use the ones from the scanAngleEffectiveMax or scanAngle (whichever is lower)
    if(std::isnan(cfg_setting_verticalAngleMin_rad)){
        cfg_setting_verticalAngleMin_rad = -1. * std::min(
            cfg_device_scanAngleEffectiveMax_rad,
            cfg_setting_scanAngle_rad
        );
        ss  << "\n -- verticalAngleMin not set, using the value of "
            << MathConverter::radiansToDegrees(
                cfg_setting_verticalAngleMin_rad
            ) << " degrees";
    }
    if(std::isnan(cfg_setting_verticalAngleMax_rad)){
        cfg_setting_verticalAngleMax_rad = std::min(
            cfg_device_scanAngleEffectiveMax_rad,
            cfg_setting_scanAngle_rad
        );
        ss  << "\n -- verticalAngleMax not set, using the value of "
            << MathConverter::radiansToDegrees(
                cfg_setting_verticalAngleMax_rad
            ) << " degrees";
    }
    
    // Initialize state variables
    state_currentBeamAngle_rad = 0.0; // Reset scan angle
    this->state_currentLineCounter = 0;
    //state_currentForwardAngle_rad = 0.0; // Reset forward angle state

    // For calculating the spacing between subsequent shots:
    double const angleMax = cfg_device_scanAngleMax_rad;
    double const angleMin = -cfg_device_scanAngleMax_rad;
    state_angleDiff_rad = angleMax-angleMin;
    if(settings->pulseFreq_Hz > 0){ // Avoid division by zero
        cached_angleBetweenPulses_rad = (double)(
            this->cfg_setting_scanFreq_Hz * state_angleDiff_rad
        ) / settings->pulseFreq_Hz;
    } else {
         cached_angleBetweenPulses_rad = 0;
         logging::WARN("Pulse frequency is zero, angle between pulses set to zero.");
    }

    // Calculate and cache the maximum effective forward angle limit
    // This calculates the parabolic tilt value at the edge defined by verticalAngleMax
        // --- Use Quadratic Function for 15->10->15 Range ---
        // baseTilt = A*(phase - 0.5)^2 + min_tilt
        // We want min_tilt = 10.
        // At phase=0 or phase=1, we want tilt=15.
        // 15 = A*(0 - 0.5)^2 + 10  => 15 = A*0.25 + 10 => 5 = A*0.25 => A = 20
    parCoefficient = 20.0; // parameter coefficient for the parabolic curve
    minTiltAngle_deg = 10.0; // minimum tilt in degrees
    
    // Calculate the phase corresponding to the vertical angle max limit
    double phase_at_limit = (state_angleDiff_rad > 1e-9) ?
                            (cfg_setting_verticalAngleMax_rad - (-cfg_device_scanAngleMax_rad)) / state_angleDiff_rad
                            : 0.5; // Use center phase if range is zero
    // Ensure phase is within [0, 1] in case verticalAngleMax is outside scanAngleMax range
    phase_at_limit = glm::clamp(phase_at_limit, 0.0, 1.0);

    // Calculate the tilt in degrees at this phase limit
    double SetTilt_deg = parCoefficient * pow(phase_at_limit - 0.5, 2) + minTiltAngle_deg;
    this->cached_maxSetForwardAngle_rad = MathConverter::degreesToRadians(SetTilt_deg);
    
    Rotation set_across_track = Rotation(Directions::right, cfg_setting_verticalAngleMax_rad);
    Rotation set_along_track = Rotation(Directions::up, cached_maxSetForwardAngle_rad); // Use stored value

    // Compose: Rz * Rx
    this->cached_emitterSetRelativeAttitude = set_along_track.applyTo(set_across_track);
        
    // Calculate the roll in radians at the scan Angle Limit
    //!!!const glm::dvec3 emitter_nominal_forward = Directions::forward; // (0, 1, 0)

    // // Calculate the final direction of the beam RELATIVE TO THE EMITTER MOUNT
    // // using the rotation calculated in this deflector's doSimStep()
    //!!!glm::dvec3 final_SetBeamVector = this->cached_emitterSetRelativeAttitude.applyTo(emitter_nominal_forward);
        
    double maxSetRollAngle_rad, maxSetPitchAngle_rad, maxSetYawAngle_rad;
    // Use RotationOrder::XYZ for standard Roll (X), Pitch (Y), Yaw (Z) extrinsic interpretation
    // Note: The results depend heavily on the chosen order!
    this->cached_emitterSetRelativeAttitude.getAngles(
        &RotationOrder::XYZ, // Pass address of the static const object
        maxSetRollAngle_rad,
        maxSetPitchAngle_rad,
        maxSetYawAngle_rad);
    
    // Store the calculated roll angle into the member variable for later access
    this->cached_maxSetRollAngle_rad = maxSetRollAngle_rad;
    //!!!!this->state_currentRollAngle_rad = roll_rad; // <-- STORE HERE

    //ss << "\n -- Max Effective Forward Angle Limit: " << maxTilt_deg << " degrees (calculated at phase " << phase_at_limit << ")";
    logging::INFO(ss.str());
}

void PolygonMirrorNFBBeamDeflector::doSimStep() {
    
    // Update beam angle in across-track scan
    state_currentBeamAngle_rad += cached_angleBetweenPulses_rad;

    // Reset angle and advance line counter when scan completes
    if (state_currentBeamAngle_rad > cfg_device_scanAngleMax_rad) {
        state_currentBeamAngle_rad = -cfg_device_scanAngleMax_rad;
        this->state_currentLineCounter++;
        if (this->state_currentLineCounter >= 3) { // Use >= 3 for safety, reset to 0
            this->state_currentLineCounter = 0;
        }
    }

    // Calculate the phase for the quadratic parabolic tilt based on the current across-track angle
    // Phase = 0 at -max angle, 0.5 at center (0 angle), 1.0 at +max angle
    // Avoid division by zero if scanRange is somehow zero
    double phase = (state_angleDiff_rad > 1e-9) ? (state_currentBeamAngle_rad - (-cfg_device_scanAngleMax_rad)) / state_angleDiff_rad : 0.5;
    // Clamp phase to the valid [0, 1] range for robustness
    phase = glm::clamp(phase, 0.0, 1.0);
    // Calculate the forward/backward tilt angle (local variable for calculation)
    //double state_currentForwardAngle_rad = 0.0; // Use local variable for calculation clarity
    
    if (this->state_currentLineCounter == 0) { // Nadir face
        state_currentForwardAngle_rad = MathConverter::degreesToRadians(0.0);
    }
    else {
        // Use Quadratic Function for 15->10->15 Range
        double currentTilt_deg = parCoefficient * pow(phase - 0.5, 2) + minTiltAngle_deg;

        if (this->state_currentLineCounter == 1) { // Forward face
            state_currentForwardAngle_rad = MathConverter::degreesToRadians(currentTilt_deg);
        }
        else { // Backward face (lineCounter == 2)
            state_currentForwardAngle_rad = MathConverter::degreesToRadians(-currentTilt_deg);
        }
    }

    // --- Create and Compose Independent Rotations ---
    // Using Directions::right (X) and Directions::up (Z) based on Directions.cpp
    Rotation r_across_track = Rotation(Directions::right, state_currentBeamAngle_rad);
    Rotation r_along_track = Rotation(Directions::up, state_currentForwardAngle_rad); // Use stored value

    // Compose: Rz * Rx
    this->cached_emitterRelativeAttitude = r_along_track.applyTo(r_across_track);
    
}

bool PolygonMirrorNFBBeamDeflector::lastPulseLeftDevice() {
    // four conditions for the beam to return an echo:
    // 1) abs(currentAngle) <= scanAngleEffectiveMax
    // 2) abs(currentAngle) <= scanAngle (if it is set to something smaller)
    // 3) currentAngle > verticalAngleMin
    // 4) currentAngle <= verticalAngleMax

    // Issue 1: lineCounter is not accessible here. It's static local to doSimStep.
    // You need to make lineCounter a member variable if you want to use it here.
    // Let's assume for now lineCounter IS a member: this->lineCounter

    bool condition1_effective_max = std::fabs(this->state_currentBeamAngle_rad) <= this->cfg_device_scanAngleEffectiveMax_rad;
    bool condition3_vertical_min = this->state_currentBeamAngle_rad >= this->cfg_setting_verticalAngleMin_rad; // Note: >= not >
    bool condition4_vertical_max = this->state_currentBeamAngle_rad <= this->cfg_setting_verticalAngleMax_rad;

    if (this->state_currentLineCounter == 0) { // Assuming lineCounter is a member variable
        // For Nadir, compare input across-track angle to user setting scan angle
        bool condition2_user_setting = std::fabs(this->state_currentBeamAngle_rad) <= this->cfg_setting_scanAngle_rad;
        return condition1_effective_max &&
               condition2_user_setting &&
               condition3_vertical_min &&
               condition4_vertical_max;
    }
    else {
        // For Forward/Backward, compare current across-track input to the calculated cached_maxSetRollAngle_rad
        // This cached_maxSetRollAngle_rad is the X-rotation component (roll) of the *limiting orientation*.
        // It's not directly an across-track scan angle limit.
        // If you want to limit the *input* state_currentBeamAngle_rad based on the derived roll,
        // it means you're trying to stop the input sweep earlier.

        // Let's assume the intent is: if the *input* across-track angle exceeds
        // the roll component derived from the limiting condition, then stop.
        bool condition2_roll_limit;
        if (std::isnan(this->cached_maxSetRollAngle_rad)) {
            logging::WARN("cached_maxSetRollAngle_rad is NaN, defaulting to true for this part of check");
            condition2_roll_limit = true; // Or false, depending on desired safety
        } else {
            condition2_roll_limit = std::fabs(this->state_currentBeamAngle_rad) <= std::fabs(this->cached_maxSetRollAngle_rad);
        }
        
        return condition1_effective_max &&
               condition2_roll_limit &&    // Using input angle vs. derived roll limit
               condition3_vertical_min &&
               condition4_vertical_max;
    }
}