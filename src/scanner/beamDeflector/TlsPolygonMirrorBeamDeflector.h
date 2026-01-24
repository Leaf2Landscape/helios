#pragma once

#include "scanner/beamDeflector/PolygonMirrorBeamDeflector.h"
#include "scanner/ScannerHead.h"
#include <logging.hpp>

/**
 * @brief A specialized polygon mirror deflector for TLS setups.
 * It combines the mirror's across-track scan with the scanner head's
 * rotation, reporting the head's angle as the down-track angle.
 */
class TlsPolygonMirrorBeamDeflector : public PolygonMirrorBeamDeflector
{
protected:
    // A reference to the scanner head to get its rotation angle
    std::shared_ptr<ScannerHead> scannerHead;

public:
    // *** CONSTRUCTION / DESTRUCTION *** //
    TlsPolygonMirrorBeamDeflector(
        double _scanFreqMax_Hz,
        double _scanFreqMin_Hz,
        double _scanAngleMax_rad,
        double _scanAngleEffectiveMax_rad,
        std::shared_ptr<ScannerHead> _scannerHead
    ) : PolygonMirrorBeamDeflector(
        _scanFreqMax_Hz,
        _scanFreqMin_Hz,
        _scanAngleMax_rad,
        _scanAngleEffectiveMax_rad
    ), scannerHead(_scannerHead)
    {
        if (!scannerHead) {
            logging::ERR("TlsPolygonMirrorBeamDeflector was created with a null scanner head!");
        }
    }

    // Clone method for creating copies
    std::shared_ptr<AbstractBeamDeflector> clone() override
    {
        std::shared_ptr<AbstractBeamDeflector> tlspmbd =
            std::make_shared<TlsPolygonMirrorBeamDeflector>(
                cfg_device_scanFreqMax_Hz,
                cfg_device_scanFreqMin_Hz,
                cfg_device_scanAngleMax_rad,
                cfg_device_scanAngleEffectiveMax_rad,
                scannerHead // The head reference is copied
            );
        _clone(tlspmbd);
        return tlspmbd;
    }

    /**
     * @see AbstractBeamDeflector::getOpticsType
     */
    std::string getOpticsType() const override { return "TLS_POLYGON_MIRROR"; }
    
    // *** OVERRIDE GETTERS TO PROVIDE TLS-SPECIFIC ANGLES *** //

    /**
     * @see AbstractBeamDeflector::getAcrossTrackAngle_rad
     * The across-track angle is still the mirror's scan angle.
     */
    double getAcrossTrackAngle_rad() const override
    {
        return this->state_currentBeamAngle_rad;
    }

    /**
     * @see AbstractBeamDeflector::getDownTrackAngle_rad
     * The down-track angle is the scanner head's current rotation angle.
     */
    double getDownTrackAngle_rad() const override
    {
        if (scannerHead) {
            return scannerHead->getRotateCurrent();
        }
        return 0.0;
    }
};