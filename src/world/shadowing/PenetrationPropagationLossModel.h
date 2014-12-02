//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef PENETRATIONPROPAGATIONLOSSMODEL_H_
#define PENETRATIONPROPAGATIONLOSSMODEL_H_

#include "IShadowingModel.h"
#include "MapManager.h"

/**
 * This PenetrationPropagationLossModel was developed in the context of Michael Oppermanns master thesis.
 * It uses the mm (MapManager) to get information about the building between sender and receiver.
 * The ShadowPropagationLossModel shadows the signal completely if a building is between the sender and the receiver.
 * The PenetrationPropagationLossModel measures the thickness of a building (meter between two walls) and count the number of
 * penetrated walls. The model regards irregular geometries and backyards.
 * The DiffractionPropagationLossModel is based on the knife-edge diffraction formula and calculates the signal loss on a corner of a building.
 * The master thesis give more detailed information about these signal loss models.
 *
 * @author rpr
 */
class INET_API PenetrationPropagationLossModel : public IShadowingModel {

public:
    ~PenetrationPropagationLossModel() {};
    virtual void initializeFrom(cModule *radioModule);

    /**
     * Redefined to calculate the received power of a transmission with complete shadowing of the signal at buildings.
     */
    virtual double calculateReceivedPower(double pSend, double carrierFrequency, const Coord& senderPos, const Coord& receiverPos);

protected:
    /**
     * Simple utils to convert decibels and milliwatts.
     */
    virtual double dBm2mW(double valueDbm) { return pow(10.0, valueDbm / 10.0); }
    virtual double mW2dBm(double valueMw) { return 10.0 * log10(valueMw); }

    /** Receiver sensitivity, this means the signal power when the receiver is still able to decode the frame. */
    double sensitivityDbm;

    /**
     *
     */
    double signalLossFactorWall;
    double signalLossFactorBuilding;

    /**
     * Reference to the MapManager to administrate the buildings.
     */
    MapManager* mm;
};

#endif /* PENETRATIONPROPAGATIONLOSSMODEL_H_ */
