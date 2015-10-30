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

#ifndef DIFFRACTIONPROPAGATIONLOSSMODEL_H_
#define DIFFRACTIONPROPAGATIONLOSSMODEL_H_

#include "IShadowingModel.h"
#include "MapManager.h"

/**
 * This DiffractionPropagationLossModel was developed in the context of Michael Oppermanns master thesis.
 * It uses the mm (MapManager) to get information about the building between sender and receiver.
 * The ShadowPropagationLossModel shadows the signal completely if a building is between the sender and the receiver.
 * The PenetrationPropagationLossModel measures the thickness of a building (meter between two walls) and count the number of
 * penetrated walls. The model regards irregular geometries and backyards.
 * The DiffractionPropagationLossModel is based on the knife-edge diffraction formula and calculates the signal loss on a corner of a building.
 * The master thesis give more detailed information about these signal loss models.
 *
 * @author rpr
 */
class INET_API DiffractionPropagationLossModel : public IShadowingModel {

public:
    ~DiffractionPropagationLossModel() {};
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

    /**
     * Determine the diffraction loss according to the v-value.
     */
    virtual double diffract(double v);

    /**
     * Receiver sensitivity, this means the signal power when the receiver is still able to decode the frame.
     */
    double sensitivityDbm;

    /**
     * Reference to the MapManager to administrate the buildings.
     */
    MapManager* mm;
};

#endif /* DIFFRACTIONPROPAGATIONLOSSMODEL_H_ */
