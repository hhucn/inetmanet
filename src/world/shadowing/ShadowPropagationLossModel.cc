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

#include <ShadowPropagationLossModel.h>
#include <math.h>

Register_Class(ShadowPropagationLossModel);

/**
 * Intialize (called in the initialization phase of the radioModule).
 * Gets sensitivity which is configured for the radio and
 * gets a reference to the unique mapManager in the simulation (which is used of all nodes).
 */
void ShadowPropagationLossModel::initializeFrom(cModule *radioModule) {
    EV << "ShadowPropagationLossModel initialize" << endl;
    sensitivityDbm = radioModule->par("sensitivity").doubleValue();
    mm = dynamic_cast<MapManager *>(simulation.getModuleByPath("mapManager"));
}

/**
 * Calculate the signal loss between the sender and the receiver.
 * \returns rxPowerDbm
 */
double ShadowPropagationLossModel::calculateReceivedPower(double pSend, double carrierFrequency, const Coord& senderPos, const Coord& receiverPos) {

    EV << "ShadowPropagationLossModel calculateReceivedPower: senderPos=" << senderPos << " , receiverPos=" << receiverPos << endl;

    // Indicator if sender and receiver are out of range due to too high distance
    bool outofrange = false;
    // Indicator if signal is shadowed completely (in the case when buildings are between sender and receiver)
    bool shadowed = false;

    // Precalculate waveLength, which is used several times in the following
    const double speedOfLight = 300000000.0;
    double waveLength = speedOfLight / carrierFrequency;

    // dmax = lambda/4PI * sqrt(Pt/Pr) with Pr = sensitivity
    // Find the max radius for building "examinations" and get buildings in this area
    double rMax = waveLength / (4 * PI) * sqrt(dBm2mW(pSend - sensitivityDbm));
    EV << "ShadowPropagationLossModel calculateReceivedPower: pSend=" << pSend << " , rMax=" << rMax << endl;

    // Check íf sender and receiver are in range at all (to skip possible building-calculations)
    double srDistance = senderPos.distance(receiverPos);
    if (rMax < srDistance) {
        EV << "ShadowPropagationLossModel calculateReceivedPower: srDistance=" << srDistance << " (out of range)" << endl;
        outofrange = true;
    }
    else {
        // Do shadowing-checks, first get all buildings between area of sender and receiver
        std::vector<int> buildings = mm->getBuildingsInArea(senderPos, receiverPos);
        EV << "ShadowPropagationLossModel calculateReceivedPower: buildingsInArea=" << buildings.size() << endl;

        if (buildings.empty()==false) {
            std::vector<int>::iterator itBuildings;
            for (itBuildings = buildings.begin(); itBuildings < buildings.end(); itBuildings++)
            {
                // Twofold check (due to performance issues):
                // 1) Fast check if area (abstracted area of a building) is between
                if (true==mm->isAreaBetween(senderPos.x,senderPos.y,receiverPos.x,receiverPos.y,*itBuildings)){
                    EV << "ShadowPropagationLossModel calculateReceivedPower: areaBetweenOfBuilding=" << *itBuildings << endl;

                    // 2) More detailed and slightly slower check if the actual building outline is between
                    if (true==mm->isBuildingBetween(senderPos.x,senderPos.y,receiverPos.x,receiverPos.y,*itBuildings)){
                        EV << "ShadowPropagationLossModel calculateReceivedPower: buildingBetweenOfBuilding=" << *itBuildings << endl;

                        // In this simple model, buildings shadow the transmission signal completely,
                        // so that it will not reach the receiver at all
                        shadowed=true;
                    }
                }
            }
        }
    }

    // Calculate received power
    double prec;
    if (outofrange || shadowed) {
        // In the case of too high distance or shadowing, the signal is below sensitivity
        prec = dBm2mW(sensitivityDbm - 1);
        EV << "ShadowPropagationLossModel calculateReceivedPower: prec=" << prec << "mW, " << mW2dBm(prec) << "dBm (below sensitivity)" << endl;
    }
    else {
        // Otherwise (no building in between), we assume the Freespace Pathloss according to Friis' equation
        prec = pSend * waveLength * waveLength / (16 * M_PI * M_PI * pow(srDistance, 2));
        if (prec > pSend) prec = pSend;
        EV << "ShadowPropagationLossModel calculateReceivedPower: prec=" << prec << "mW, " << mW2dBm(prec) << "dBm (Freespace @srDistance=" << srDistance << ")" << endl;
    }
    return prec;
}
