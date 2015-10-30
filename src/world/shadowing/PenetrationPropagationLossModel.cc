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

#include <PenetrationPropagationLossModel.h>
#include <math.h>

Register_Class(PenetrationPropagationLossModel);

/**
 * Intialize (called in the initialization phase of the radioModule).
 * Gets sensitivity which is configured for the radio and
 * gets a reference to the unique mapManager in the simulation (which is used of all nodes).
 */
void PenetrationPropagationLossModel::initializeFrom(cModule *radioModule) {
    EV << "PenetrationPropagationLossModel initialize" << endl;
    sensitivityDbm = radioModule->par("sensitivity").doubleValue();
    signalLossFactorWall = radioModule->par("signalLossFactorWall").doubleValue();
    signalLossFactorBuilding = radioModule->par("signalLossFactorBuilding").doubleValue();
    EV << "sensitivityDbm=" << sensitivityDbm <<
            ", signalLossFactorWall=" << signalLossFactorWall <<
            ", signalLossFactorBuilding=" << signalLossFactorBuilding << endl;
    mm = dynamic_cast<MapManager *>(simulation.getModuleByPath("mapManager"));
}

/**
 * Calculate the signal loss between the sender and the receiver.
 * \returns rxPowerDbm
 */
double PenetrationPropagationLossModel::calculateReceivedPower(double pSend, double carrierFrequency, const Coord& senderPos, const Coord& receiverPos) {

    EV << "PenetrationPropagationLossModel calculateReceivedPower: senderPos=" << senderPos << " , receiverPos=" << receiverPos << endl;

    // Number of walls and distance through penetrated buildings
    int nWalls = 0;
    double dBuilding = 0;

    // Precalculate waveLength, which is used several times in the following
    const double speedOfLight = 300000000.0;
    double waveLength = speedOfLight / carrierFrequency;

    // dmax = lambda/4PI * sqrt(Pt/Pr) with Pr = sensitivity
    // Find the max radius for building "examinations" and get buildings in this area
    double rMax = waveLength / (4 * PI) * sqrt(dBm2mW(pSend - sensitivityDbm));
    EV << "PenetrationPropagationLossModel calculateReceivedPower: pSend=" << pSend << ", rMax=" << rMax << endl;

    // Check íf sender and receiver are in range at all (to skip possible building-calculations)
    double srDistance = senderPos.distance(receiverPos);
    if (rMax < srDistance) {
       double pr = dBm2mW(sensitivityDbm - 1);
       EV << "ShadowPropagationLossModel calculateReceivedPower: srDistance=" << srDistance <<
               ", prec=" << pr << "mW, " << mW2dBm(pr) << "dBm (out of range)" << endl;
       return pr;
    }

    std::vector<int> buildings = mm->getBuildingsInArea(senderPos, receiverPos);
    EV << "PenetrationPropagationLossModel calculateReceivedPower: buildingsInArea=" << buildings.size() << endl;
    if (buildings.empty()==false) {
        std::vector<int>::iterator itBuildings;
        for (itBuildings = buildings.begin(); itBuildings < buildings.end(); itBuildings++)
        {
            // Check if area (abstracted area of a building) is between
            if (true==mm->isAreaBetween(senderPos.x,senderPos.y,receiverPos.x,receiverPos.y,*itBuildings)){
                EV << "PenetrationPropagationLossModel calculateReceivedPower: areBetweenOfBuilding=" << *itBuildings << endl;
                std::vector<Coord> intersections = mm->getIntersectionBetween(
                        senderPos.x,senderPos.y,receiverPos.x,receiverPos.y,*itBuildings);

                // Calculate number of walls and distance of penetrated buildings,
                // when they intersect the direct line between sender and receiver
                if (!intersections.empty()) {
                    EV << "PenetrationPropagationLossModel calculateReceivedPower: buildingBetweenOfBuilding=" << *itBuildings << endl;
                    std::vector<Coord>::iterator itwalls;
                    Coord first, second;
                    bool firstWall = true;
                    for (itwalls = intersections.begin(); itwalls < intersections.end(); itwalls++)
                    {
                        nWalls++;
                        if (firstWall==true){
                            first.x = itwalls->x;
                            first.y = itwalls->y;
                            firstWall=false;
                        }else{
                            second.x=itwalls->x;
                            second.y=itwalls->y;
                            dBuilding += first.distance(second);
                            firstWall=true;
                        }
                    }
                }
            }
        }
    }

    // Calculate actually received power
    // 1) Freespace distance
    double distance = senderPos.distance(receiverPos) - dBuilding;
    // 2) Losses at walls and per meter of buildings
    double signalLossWalls = nWalls * signalLossFactorWall;
    double signalLossBuilding = dBuilding * signalLossFactorBuilding;
    // 3a) Apply Friis' equation, 3b) Subtract additional losses (which means divide in mW)
    double prec = pSend * waveLength * waveLength / (16 * M_PI * M_PI * pow(distance, 2)) /
            dBm2mW(signalLossWalls + signalLossBuilding);
    if (prec > pSend) prec = pSend;
    EV << "PenetrationPropagationLossModel calculateReceivedPower: prec=" << prec << "mW, " << mW2dBm(prec) << "dBm" << endl;
    EV << "PenetrationPropagationLossModel calculateReceivedPower: distance=" << distance << ", nWalls=" << nWalls << ", dBuilding=" << dBuilding << endl;
    return prec;
}
