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

#include <DiffractionPropagationLossModel.h>
#include <math.h>

Register_Class(DiffractionPropagationLossModel);

/**
 * Intialize (called in the initialization phase of the radioModule).
 * Gets sensitivity which is configured for the radio and
 * gets a reference to the unique mapManager in the simulation (which is used of all nodes).
 */
void DiffractionPropagationLossModel::initializeFrom(cModule *radioModule) {
    EV << "DiffractionPropagationLossModel initialize" << endl;
    sensitivityDbm = radioModule->par("sensitivity").doubleValue();
    mm = dynamic_cast<MapManager *>(simulation.getModuleByPath("mapManager"));
}

/**
 * Calculate the signal loss between the sender and the receiver.
 */
double DiffractionPropagationLossModel::calculateReceivedPower(double pSend, double carrierFrequency, const Coord& senderPos, const Coord& receiverPos) {

    EV << "DiffractionPropagationLossModel calculateReceivedPower: senderPos=" << senderPos << " , receiverPos=" << receiverPos << endl;

    // Precalculate waveLength, which is used several times in the following
    const double speedOfLight = 300000000.0;
    double waveLength = speedOfLight / carrierFrequency;

    // dmax = lambda/4PI * sqrt(Pt/Pr) with Pr = sensitivity
    // Find the max radius for building "examinations" and get buildings in this area
    double rMax = waveLength / (4 * PI) * sqrt(dBm2mW(pSend - sensitivityDbm));
    EV << "DiffractionPropagationLossModel calculateReceivedPower: pSend=" << pSend << " , rMax=" << rMax << endl;

    // Check íf sender and receiver are in range at all (to skip possible building-calculations)
    double srDistance = senderPos.distance(receiverPos);
    if (rMax < srDistance) {
       double pr = dBm2mW(sensitivityDbm - 1);
       EV << "ShadowPropagationLossModel calculateReceivedPower: srDistance=" << srDistance <<
               ", prec=" << pr << "mW, " << mW2dBm(pr) << "dBm (out of range)" << endl;
       return pr;
    }

    // Start exactly like the ShadowingPropagationLossModel and
    std::vector<int> buildings = mm->getBuildingsInArea(senderPos, receiverPos);
    EV << "DiffractionPropagationLossModel calculateReceivedPower: buildingsInArea=" << buildings.size() << endl;
    if (!buildings.empty()) {
        //  1) Fill list with buildings that are between
        std::vector<int> betweenBuildings;
        std::vector<int>::iterator itBuildings;
        EV << "DiffractionPropagationLossModel calculateReceivedPower: buildingBetween=" << endl;
        for (itBuildings = buildings.begin(); itBuildings < buildings.end(); itBuildings++) {
            if (mm->isAreaBetween(senderPos.x, senderPos.y, receiverPos.x, receiverPos.y, *itBuildings)) {
                if (mm->isBuildingBetween(senderPos.x, senderPos.y, receiverPos.x, receiverPos.y, *itBuildings)) {
                    betweenBuildings.push_back(*itBuildings);
                    EV << *itBuildings << endl;
                }
            }
        }

        // 2) Treat these buildings between sender and receiver for the diffraction calculation and
        if (!betweenBuildings.empty()) {
            // collect the final diff-corners which stand all line of sight checks
            std::vector<Coord> finalDiffCorners;
            for (itBuildings = betweenBuildings.begin(); itBuildings < betweenBuildings.end(); itBuildings++) {
                // Get all initial diffraction corners which have a clear line of sight between sender and receiver
                EV << "DiffractionPropagationLossModel calculateReceivedPower: building=" << *itBuildings << endl;
                std::vector<Coord> initialDiffCorners = mm->getDiffractionCorners(senderPos.x, senderPos.y, receiverPos.x, receiverPos.y, *itBuildings);
                EV << "DiffractionPropagationLossModel calculateReceivedPower: building=" << *itBuildings << " initialDiffCorners=";

                // Double-check each found diffraction-corner for
                std::vector<Coord>::iterator itidc;
                for (itidc = initialDiffCorners.begin(); itidc < initialDiffCorners.end(); itidc++) {
                    EV << "(" << itidc->x << "," << itidc->y << ")";
                    // The clear line of sight against the other buildings between sender and receiver
                    bool diffClear = true;
                    std::vector<int>::iterator itBb;
                    for (itBb = betweenBuildings.begin(); itBb < betweenBuildings.end(); itBb++) {
                        if (*itBb != *itBuildings) {
                            EV << " (check bBetween=" << *itBb << ") ";
                            if (!mm->isBuildingBetween(senderPos.x, senderPos.y, itidc->x, itidc->y, *itBb)) {
                                if (!mm->isBuildingBetween(itidc->x, itidc->y, receiverPos.x, receiverPos.y, *itBb)) {
                                    // Only when no further building is between sender->diffcorner and diffcorner->receiver,
                                    // diffraction effectively happens (however, go on checking other buildings)
                                    EV << " clear";
                                    diffClear = true;
                                }
                                else {
                                    // When merely one building is in view, diffraction cannot happen at that corner (break)
                                    EV << " not clear";
                                    diffClear = false;
                                    break;
                                }
                            }
                            else {
                                // When merely one building is in view, diffraction cannot happen at that corner (break)
                                EV << " not clear";
                                diffClear = false;
                                break;
                            }
                        }
                    }
                    // Save the few corners left (probably no corner stands all checks)
                    if (diffClear) {
                        EV << " - push final";
                        finalDiffCorners.push_back(*itidc);
                    }
                }
                EV << endl;
            }

            // 3) Do the final round of checks with the diff-corners to find the one with the smallest diffraction loss
            if (!finalDiffCorners.empty()) {
                EV << "DiffractionPropagationLossModel calculateReceivedPower: finalDiffCorners=" << endl;

                double dsr = senderPos.distance(receiverPos);
                double Ld = sensitivityDbm;

                // Calculate the loss according to the knife-edge diffraction procedure
                std::vector<Coord>::iterator itfdc;
                for (itfdc = finalDiffCorners.begin(); itfdc < finalDiffCorners.end(); itfdc++) {
                    EV << "(" << itfdc->x << "," << itfdc->y << ") ";

                    // Calculate the height of the diff-corner relative to the line sender->receiver with trigonometry
                    // (length of the edge sender-corner)
                    double dsc = senderPos.distance(*itfdc);
                    // (angle between vectors sender->corner, sender->receiver)
                    double alpha = acos(((itfdc->x - senderPos.x) * (receiverPos.x - senderPos.x) +
                            (itfdc->y - senderPos.y) * (receiverPos.y - senderPos.y)) / (dsc * dsr));
                    // (height itself)
                    double h = dsc * sin(alpha);
                    // (length of the edge sender-intersection/height)
                    double ds = sqrt((dsc * dsc) - (h * h));
                    // (length of the edge intersection/height-receiver)
                    double dr = dsr - ds;
                    // V-value according to knife-edge equation
                    double v = h * sqrt((2 * (dr + ds)) / (waveLength * dr * ds));
                    // Diffraction loss
                    double LdV = diffract(v);
                    if (LdV > Ld) Ld = LdV;

                    EV << "dsc=" << dsc << ", alpha=" << alpha*180/PI << ", h=" << h << ", lambda=" << waveLength << ", ds=" << ds << ", dr=" << dr << ", v=" << v << ", LdV=" << LdV << ", Ld=" << Ld << endl;
                }
                // Calculate received power and include additional diffraction loss
                double Lfs = pSend * waveLength * waveLength / (16 * M_PI * M_PI * pow(dsr, 2));
                double prec = Lfs * dBm2mW(Ld);
                if (prec > pSend) prec = pSend;
                EV << "DiffractionPropagationLossModel calculateReceivedPower: prec=" << prec << "mW, " << mW2dBm(prec) <<
                        "dBm (diffracted Lfs= " << mW2dBm(Lfs) << ", Ld=" << Ld << ")" << endl;
                return prec;
            }
            else {
                // When no unobstructed diff-corner exists (but buildings are between), return a signal below sensitivity
                double prec = dBm2mW(sensitivityDbm - 1);
                EV << "DiffractionPropagationLossModel calculateReceivedPower: prec=" << prec << "mW, " << mW2dBm(prec) << "dBm (shadowed)" << endl;
                return prec;
            }
        } // When buildings are around in the area, but no one is between sender and receiver ...
    } // ... OR no buildings at all are around the sender,

    // Apply the Freespace propagation
    double distance = senderPos.distance(receiverPos);
    double prec = pSend * waveLength * waveLength / (16 * M_PI * M_PI * pow(distance, 2));
    if (prec > pSend) prec = pSend;
    EV << "DiffractionPropagationLossModel calculateReceivedPower: prec=" << prec << "mW, " << mW2dBm(prec) << "dBm (Freespace @distance=" << distance << ")" << endl;
    return prec;
}

/**
 * Calculate the diffraction loss in dependency of v according to
 * the Knife-Edge-Diffraction Model introduced in
 * W.C.Y. Lee "Mobile Communications Engineering: Theory and Applications" - 1985.
 */
double DiffractionPropagationLossModel::diffract(double v) {
    double LdV =0;
    if (v <= -1) {
        LdV = 0;
    }
    else if ((v > -1) && (v <= 0)) {
        LdV = 20 * log(0.5 - 0.62 * v);
    }
    else if ((v > 0) && (v <= 1)) {
        LdV = 20 * log(0.5 * exp(-0.95 * v));
    }
    else if ((v > 1) && (v <= 2.4)) {
        double v_term = (0.38 - 0.1 * v);
        LdV = 20 * log(0.4 - sqrt(0.1184 - (v_term * v_term)));
    }
    else {
        // (v > 2.4)
        LdV = 20 * log(0.225 / v);
    }
    return LdV;
}
