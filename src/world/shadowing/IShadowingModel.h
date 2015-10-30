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

#ifndef ISHADOWINGMODEL_H_
#define ISHADOWINGMODEL_H_

#include "INETDefs.h"
#include "Coord.h"

/**
 * Abstract class (interface) for the calculation of message reception (i.e. received power)
 * with the consideration of building geometries to shadow the propagation of the signal.
 * (Needs to be implemented by the actual models).
 *
 * @author rpr
 */
class INET_API IShadowingModel : public cObject {

public:
    /**
     * Virtual class destructor for base class.
     */
    virtual ~IShadowingModel() {};

    /**
     * Initialize method, typically called in the initialization phase of the
     * radio module (AbstractRadio or AbstractRadioExtended).
     */
    virtual void initializeFrom(cModule *radioModule) = 0;

    /**
     * Method to simulate the reception, typically called in handleLowerMsgStart at the arrival of an airframe in the
     * radio module (AbstractRadio or AbstractRadioExtended).
     */
    virtual double calculateReceivedPower(double pSend, double carrierFrequency, const Coord& senderPos, const Coord& receiverPos) = 0;
//    virtual double calculateReceivedPower(double pSend, std::string senderMod, std::string receiverMod);
};

#endif /* ISHADOWINGMODEL_H_ */
