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

#ifndef TWORAYINTERFERENCEMODEL_H_
#define TWORAYINTERFERENCEMODEL_H_

#include "FreeSpaceModel.h"

class INET_API TwoRayInterferenceModel : public FreeSpaceModel {

	public:
	~TwoRayInterferenceModel();
	 virtual void initializeFrom(cModule *radioModule);
	/**
	 * To be redefined to calculate the received power of a transmission.
	 */
	virtual double calculateReceivedPower(double pSend, double carrierFrequency, double distance);

	private:
	double ht, hr;
	double epsilon_r;
};

#endif /* TWORAYINTERFERENCEMODEL_H_ */
