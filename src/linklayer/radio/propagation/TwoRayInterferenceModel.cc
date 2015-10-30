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

#include "ChannelControl.h"
#include "ChannelAccess.h"
#include "TwoRayInterferenceModel.h"

#include <FWMath.h>

using namespace std;

Register_Class(TwoRayInterferenceModel);

TwoRayInterferenceModel::~TwoRayInterferenceModel() {
}

void TwoRayInterferenceModel::initializeFrom(cModule *radioModule) {
	initializeFreeSpace(radioModule);
	ht = radioModule->par("TransmiterAntennaHigh");
	hr = radioModule->par("ReceiverAntennaHigh");
	epsilon_r = radioModule->par("DielectricConstant");
}

double TwoRayInterferenceModel::calculateReceivedPower(double pSend, double carrierFrequency, double distance) {
	const double speedOfLight = 300000000.0;
    double waveLength = speedOfLight / carrierFrequency;

    if (distance == 0)
	return pSend;

    double d_dir = sqrt( pow (distance,2) + pow((ht - hr),2) ); // direct distance
	double d_ref = sqrt( pow (distance,2) + pow((ht + hr),2) ); // distance via ground reflection
	double sin_theta = (ht + hr)/d_ref;
	double cos_theta = distance/d_ref;

	double gamma = (sin_theta - sqrt(epsilon_r - pow(cos_theta,2)))/
		(sin_theta + sqrt(epsilon_r - pow(cos_theta,2)));

	double phi = (2*M_PI/waveLength * (d_dir - d_ref));
	double loss = pow(4 * M_PI * (distance/waveLength) *
				1/(sqrt((pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)))), 2);

    double pRecv = pSend/loss;

    EV << "d_dir=" << d_dir << ", d_ref=" << d_ref << ", phi=" << phi << endl;
    EV << "Precv=" << pRecv << " at d=" << distance << endl;

    return pRecv;
}
