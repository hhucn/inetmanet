//
// Copyright (C) 2006 Andras Varga
// Copyright (C) 2011 Alfonso Ariza
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//


package inet.linklayer.ieee80211;

import inet.linklayer.ieee80211.mac.Ieee80211NewMac;
import inet.linklayer.ieee80211.mgmt.Ieee80211MgmtAdhoc;
import inet.linklayer.radio.Ieee80211NewRadio;

//
// This NIC implements an 802.11e(b,g) network interface card in ad-hoc mode.
//
module Ieee80211NewNicAdhoc
{
    parameters:
        @display("i=block/ifcard");
        string OpMode @enum ("b","g","a","p") = default("g"); 
        string PreambleMode @enum ("LONG","SHORT") = default("LONG"); // Wifi preamble mode Ieee 2007, 19.3.2
    gates:
        input uppergateIn; // to upper layers
        output uppergateOut; // from upper layers
        input radioIn @labels(AirFrame); // to receive AirFrames
    submodules:
        mgmt: Ieee80211MgmtAdhoc {
            parameters:
                @display("p=96,69;q=wlanDataQueue");
        }
        mac: Ieee80211NewMac {
            parameters:
                //opMode=OpMode;
                WifiPreambleMode = PreambleMode;
                queueModule = "mgmt";
                @display("p=96,155");
        }
        radio: Ieee80211NewRadio {
            parameters:
                //phyOpMode=OpMode;
                WifiPreambleMode = PreambleMode;
                
                @display("p=96,240");
        }
    connections:
        radioIn --> radio.radioIn;
        radio.uppergateIn <-- mac.lowergateOut;
        radio.uppergateOut --> mac.lowergateIn;

        mac.uppergateOut --> mgmt.macIn;
        mac.uppergateIn <-- mgmt.macOut;

        mgmt.uppergateOut --> uppergateOut;
        mgmt.uppergateIn <-- uppergateIn;

}

