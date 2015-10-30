package inet.underTest.linklayer.ieee802154.phyLayer;
simple Ieee802154Phy
{
    parameters:

        bool debug;
        double channelNumber = default(0);

        // power used for transmission of messages (mW)
        double transmitterPower @unit("mW") = default(1.0mW);

        // received signals with power below sensitivity are ignored (dBm)
        double sensitivity @unit("dBm")=default(-85dBm);

        // base noise level (dBm)
        double thermalNoise @unit("dBm") = default(-110dBm);

        // used by PathLossReceptionModel
        double pathLossAlpha = default(2);
        double shadowingDeviation @unit("dB") = default(0dB); // used by the shadowing model calculation

        // used by Ieee802154RadioModel
        double snirThreshold @unit("dB") = default(4dB); 
        double carrierFrequency @unit("Hz") = default(2.4GHz); 
                // parameters for energy usage
        double usage_radio_idle @unit("mA");
        double usage_radio_recv @unit("mA");
        double usage_radio_sleep @unit("mA");


    gates:
        input uppergateIn;			// from Mac
        output uppergateOut;			// to Mac
        input radioIn; 					// to receive frames (AirFrame) on the radio channel
}
