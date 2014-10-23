#
# OMNeT++/OMNEST Makefile for libinet
#
# This file was generated with the command:
#  opp_makemake -f --deep --make-so -o inet -O out -XunderTest -lsqlite3
#

# Name of target to be created (-o option)
TARGET = libinet$(SHARED_LIB_SUFFIX)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I. \
    -Iapplications \
    -Iapplications/dhcp \
    -Iapplications/ethernet \
    -Iapplications/generic \
    -Iapplications/pingapp \
    -Iapplications/rtpapp \
    -Iapplications/sctpapp \
    -Iapplications/tcpapp \
    -Iapplications/traci \
    -Iapplications/trafgen \
    -Iapplications/udpapp \
    -Ibase \
    -Ibattery \
    -Iexperimental \
    -Iexperimental/linklayer \
    -Iexperimental/linklayer/ieee80211 \
    -Iexperimental/linklayer/ieee80211/hwmp \
    -Iexperimental/linklayer/ieee80211/mgmt \
    -Iexperimental/nodes \
    -Iexperimental/nodes/adhoc \
    -Ilinklayer \
    -Ilinklayer/contract \
    -Ilinklayer/ethernet \
    -Ilinklayer/etherswitch \
    -Ilinklayer/ext \
    -Ilinklayer/ieee80211 \
    -Ilinklayer/ieee80211/mac \
    -Ilinklayer/ieee80211/mgmt \
    -Ilinklayer/mf80211 \
    -Ilinklayer/mf80211/macLayer \
    -Ilinklayer/mf80211/phyLayer \
    -Ilinklayer/mf80211/phyLayer/decider \
    -Ilinklayer/mf80211/phyLayer/snrEval \
    -Ilinklayer/mfcore \
    -Ilinklayer/ppp \
    -Ilinklayer/radio \
    -Ilinklayer/radio/errormodel \
    -Ilinklayer/radio/propagation \
    -Imobility \
    -Imobility/traci \
    -Inetworklayer \
    -Inetworklayer/arp \
    -Inetworklayer/autorouting \
    -Inetworklayer/common \
    -Inetworklayer/contract \
    -Inetworklayer/extras \
    -Inetworklayer/icmpv6 \
    -Inetworklayer/ipv4 \
    -Inetworklayer/ipv6 \
    -Inetworklayer/ldp \
    -Inetworklayer/manetrouting \
    -Inetworklayer/manetrouting/aodv \
    -Inetworklayer/manetrouting/aodv/aodv-uu \
    -Inetworklayer/manetrouting/base \
    -Inetworklayer/manetrouting/batman \
    -Inetworklayer/manetrouting/dsdv \
    -Inetworklayer/manetrouting/dsr \
    -Inetworklayer/manetrouting/dsr/dsr-uu \
    -Inetworklayer/manetrouting/dymo \
    -Inetworklayer/manetrouting/dymo/dymoum \
    -Inetworklayer/manetrouting/dymo_fau \
    -Inetworklayer/manetrouting/manager \
    -Inetworklayer/manetrouting/olsr \
    -Inetworklayer/mpls \
    -Inetworklayer/ospfv2 \
    -Inetworklayer/ospfv2/interface \
    -Inetworklayer/ospfv2/messagehandler \
    -Inetworklayer/ospfv2/neighbor \
    -Inetworklayer/ospfv2/router \
    -Inetworklayer/queue \
    -Inetworklayer/rsvp_te \
    -Inetworklayer/ted \
    -Inodes \
    -Inodes/adhoc \
    -Inodes/ethernet \
    -Inodes/inet \
    -Inodes/ipv6 \
    -Inodes/mpls \
    -Inodes/wireless \
    -Itransport \
    -Itransport/contract \
    -Itransport/rtp \
    -Itransport/rtp/profiles \
    -Itransport/rtp/profiles/avprofile \
    -Itransport/sctp \
    -Itransport/tcp \
    -Itransport/tcp/flavours \
    -Itransport/tcp/queues \
    -Itransport/tcp_nsc \
    -Itransport/tcp_nsc/queues \
    -Itransport/tcp_old \
    -Itransport/tcp_old/flavours \
    -Itransport/tcp_old/queues \
    -Itransport/udp \
    -Iutil \
    -Iutil/headerserializers \
    -Iutil/headerserializers/headers \
    -Iworld \
    -Iworld/annotations \
    -Iworld/obstacles \
    -Iworld/shadowing \
    -Iworld/traci

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS =  -lsqlite3

# Output directory
PROJECT_OUTPUT_DIR = ../out
PROJECTRELATIVE_PATH = /src
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS = \
    $O/applications/dhcp/DHCPClient.o \
    $O/applications/dhcp/DHCPServer.o \
    $O/applications/ethernet/EtherAppCli.o \
    $O/applications/ethernet/EtherAppSrv.o \
    $O/applications/generic/IPTrafGen.o \
    $O/applications/pingapp/PingApp.o \
    $O/applications/rtpapp/RTPApplication.o \
    $O/applications/sctpapp/SCTPClient.o \
    $O/applications/sctpapp/SCTPPeer.o \
    $O/applications/sctpapp/SCTPServer.o \
    $O/applications/tcpapp/TCPBasicClientApp.o \
    $O/applications/tcpapp/TCPEchoApp.o \
    $O/applications/tcpapp/TCPGenericCliAppBase.o \
    $O/applications/tcpapp/TCPGenericSrvApp.o \
    $O/applications/tcpapp/TCPGenericSrvThread.o \
    $O/applications/tcpapp/TCPSessionApp.o \
    $O/applications/tcpapp/TCPSinkApp.o \
    $O/applications/tcpapp/TCPSpoof.o \
    $O/applications/tcpapp/TCPSrvHostApp.o \
    $O/applications/tcpapp/TelnetApp.o \
    $O/applications/traci/TraCIDemo.o \
    $O/applications/traci/TraCITestApp.o \
    $O/applications/trafgen/TrafGen.o \
    $O/applications/trafgen/TrafGenPar.o \
    $O/applications/udpapp/UDPAppBase.o \
    $O/applications/udpapp/UDPBasicApp.o \
    $O/applications/udpapp/UDPBasicBurst.o \
    $O/applications/udpapp/UDPBasicBurst2.o \
    $O/applications/udpapp/UDPEchoApp.o \
    $O/applications/udpapp/UDPSink.o \
    $O/applications/udpapp/UDPVideoStreamCli.o \
    $O/applications/udpapp/UDPVideoStreamSvr.o \
    $O/base/AbstractQueue.o \
    $O/base/BasicModule.o \
    $O/base/Blackboard.o \
    $O/base/ByteArrayMessage.o \
    $O/base/Join.o \
    $O/base/MMapBoard.o \
    $O/base/ModuleAccess.o \
    $O/base/NotificationBoard.o \
    $O/base/NotifierConsts.o \
    $O/base/PassiveQueueBase.o \
    $O/base/ProtocolMap.o \
    $O/base/QueueBase.o \
    $O/base/QueueWithQoS.o \
    $O/base/ReassemblyBuffer.o \
    $O/base/Sink.o \
    $O/battery/BasicBattery.o \
    $O/battery/InetSimpleBattery.o \
    $O/experimental/linklayer/ieee80211/hwmp/hwmp-rtable.o \
    $O/experimental/linklayer/ieee80211/hwmp/hwmp.o \
    $O/experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.o \
    $O/experimental/linklayer/ieee80211/mgmt/Ieee80211Mesh.o \
    $O/experimental/linklayer/ieee80211/mgmt/Ieee80211MeshLwmpls.o \
    $O/experimental/linklayer/ieee80211/mgmt/Ieee80211MgmtAdhocWithEtx.o \
    $O/experimental/linklayer/ieee80211/mgmt/lwmpls_data.o \
    $O/linklayer/contract/MACAddress.o \
    $O/linklayer/ethernet/EtherBus.o \
    $O/linklayer/ethernet/EtherEncap.o \
    $O/linklayer/ethernet/EtherHub.o \
    $O/linklayer/ethernet/EtherLLC.o \
    $O/linklayer/ethernet/EtherMAC.o \
    $O/linklayer/ethernet/EtherMAC2.o \
    $O/linklayer/ethernet/EtherMACBase.o \
    $O/linklayer/etherswitch/MACRelayUnitBase.o \
    $O/linklayer/etherswitch/MACRelayUnitNP.o \
    $O/linklayer/etherswitch/MACRelayUnitPP.o \
    $O/linklayer/ext/cSocketRTScheduler.o \
    $O/linklayer/ext/ExtInterface.o \
    $O/linklayer/ieee80211/mac/BlockAckCache.o \
    $O/linklayer/ieee80211/mac/BlockAckFrame.o \
    $O/linklayer/ieee80211/mac/FrameBlock.o \
    $O/linklayer/ieee80211/mac/Ieee80211aMac.o \
    $O/linklayer/ieee80211/mac/Ieee80211egMac.o \
    $O/linklayer/ieee80211/mac/Ieee80211eMac.o \
    $O/linklayer/ieee80211/mac/Ieee80211gMac.o \
    $O/linklayer/ieee80211/mac/Ieee80211Mac.o \
    $O/linklayer/ieee80211/mac/Ieee80211NewMac.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211AgentSTA.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211eClassifier.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtAdhoc.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtAP.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtAPBase.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtAPSimplified.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtBase.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtSTA.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtSTASimplified.o \
    $O/linklayer/mf80211/macLayer/CSMAMacLayer.o \
    $O/linklayer/mf80211/macLayer/Mac80211.o \
    $O/linklayer/mf80211/phyLayer/decider/Decider80211.o \
    $O/linklayer/mf80211/phyLayer/decider/ErrAndCollDecider.o \
    $O/linklayer/mf80211/phyLayer/decider/SnrDecider.o \
    $O/linklayer/mf80211/phyLayer/snrEval/GilbertElliotSnr.o \
    $O/linklayer/mf80211/phyLayer/snrEval/SnrEval.o \
    $O/linklayer/mf80211/phyLayer/snrEval/SnrEval80211.o \
    $O/linklayer/mfcore/BasicDecider.o \
    $O/linklayer/mfcore/BasicSnrEval.o \
    $O/linklayer/mfcore/WirelessMacBase.o \
    $O/linklayer/ppp/DropsGenerator.o \
    $O/linklayer/ppp/DuplicatesGenerator.o \
    $O/linklayer/ppp/PPP.o \
    $O/linklayer/ppp/ThruputMeter.o \
    $O/linklayer/radio/AbstractRadio.o \
    $O/linklayer/radio/AbstractRadioExtended.o \
    $O/linklayer/radio/BerParseFile.o \
    $O/linklayer/radio/bpskMode.o \
    $O/linklayer/radio/GenericRadio.o \
    $O/linklayer/radio/GenericRadioModel.o \
    $O/linklayer/radio/Ieee80211aRadio.o \
    $O/linklayer/radio/Ieee80211aRadioModel.o \
    $O/linklayer/radio/Ieee80211gRadio.o \
    $O/linklayer/radio/Ieee80211gRadioModel.o \
    $O/linklayer/radio/Ieee80211NewRadio.o \
    $O/linklayer/radio/Ieee80211NewRadioModel.o \
    $O/linklayer/radio/Ieee80211Radio.o \
    $O/linklayer/radio/Ieee80211RadioModel.o \
    $O/linklayer/radio/Modulation.o \
    $O/linklayer/radio/PathLossReceptionModel.o \
    $O/linklayer/radio/qamMode.o \
    $O/linklayer/radio/transmissionMode.o \
    $O/linklayer/radio/TwoRayModel.o \
    $O/linklayer/radio/errormodel/dsss-error-rate-model.o \
    $O/linklayer/radio/errormodel/WifiMode.o \
    $O/linklayer/radio/errormodel/yans-error-rate-model.o \
    $O/linklayer/radio/propagation/FreeSpaceModel.o \
    $O/linklayer/radio/propagation/LogNormalShadowingModel.o \
    $O/linklayer/radio/propagation/NakagamiModel.o \
    $O/linklayer/radio/propagation/RayleighModel.o \
    $O/linklayer/radio/propagation/RiceModel.o \
    $O/linklayer/radio/propagation/TwoRayGroundModel.o \
    $O/linklayer/radio/propagation/TwoRayInterferenceModel.o \
    $O/mobility/ANSimMobility.o \
    $O/mobility/BasicMobility.o \
    $O/mobility/BonnMotionFileCache.o \
    $O/mobility/BonnMotionMobility.o \
    $O/mobility/ChiangMobility.o \
    $O/mobility/CircleMobility.o \
    $O/mobility/ConstSpeedMobility.o \
    $O/mobility/GaussMarkovMobility.o \
    $O/mobility/LinearMobility.o \
    $O/mobility/LineSegmentsMobilityBase.o \
    $O/mobility/MassMobility.o \
    $O/mobility/Ns2MotionMobility.o \
    $O/mobility/NullMobility.o \
    $O/mobility/RandomWPMobility.o \
    $O/mobility/RandomWPMobilityTrace.o \
    $O/mobility/RectangleMobility.o \
    $O/mobility/RestrictedConstSpeedMobility.o \
    $O/mobility/RestrictedLinearMobility.o \
    $O/mobility/RestrictedRandomWPMobility.o \
    $O/mobility/StaticGridMobility.o \
    $O/mobility/TurtleMobility.o \
    $O/mobility/traci/TraCIMobility.o \
    $O/networklayer/arp/ARP.o \
    $O/networklayer/autorouting/FlatNetworkConfigurator.o \
    $O/networklayer/autorouting/FlatNetworkConfigurator6.o \
    $O/networklayer/autorouting/HostAutoConfigurator.o \
    $O/networklayer/autorouting/HostAutoConfigurator2.o \
    $O/networklayer/autorouting/NetworkConfigurator.o \
    $O/networklayer/common/InterfaceEntry.o \
    $O/networklayer/common/InterfaceTable.o \
    $O/networklayer/contract/IPAddress.o \
    $O/networklayer/contract/IPAddressResolver.o \
    $O/networklayer/contract/IPControlInfo.o \
    $O/networklayer/contract/IPv6Address.o \
    $O/networklayer/contract/IPv6ControlInfo.o \
    $O/networklayer/contract/IPvXAddress.o \
    $O/networklayer/extras/Dummy.o \
    $O/networklayer/extras/FailureManager.o \
    $O/networklayer/extras/NetworkInfo.o \
    $O/networklayer/icmpv6/ICMPv6.o \
    $O/networklayer/icmpv6/IPv6NeighbourCache.o \
    $O/networklayer/icmpv6/IPv6NeighbourDiscovery.o \
    $O/networklayer/ipv4/ErrorHandling.o \
    $O/networklayer/ipv4/ICMP.o \
    $O/networklayer/ipv4/IP.o \
    $O/networklayer/ipv4/IPFragBuf.o \
    $O/networklayer/ipv4/IPRoute.o \
    $O/networklayer/ipv4/IPRouteRule.o \
    $O/networklayer/ipv4/IPv4InterfaceData.o \
    $O/networklayer/ipv4/RoutingTable.o \
    $O/networklayer/ipv4/RoutingTableParser.o \
    $O/networklayer/ipv6/IPv6.o \
    $O/networklayer/ipv6/IPv6Datagram.o \
    $O/networklayer/ipv6/IPv6ErrorHandling.o \
    $O/networklayer/ipv6/IPv6FragBuf.o \
    $O/networklayer/ipv6/IPv6InterfaceData.o \
    $O/networklayer/ipv6/RoutingTable6.o \
    $O/networklayer/ldp/LDP.o \
    $O/networklayer/manetrouting/aodv/aodv_msg_struct.o \
    $O/networklayer/manetrouting/aodv/aodv_uu_omnet.o \
    $O/networklayer/manetrouting/aodv/packet_queue_omnet.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/aodv_hello.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/aodv_socket.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/debug_aodv.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/list.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/locality.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/routing_table.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/seek_list.o \
    $O/networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.o \
    $O/networklayer/manetrouting/base/int128.o \
    $O/networklayer/manetrouting/base/ManetRoutingBase.o \
    $O/networklayer/manetrouting/base/uint128.o \
    $O/networklayer/manetrouting/batman/BatmanData.o \
    $O/networklayer/manetrouting/batman/batmanHna.o \
    $O/networklayer/manetrouting/batman/BatmanMain.o \
    $O/networklayer/manetrouting/dsdv/DSDV_2.o \
    $O/networklayer/manetrouting/dsr/dsr-pkt_omnet.o \
    $O/networklayer/manetrouting/dsr/dsr-uu-omnetpp.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-ack.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-io.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-opt.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/dsr-srt.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/link-cache.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/maint-buf.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/neigh.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/path-cache.o \
    $O/networklayer/manetrouting/dsr/dsr-uu/send-buf.o \
    $O/networklayer/manetrouting/dymo/dymo_msg_struct.o \
    $O/networklayer/manetrouting/dymo/dymo_packet_queue_omnet.o \
    $O/networklayer/manetrouting/dymo/dymo_um_omnet.o \
    $O/networklayer/manetrouting/dymo/dymoum/blacklist.o \
    $O/networklayer/manetrouting/dymo/dymoum/debug_dymo.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_generic.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_hello.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_nb.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_re.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_rerr.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_socket.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_timeout.o \
    $O/networklayer/manetrouting/dymo/dymoum/dymo_uerr.o \
    $O/networklayer/manetrouting/dymo/dymoum/icmp_socket.o \
    $O/networklayer/manetrouting/dymo/dymoum/pending_rreq.o \
    $O/networklayer/manetrouting/dymo/dymoum/rtable.o \
    $O/networklayer/manetrouting/dymo/dymoum/timer_queue.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_DataQueue.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_RoutingEntry.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_RoutingTable.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_Timer.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_TokenBucket.o \
    $O/networklayer/manetrouting/dymo_fau/InspectorHelpers.o \
    $O/networklayer/manetrouting/manager/ManetManager.o \
    $O/networklayer/manetrouting/olsr/OLSR.o \
    $O/networklayer/manetrouting/olsr/OLSR_ETX.o \
    $O/networklayer/manetrouting/olsr/OLSR_ETX_dijkstra.o \
    $O/networklayer/manetrouting/olsr/OLSR_ETX_state.o \
    $O/networklayer/manetrouting/olsr/OLSR_rtable.o \
    $O/networklayer/manetrouting/olsr/OLSR_state.o \
    $O/networklayer/mpls/LIBTable.o \
    $O/networklayer/mpls/MPLS.o \
    $O/networklayer/mpls/MPLSPacket.o \
    $O/networklayer/ospfv2/OSPFRouting.o \
    $O/networklayer/ospfv2/interface/OSPFInterface.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceState.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceStateBackup.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceStateDesignatedRouter.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceStateDown.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceStateNotDesignatedRouter.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceStatePointToPoint.o \
    $O/networklayer/ospfv2/interface/OSPFInterfaceStateWaiting.o \
    $O/networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.o \
    $O/networklayer/ospfv2/messagehandler/HelloHandler.o \
    $O/networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.o \
    $O/networklayer/ospfv2/messagehandler/LinkStateRequestHandler.o \
    $O/networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.o \
    $O/networklayer/ospfv2/messagehandler/MessageHandler.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighbor.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborState.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateAttempt.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateDown.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateExchange.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateFull.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateInit.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateLoading.o \
    $O/networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.o \
    $O/networklayer/ospfv2/router/ASExternalLSA.o \
    $O/networklayer/ospfv2/router/NetworkLSA.o \
    $O/networklayer/ospfv2/router/OSPFArea.o \
    $O/networklayer/ospfv2/router/OSPFRouter.o \
    $O/networklayer/ospfv2/router/RouterLSA.o \
    $O/networklayer/ospfv2/router/SummaryLSA.o \
    $O/networklayer/queue/BasicDSCPClassifier.o \
    $O/networklayer/queue/DropTailQoSQueue.o \
    $O/networklayer/queue/DropTailQueue.o \
    $O/networklayer/queue/REDQueue.o \
    $O/networklayer/queue/WeightedFairQueue.o \
    $O/networklayer/rsvp_te/RSVP.o \
    $O/networklayer/rsvp_te/SimpleClassifier.o \
    $O/networklayer/rsvp_te/Utils.o \
    $O/networklayer/ted/LinkStateRouting.o \
    $O/networklayer/ted/TED.o \
    $O/transport/contract/SCTPSocket.o \
    $O/transport/contract/TCPSocket.o \
    $O/transport/contract/TCPSocketMap.o \
    $O/transport/contract/UDPSocket.o \
    $O/transport/rtp/reports.o \
    $O/transport/rtp/RTCP.o \
    $O/transport/rtp/RTCPPacket.o \
    $O/transport/rtp/RTP.o \
    $O/transport/rtp/RTPInnerPacket.o \
    $O/transport/rtp/RTPInterfacePacket.o \
    $O/transport/rtp/RTPPacket.o \
    $O/transport/rtp/RTPParticipantInfo.o \
    $O/transport/rtp/RTPPayloadReceiver.o \
    $O/transport/rtp/RTPPayloadSender.o \
    $O/transport/rtp/RTPProfile.o \
    $O/transport/rtp/RTPReceiverInfo.o \
    $O/transport/rtp/RTPSenderInfo.o \
    $O/transport/rtp/sdes.o \
    $O/transport/rtp/profiles/avprofile/RTPAVProfile.o \
    $O/transport/rtp/profiles/avprofile/RTPAVProfilePayload32Receiver.o \
    $O/transport/rtp/profiles/avprofile/RTPAVProfilePayload32Sender.o \
    $O/transport/sctp/SCTP.o \
    $O/transport/sctp/SCTPAlg.o \
    $O/transport/sctp/SCTPAssociationBase.o \
    $O/transport/sctp/SCTPAssociationEventProc.o \
    $O/transport/sctp/SCTPAssociationRcvMessage.o \
    $O/transport/sctp/SCTPAssociationSendAll.o \
    $O/transport/sctp/SCTPAssociationUtil.o \
    $O/transport/sctp/SCTPCCFunctions.o \
    $O/transport/sctp/SCTPMessage.o \
    $O/transport/sctp/SCTPQueue.o \
    $O/transport/sctp/SCTPReceiveStream.o \
    $O/transport/sctp/SCTPSendStream.o \
    $O/transport/sctp/SCTPSSFunctions.o \
    $O/transport/tcp/TCP.o \
    $O/transport/tcp/TCPConnectionBase.o \
    $O/transport/tcp/TCPConnectionEventProc.o \
    $O/transport/tcp/TCPConnectionRcvSegment.o \
    $O/transport/tcp/TCPConnectionUtil.o \
    $O/transport/tcp/TCPSACKRexmitQueue.o \
    $O/transport/tcp/TCPSegment.o \
    $O/transport/tcp/flavours/DumbTCP.o \
    $O/transport/tcp/flavours/TCPBaseAlg.o \
    $O/transport/tcp/flavours/TCPNewReno.o \
    $O/transport/tcp/flavours/TCPNoCongestionControl.o \
    $O/transport/tcp/flavours/TCPReno.o \
    $O/transport/tcp/flavours/TCPTahoe.o \
    $O/transport/tcp/flavours/TCPTahoeRenoFamily.o \
    $O/transport/tcp/queues/TCPMsgBasedRcvQueue.o \
    $O/transport/tcp/queues/TCPMsgBasedSendQueue.o \
    $O/transport/tcp/queues/TCPVirtualDataRcvQueue.o \
    $O/transport/tcp/queues/TCPVirtualDataSendQueue.o \
    $O/transport/tcp_nsc/TCP_NSC.o \
    $O/transport/tcp_nsc/TCP_NSC_Connection.o \
    $O/transport/tcp_nsc/queues/TCP_NSC_VirtualDataQueues.o \
    $O/transport/tcp_old/TCP.o \
    $O/transport/tcp_old/TCPConnectionBase.o \
    $O/transport/tcp_old/TCPConnectionEventProc.o \
    $O/transport/tcp_old/TCPConnectionRcvSegment.o \
    $O/transport/tcp_old/TCPConnectionUtil.o \
    $O/transport/tcp_old/flavours/DumbTCP.o \
    $O/transport/tcp_old/flavours/TCPBaseAlg.o \
    $O/transport/tcp_old/flavours/TCPNoCongestionControl.o \
    $O/transport/tcp_old/flavours/TCPReno.o \
    $O/transport/tcp_old/flavours/TCPTahoe.o \
    $O/transport/tcp_old/flavours/TCPTahoeRenoFamily.o \
    $O/transport/tcp_old/queues/TCPMsgBasedRcvQueue.o \
    $O/transport/tcp_old/queues/TCPMsgBasedSendQueue.o \
    $O/transport/tcp_old/queues/TCPVirtualDataRcvQueue.o \
    $O/transport/tcp_old/queues/TCPVirtualDataSendQueue.o \
    $O/transport/udp/UDP.o \
    $O/util/common.o \
    $O/util/dijktraKShortest.o \
    $O/util/NAMTraceWriter.o \
    $O/util/NetAnimTrace.o \
    $O/util/opp_utils.o \
    $O/util/PowerControlManager.o \
    $O/util/TCPDump.o \
    $O/util/ThruputMeteringChannel.o \
    $O/util/XMLUtils.o \
    $O/util/headerserializers/ICMPSerializer.o \
    $O/util/headerserializers/IPSerializer.o \
    $O/util/headerserializers/SCTPSerializer.o \
    $O/util/headerserializers/TCPIPchecksum.o \
    $O/util/headerserializers/TCPSerializer.o \
    $O/util/headerserializers/UDPSerializer.o \
    $O/world/ChannelAccess.o \
    $O/world/ChannelAccessExtended.o \
    $O/world/ChannelControl.o \
    $O/world/ChannelControlExtended.o \
    $O/world/ChannelInstaller.o \
    $O/world/NAMTrace.o \
    $O/world/ScenarioManager.o \
    $O/world/annotations/AnnotationDummy.o \
    $O/world/annotations/AnnotationManager.o \
    $O/world/obstacles/Obstacle.o \
    $O/world/obstacles/ObstacleControl.o \
    $O/world/shadowing/DiffractionPropagationLossModel.o \
    $O/world/shadowing/MapManager.o \
    $O/world/shadowing/PenetrationPropagationLossModel.o \
    $O/world/shadowing/ShadowPropagationLossModel.o \
    $O/world/traci/TraCIScenarioManager.o \
    $O/world/traci/TraCIScenarioManagerLaunchd.o \
    $O/applications/dhcp/DHCP_m.o \
    $O/applications/ethernet/EtherApp_m.o \
    $O/applications/pingapp/PingPayload_m.o \
    $O/applications/tcpapp/GenericAppMsg_m.o \
    $O/applications/udpapp/UDPEchoAppMsg_m.o \
    $O/base/ByteArrayMessage_m.o \
    $O/experimental/linklayer/ieee80211/mgmt/ETXPacket_m.o \
    $O/experimental/linklayer/ieee80211/mgmt/LWMPLSPacket_m.o \
    $O/linklayer/contract/ControlInfoBreakLink_m.o \
    $O/linklayer/contract/Ieee802Ctrl_m.o \
    $O/linklayer/contract/MeshControlInfo_m.o \
    $O/linklayer/contract/PhyControlInfo_m.o \
    $O/linklayer/contract/Radio80211aControlInfo_m.o \
    $O/linklayer/ethernet/EtherFrame_m.o \
    $O/linklayer/ext/ExtFrame_m.o \
    $O/linklayer/ieee80211/mac/Ieee80211Frame_m.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.o \
    $O/linklayer/ieee80211/mgmt/Ieee80211Primitives_m.o \
    $O/linklayer/mf80211/macLayer/Mac80211Pkt_m.o \
    $O/linklayer/mf80211/macLayer/MacPkt_m.o \
    $O/linklayer/mfcore/AirFrame_m.o \
    $O/linklayer/mfcore/SnrControlInfo_m.o \
    $O/linklayer/mfcore/TransmComplete_m.o \
    $O/linklayer/ppp/PPPFrame_m.o \
    $O/networklayer/arp/ARPPacket_m.o \
    $O/networklayer/contract/IPControlInfo_m.o \
    $O/networklayer/contract/IPProtocolId_m.o \
    $O/networklayer/contract/IPv6ControlInfo_m.o \
    $O/networklayer/icmpv6/ICMPv6Message_m.o \
    $O/networklayer/icmpv6/IPv6NDMessage_m.o \
    $O/networklayer/ipv4/ControlManetRouting_m.o \
    $O/networklayer/ipv4/ICMPMessage_m.o \
    $O/networklayer/ipv4/IPDatagram_m.o \
    $O/networklayer/ipv6/IPv6Datagram_m.o \
    $O/networklayer/ipv6/IPv6ExtensionHeaders_m.o \
    $O/networklayer/ldp/LDPPacket_m.o \
    $O/networklayer/manetrouting/batman/BatmanMsg_m.o \
    $O/networklayer/manetrouting/dsdv/DSDVhello_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_Packet_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_PacketBBMessage_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_RERR_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_RM_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_RREP_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_RREQ_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_Timeout_m.o \
    $O/networklayer/manetrouting/dymo_fau/DYMO_UERR_m.o \
    $O/networklayer/manetrouting/olsr/OLSRpkt_m.o \
    $O/networklayer/ospfv2/OSPFPacket_m.o \
    $O/networklayer/ospfv2/OSPFTimer_m.o \
    $O/networklayer/rsvp_te/IntServ_m.o \
    $O/networklayer/rsvp_te/RSVPHello_m.o \
    $O/networklayer/rsvp_te/RSVPPacket_m.o \
    $O/networklayer/rsvp_te/RSVPPathMsg_m.o \
    $O/networklayer/rsvp_te/RSVPResvMsg_m.o \
    $O/networklayer/rsvp_te/SignallingMsg_m.o \
    $O/networklayer/ted/LinkStatePacket_m.o \
    $O/networklayer/ted/TED_m.o \
    $O/transport/contract/SCTPCommand_m.o \
    $O/transport/contract/TCPCommand_m.o \
    $O/transport/contract/UDPControlInfo_m.o \
    $O/transport/rtp/RTPSenderControlMessage_m.o \
    $O/transport/rtp/RTPSenderStatusMessage_m.o \
    $O/transport/rtp/profiles/avprofile/RTPMpegPacket_m.o \
    $O/transport/sctp/SCTPMessage_m.o \
    $O/transport/tcp/TCPSegment_m.o \
    $O/transport/udp/UDPPacket_m.o \
    $O/util/PowerControlMessages_m.o

# Message files
MSGFILES = \
    applications/dhcp/DHCP.msg \
    applications/ethernet/EtherApp.msg \
    applications/pingapp/PingPayload.msg \
    applications/tcpapp/GenericAppMsg.msg \
    applications/udpapp/UDPEchoAppMsg.msg \
    base/ByteArrayMessage.msg \
    experimental/linklayer/ieee80211/mgmt/ETXPacket.msg \
    experimental/linklayer/ieee80211/mgmt/LWMPLSPacket.msg \
    linklayer/contract/ControlInfoBreakLink.msg \
    linklayer/contract/Ieee802Ctrl.msg \
    linklayer/contract/MeshControlInfo.msg \
    linklayer/contract/PhyControlInfo.msg \
    linklayer/contract/Radio80211aControlInfo.msg \
    linklayer/ethernet/EtherFrame.msg \
    linklayer/ext/ExtFrame.msg \
    linklayer/ieee80211/mac/Ieee80211Frame.msg \
    linklayer/ieee80211/mgmt/Ieee80211MgmtFrames.msg \
    linklayer/ieee80211/mgmt/Ieee80211Primitives.msg \
    linklayer/mf80211/macLayer/Mac80211Pkt.msg \
    linklayer/mf80211/macLayer/MacPkt.msg \
    linklayer/mfcore/AirFrame.msg \
    linklayer/mfcore/SnrControlInfo.msg \
    linklayer/mfcore/TransmComplete.msg \
    linklayer/ppp/PPPFrame.msg \
    networklayer/arp/ARPPacket.msg \
    networklayer/contract/IPControlInfo.msg \
    networklayer/contract/IPProtocolId.msg \
    networklayer/contract/IPv6ControlInfo.msg \
    networklayer/icmpv6/ICMPv6Message.msg \
    networklayer/icmpv6/IPv6NDMessage.msg \
    networklayer/ipv4/ControlManetRouting.msg \
    networklayer/ipv4/ICMPMessage.msg \
    networklayer/ipv4/IPDatagram.msg \
    networklayer/ipv6/IPv6Datagram.msg \
    networklayer/ipv6/IPv6ExtensionHeaders.msg \
    networklayer/ldp/LDPPacket.msg \
    networklayer/manetrouting/batman/BatmanMsg.msg \
    networklayer/manetrouting/dsdv/DSDVhello.msg \
    networklayer/manetrouting/dymo_fau/DYMO_Packet.msg \
    networklayer/manetrouting/dymo_fau/DYMO_PacketBBMessage.msg \
    networklayer/manetrouting/dymo_fau/DYMO_RERR.msg \
    networklayer/manetrouting/dymo_fau/DYMO_RM.msg \
    networklayer/manetrouting/dymo_fau/DYMO_RREP.msg \
    networklayer/manetrouting/dymo_fau/DYMO_RREQ.msg \
    networklayer/manetrouting/dymo_fau/DYMO_Timeout.msg \
    networklayer/manetrouting/dymo_fau/DYMO_UERR.msg \
    networklayer/manetrouting/olsr/OLSRpkt.msg \
    networklayer/ospfv2/OSPFPacket.msg \
    networklayer/ospfv2/OSPFTimer.msg \
    networklayer/rsvp_te/IntServ.msg \
    networklayer/rsvp_te/RSVPHello.msg \
    networklayer/rsvp_te/RSVPPacket.msg \
    networklayer/rsvp_te/RSVPPathMsg.msg \
    networklayer/rsvp_te/RSVPResvMsg.msg \
    networklayer/rsvp_te/SignallingMsg.msg \
    networklayer/ted/LinkStatePacket.msg \
    networklayer/ted/TED.msg \
    transport/contract/SCTPCommand.msg \
    transport/contract/TCPCommand.msg \
    transport/contract/UDPControlInfo.msg \
    transport/rtp/RTPSenderControlMessage.msg \
    transport/rtp/RTPSenderStatusMessage.msg \
    transport/rtp/profiles/avprofile/RTPMpegPacket.msg \
    transport/sctp/SCTPMessage.msg \
    transport/tcp/TCPSegment.msg \
    transport/udp/UDPPacket.msg \
    util/PowerControlMessages.msg

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppenvir$D $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# inserted from file 'makefrag':
# if you want to use external interfaces with inet, you should install
# winpcap (on windows) or libpcap-dev (on linux) and re-run ./configure in
# the omnetpp root directory and comment out the following line:
HAVE_PCAP=yes

ifeq ($(HAVE_PCAP),yes)
  # link with PCAP libs too
  LIBS += $(PCAP_LIBS)
else
  # remove the HAVE_PCAP define if we do not need PCAP
  CFLAGS := $(filter-out -DHAVE_PCAP,$(CFLAGS))
endif

# TCP implementaion using the Network Simulation Cradle
NSC_VERSION= $(shell ls -d ../3rdparty/nsc* 2>/dev/null | sed 's/^.*-//')

ifneq ($(NSC_VERSION),)
  CFLAGS += -DWITH_TCP_NSC -I../3rdparty/nsc-$(NSC_VERSION)/sim
  LIBS += -Wl,-rpath,`abspath ../3rdparty/nsc-$(NSC_VERSION)`
endif

# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating shared library: $@
	$(Q)$(SHLIB_LD) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS) $(LDFLAGS)
	$(Q)$(SHLIB_POSTPROCESS) $O/$(TARGET)

.PHONY: all clean cleanall depend msgheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f inet inet.exe libinet.so libinet.a libinet.dll libinet.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h
	$(Q)-rm -f applications/*_m.cc applications/*_m.h
	$(Q)-rm -f applications/dhcp/*_m.cc applications/dhcp/*_m.h
	$(Q)-rm -f applications/ethernet/*_m.cc applications/ethernet/*_m.h
	$(Q)-rm -f applications/generic/*_m.cc applications/generic/*_m.h
	$(Q)-rm -f applications/pingapp/*_m.cc applications/pingapp/*_m.h
	$(Q)-rm -f applications/rtpapp/*_m.cc applications/rtpapp/*_m.h
	$(Q)-rm -f applications/sctpapp/*_m.cc applications/sctpapp/*_m.h
	$(Q)-rm -f applications/tcpapp/*_m.cc applications/tcpapp/*_m.h
	$(Q)-rm -f applications/traci/*_m.cc applications/traci/*_m.h
	$(Q)-rm -f applications/trafgen/*_m.cc applications/trafgen/*_m.h
	$(Q)-rm -f applications/udpapp/*_m.cc applications/udpapp/*_m.h
	$(Q)-rm -f base/*_m.cc base/*_m.h
	$(Q)-rm -f battery/*_m.cc battery/*_m.h
	$(Q)-rm -f experimental/*_m.cc experimental/*_m.h
	$(Q)-rm -f experimental/linklayer/*_m.cc experimental/linklayer/*_m.h
	$(Q)-rm -f experimental/linklayer/ieee80211/*_m.cc experimental/linklayer/ieee80211/*_m.h
	$(Q)-rm -f experimental/linklayer/ieee80211/hwmp/*_m.cc experimental/linklayer/ieee80211/hwmp/*_m.h
	$(Q)-rm -f experimental/linklayer/ieee80211/mgmt/*_m.cc experimental/linklayer/ieee80211/mgmt/*_m.h
	$(Q)-rm -f experimental/nodes/*_m.cc experimental/nodes/*_m.h
	$(Q)-rm -f experimental/nodes/adhoc/*_m.cc experimental/nodes/adhoc/*_m.h
	$(Q)-rm -f linklayer/*_m.cc linklayer/*_m.h
	$(Q)-rm -f linklayer/contract/*_m.cc linklayer/contract/*_m.h
	$(Q)-rm -f linklayer/ethernet/*_m.cc linklayer/ethernet/*_m.h
	$(Q)-rm -f linklayer/etherswitch/*_m.cc linklayer/etherswitch/*_m.h
	$(Q)-rm -f linklayer/ext/*_m.cc linklayer/ext/*_m.h
	$(Q)-rm -f linklayer/ieee80211/*_m.cc linklayer/ieee80211/*_m.h
	$(Q)-rm -f linklayer/ieee80211/mac/*_m.cc linklayer/ieee80211/mac/*_m.h
	$(Q)-rm -f linklayer/ieee80211/mgmt/*_m.cc linklayer/ieee80211/mgmt/*_m.h
	$(Q)-rm -f linklayer/mf80211/*_m.cc linklayer/mf80211/*_m.h
	$(Q)-rm -f linklayer/mf80211/macLayer/*_m.cc linklayer/mf80211/macLayer/*_m.h
	$(Q)-rm -f linklayer/mf80211/phyLayer/*_m.cc linklayer/mf80211/phyLayer/*_m.h
	$(Q)-rm -f linklayer/mf80211/phyLayer/decider/*_m.cc linklayer/mf80211/phyLayer/decider/*_m.h
	$(Q)-rm -f linklayer/mf80211/phyLayer/snrEval/*_m.cc linklayer/mf80211/phyLayer/snrEval/*_m.h
	$(Q)-rm -f linklayer/mfcore/*_m.cc linklayer/mfcore/*_m.h
	$(Q)-rm -f linklayer/ppp/*_m.cc linklayer/ppp/*_m.h
	$(Q)-rm -f linklayer/radio/*_m.cc linklayer/radio/*_m.h
	$(Q)-rm -f linklayer/radio/errormodel/*_m.cc linklayer/radio/errormodel/*_m.h
	$(Q)-rm -f linklayer/radio/propagation/*_m.cc linklayer/radio/propagation/*_m.h
	$(Q)-rm -f mobility/*_m.cc mobility/*_m.h
	$(Q)-rm -f mobility/traci/*_m.cc mobility/traci/*_m.h
	$(Q)-rm -f networklayer/*_m.cc networklayer/*_m.h
	$(Q)-rm -f networklayer/arp/*_m.cc networklayer/arp/*_m.h
	$(Q)-rm -f networklayer/autorouting/*_m.cc networklayer/autorouting/*_m.h
	$(Q)-rm -f networklayer/common/*_m.cc networklayer/common/*_m.h
	$(Q)-rm -f networklayer/contract/*_m.cc networklayer/contract/*_m.h
	$(Q)-rm -f networklayer/extras/*_m.cc networklayer/extras/*_m.h
	$(Q)-rm -f networklayer/icmpv6/*_m.cc networklayer/icmpv6/*_m.h
	$(Q)-rm -f networklayer/ipv4/*_m.cc networklayer/ipv4/*_m.h
	$(Q)-rm -f networklayer/ipv6/*_m.cc networklayer/ipv6/*_m.h
	$(Q)-rm -f networklayer/ldp/*_m.cc networklayer/ldp/*_m.h
	$(Q)-rm -f networklayer/manetrouting/*_m.cc networklayer/manetrouting/*_m.h
	$(Q)-rm -f networklayer/manetrouting/aodv/*_m.cc networklayer/manetrouting/aodv/*_m.h
	$(Q)-rm -f networklayer/manetrouting/aodv/aodv-uu/*_m.cc networklayer/manetrouting/aodv/aodv-uu/*_m.h
	$(Q)-rm -f networklayer/manetrouting/base/*_m.cc networklayer/manetrouting/base/*_m.h
	$(Q)-rm -f networklayer/manetrouting/batman/*_m.cc networklayer/manetrouting/batman/*_m.h
	$(Q)-rm -f networklayer/manetrouting/dsdv/*_m.cc networklayer/manetrouting/dsdv/*_m.h
	$(Q)-rm -f networklayer/manetrouting/dsr/*_m.cc networklayer/manetrouting/dsr/*_m.h
	$(Q)-rm -f networklayer/manetrouting/dsr/dsr-uu/*_m.cc networklayer/manetrouting/dsr/dsr-uu/*_m.h
	$(Q)-rm -f networklayer/manetrouting/dymo/*_m.cc networklayer/manetrouting/dymo/*_m.h
	$(Q)-rm -f networklayer/manetrouting/dymo/dymoum/*_m.cc networklayer/manetrouting/dymo/dymoum/*_m.h
	$(Q)-rm -f networklayer/manetrouting/dymo_fau/*_m.cc networklayer/manetrouting/dymo_fau/*_m.h
	$(Q)-rm -f networklayer/manetrouting/manager/*_m.cc networklayer/manetrouting/manager/*_m.h
	$(Q)-rm -f networklayer/manetrouting/olsr/*_m.cc networklayer/manetrouting/olsr/*_m.h
	$(Q)-rm -f networklayer/mpls/*_m.cc networklayer/mpls/*_m.h
	$(Q)-rm -f networklayer/ospfv2/*_m.cc networklayer/ospfv2/*_m.h
	$(Q)-rm -f networklayer/ospfv2/interface/*_m.cc networklayer/ospfv2/interface/*_m.h
	$(Q)-rm -f networklayer/ospfv2/messagehandler/*_m.cc networklayer/ospfv2/messagehandler/*_m.h
	$(Q)-rm -f networklayer/ospfv2/neighbor/*_m.cc networklayer/ospfv2/neighbor/*_m.h
	$(Q)-rm -f networklayer/ospfv2/router/*_m.cc networklayer/ospfv2/router/*_m.h
	$(Q)-rm -f networklayer/queue/*_m.cc networklayer/queue/*_m.h
	$(Q)-rm -f networklayer/rsvp_te/*_m.cc networklayer/rsvp_te/*_m.h
	$(Q)-rm -f networklayer/ted/*_m.cc networklayer/ted/*_m.h
	$(Q)-rm -f nodes/*_m.cc nodes/*_m.h
	$(Q)-rm -f nodes/adhoc/*_m.cc nodes/adhoc/*_m.h
	$(Q)-rm -f nodes/ethernet/*_m.cc nodes/ethernet/*_m.h
	$(Q)-rm -f nodes/inet/*_m.cc nodes/inet/*_m.h
	$(Q)-rm -f nodes/ipv6/*_m.cc nodes/ipv6/*_m.h
	$(Q)-rm -f nodes/mpls/*_m.cc nodes/mpls/*_m.h
	$(Q)-rm -f nodes/wireless/*_m.cc nodes/wireless/*_m.h
	$(Q)-rm -f transport/*_m.cc transport/*_m.h
	$(Q)-rm -f transport/contract/*_m.cc transport/contract/*_m.h
	$(Q)-rm -f transport/rtp/*_m.cc transport/rtp/*_m.h
	$(Q)-rm -f transport/rtp/profiles/*_m.cc transport/rtp/profiles/*_m.h
	$(Q)-rm -f transport/rtp/profiles/avprofile/*_m.cc transport/rtp/profiles/avprofile/*_m.h
	$(Q)-rm -f transport/sctp/*_m.cc transport/sctp/*_m.h
	$(Q)-rm -f transport/tcp/*_m.cc transport/tcp/*_m.h
	$(Q)-rm -f transport/tcp/flavours/*_m.cc transport/tcp/flavours/*_m.h
	$(Q)-rm -f transport/tcp/queues/*_m.cc transport/tcp/queues/*_m.h
	$(Q)-rm -f transport/tcp_nsc/*_m.cc transport/tcp_nsc/*_m.h
	$(Q)-rm -f transport/tcp_nsc/queues/*_m.cc transport/tcp_nsc/queues/*_m.h
	$(Q)-rm -f transport/tcp_old/*_m.cc transport/tcp_old/*_m.h
	$(Q)-rm -f transport/tcp_old/flavours/*_m.cc transport/tcp_old/flavours/*_m.h
	$(Q)-rm -f transport/tcp_old/queues/*_m.cc transport/tcp_old/queues/*_m.h
	$(Q)-rm -f transport/udp/*_m.cc transport/udp/*_m.h
	$(Q)-rm -f util/*_m.cc util/*_m.h
	$(Q)-rm -f util/headerserializers/*_m.cc util/headerserializers/*_m.h
	$(Q)-rm -f util/headerserializers/headers/*_m.cc util/headerserializers/headers/*_m.h
	$(Q)-rm -f world/*_m.cc world/*_m.h
	$(Q)-rm -f world/annotations/*_m.cc world/annotations/*_m.h
	$(Q)-rm -f world/obstacles/*_m.cc world/obstacles/*_m.h
	$(Q)-rm -f world/shadowing/*_m.cc world/shadowing/*_m.h
	$(Q)-rm -f world/traci/*_m.cc world/traci/*_m.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc applications/*.cc applications/dhcp/*.cc applications/ethernet/*.cc applications/generic/*.cc applications/pingapp/*.cc applications/rtpapp/*.cc applications/sctpapp/*.cc applications/tcpapp/*.cc applications/traci/*.cc applications/trafgen/*.cc applications/udpapp/*.cc base/*.cc battery/*.cc experimental/*.cc experimental/linklayer/*.cc experimental/linklayer/ieee80211/*.cc experimental/linklayer/ieee80211/hwmp/*.cc experimental/linklayer/ieee80211/mgmt/*.cc experimental/nodes/*.cc experimental/nodes/adhoc/*.cc linklayer/*.cc linklayer/contract/*.cc linklayer/ethernet/*.cc linklayer/etherswitch/*.cc linklayer/ext/*.cc linklayer/ieee80211/*.cc linklayer/ieee80211/mac/*.cc linklayer/ieee80211/mgmt/*.cc linklayer/mf80211/*.cc linklayer/mf80211/macLayer/*.cc linklayer/mf80211/phyLayer/*.cc linklayer/mf80211/phyLayer/decider/*.cc linklayer/mf80211/phyLayer/snrEval/*.cc linklayer/mfcore/*.cc linklayer/ppp/*.cc linklayer/radio/*.cc linklayer/radio/errormodel/*.cc linklayer/radio/propagation/*.cc mobility/*.cc mobility/traci/*.cc networklayer/*.cc networklayer/arp/*.cc networklayer/autorouting/*.cc networklayer/common/*.cc networklayer/contract/*.cc networklayer/extras/*.cc networklayer/icmpv6/*.cc networklayer/ipv4/*.cc networklayer/ipv6/*.cc networklayer/ldp/*.cc networklayer/manetrouting/*.cc networklayer/manetrouting/aodv/*.cc networklayer/manetrouting/aodv/aodv-uu/*.cc networklayer/manetrouting/base/*.cc networklayer/manetrouting/batman/*.cc networklayer/manetrouting/dsdv/*.cc networklayer/manetrouting/dsr/*.cc networklayer/manetrouting/dsr/dsr-uu/*.cc networklayer/manetrouting/dymo/*.cc networklayer/manetrouting/dymo/dymoum/*.cc networklayer/manetrouting/dymo_fau/*.cc networklayer/manetrouting/manager/*.cc networklayer/manetrouting/olsr/*.cc networklayer/mpls/*.cc networklayer/ospfv2/*.cc networklayer/ospfv2/interface/*.cc networklayer/ospfv2/messagehandler/*.cc networklayer/ospfv2/neighbor/*.cc networklayer/ospfv2/router/*.cc networklayer/queue/*.cc networklayer/rsvp_te/*.cc networklayer/ted/*.cc nodes/*.cc nodes/adhoc/*.cc nodes/ethernet/*.cc nodes/inet/*.cc nodes/ipv6/*.cc nodes/mpls/*.cc nodes/wireless/*.cc transport/*.cc transport/contract/*.cc transport/rtp/*.cc transport/rtp/profiles/*.cc transport/rtp/profiles/avprofile/*.cc transport/sctp/*.cc transport/tcp/*.cc transport/tcp/flavours/*.cc transport/tcp/queues/*.cc transport/tcp_nsc/*.cc transport/tcp_nsc/queues/*.cc transport/tcp_old/*.cc transport/tcp_old/flavours/*.cc transport/tcp_old/queues/*.cc transport/udp/*.cc util/*.cc util/headerserializers/*.cc util/headerserializers/headers/*.cc world/*.cc world/annotations/*.cc world/obstacles/*.cc world/shadowing/*.cc world/traci/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/applications/dhcp/DHCPClient.o: applications/dhcp/DHCPClient.cc \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/arp/ARP.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/common/InterfaceTable.h \
  networklayer/contract/IPAddressResolver.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  transport/contract/UDPControlInfo_m.h \
  applications/dhcp/DHCPLease.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  applications/udpapp/UDPAppBase.h \
  base/NotificationBoard.h \
  applications/dhcp/DHCP_m.h \
  applications/dhcp/DHCPOptions.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  applications/dhcp/Byte.h \
  networklayer/ipv4/IRoutingTable.h \
  applications/dhcp/DHCPClient.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/arp/ARPPacket_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h
$O/applications/dhcp/DHCPServer.o: applications/dhcp/DHCPServer.cc \
  applications/udpapp/UDPAppBase.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  applications/dhcp/DHCPLease.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  applications/dhcp/DHCPOptions.h \
  applications/dhcp/DHCP_m.h \
  base/NotificationBoard.h \
  networklayer/contract/IPv6Address.h \
  networklayer/arp/ARP.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/common/InterfaceTable.h \
  applications/dhcp/DHCPServer.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/arp/ARPPacket_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  networklayer/ipv4/IRoutingTable.h \
  applications/dhcp/Byte.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h
$O/applications/dhcp/DHCP_m.o: applications/dhcp/DHCP_m.cc \
  linklayer/contract/MACAddress.h \
  applications/dhcp/DHCP_m.h \
  applications/dhcp/DHCPOptions.h \
  base/INETDefs.h \
  applications/dhcp/Byte.h \
  networklayer/contract/IPAddress.h
$O/applications/ethernet/EtherAppCli.o: applications/ethernet/EtherAppCli.cc \
  base/INETDefs.h \
  applications/ethernet/EtherApp_m.h \
  applications/ethernet/EtherAppCli.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  linklayer/contract/MACAddress.h
$O/applications/ethernet/EtherAppSrv.o: applications/ethernet/EtherAppSrv.cc \
  applications/ethernet/EtherApp_m.h \
  base/INETDefs.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  applications/ethernet/EtherAppSrv.h \
  linklayer/contract/MACAddress.h
$O/applications/ethernet/EtherApp_m.o: applications/ethernet/EtherApp_m.cc \
  base/INETDefs.h \
  applications/ethernet/EtherApp_m.h
$O/applications/generic/IPTrafGen.o: applications/generic/IPTrafGen.cc \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  networklayer/contract/IPAddress.h \
  linklayer/contract/MACAddress.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  applications/generic/IPTrafGen.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/contract/IPControlInfo_m.h
$O/applications/pingapp/PingApp.o: applications/pingapp/PingApp.cc \
  linklayer/contract/MACAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/contract/IPProtocolId_m.h \
  applications/pingapp/PingApp.h \
  networklayer/contract/IPControlInfo.h \
  applications/pingapp/PingPayload_m.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/applications/pingapp/PingPayload_m.o: applications/pingapp/PingPayload_m.cc \
  applications/pingapp/PingPayload_m.h \
  base/INETDefs.h
$O/applications/rtpapp/RTPApplication.o: applications/rtpapp/RTPApplication.cc \
  applications/rtpapp/RTPApplication.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  base/INETDefs.h \
  transport/rtp/RTPInterfacePacket.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/RTPSenderStatusMessage_m.h
$O/applications/sctpapp/SCTPClient.o: applications/sctpapp/SCTPClient.cc \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  transport/sctp/SCTPMessage.h \
  transport/contract/SCTPCommand.h \
  networklayer/contract/IPControlInfo.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  util/common.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/common/InterfaceTable.h \
  base/ModuleAccess.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/contract/IPControlInfo_m.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h \
  transport/contract/SCTPSocket.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  networklayer/rsvp_te/IntServ_m.h \
  applications/sctpapp/SCTPClient.h \
  transport/sctp/SCTP.h \
  transport/contract/SCTPCommand_m.h
$O/applications/sctpapp/SCTPPeer.o: applications/sctpapp/SCTPPeer.cc \
  linklayer/contract/MACAddress.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/SCTPSocket.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  transport/contract/SCTPCommand_m.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/contract/IPAddressResolver.h \
  base/ModuleAccess.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/common/InterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/contract/IPControlInfo.h \
  transport/sctp/SCTPMessage.h \
  transport/contract/SCTPCommand.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  util/common.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  networklayer/common/InterfaceEntry.h \
  applications/sctpapp/SCTPPeer.h \
  networklayer/contract/IPAddress.h
$O/applications/sctpapp/SCTPServer.o: applications/sctpapp/SCTPServer.cc \
  util/common.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  transport/contract/SCTPCommand.h \
  transport/sctp/SCTPMessage.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  applications/sctpapp/SCTPServer.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTP.h \
  transport/contract/SCTPCommand_m.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/SCTPSocket.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h
$O/applications/tcpapp/GenericAppMsg_m.o: applications/tcpapp/GenericAppMsg_m.cc \
  base/INETDefs.h \
  applications/tcpapp/GenericAppMsg_m.h
$O/applications/tcpapp/TCPBasicClientApp.o: applications/tcpapp/TCPBasicClientApp.cc \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/contract/TCPCommand_m.h \
  applications/tcpapp/TCPBasicClientApp.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  applications/tcpapp/TCPGenericCliAppBase.h
$O/applications/tcpapp/TCPEchoApp.o: applications/tcpapp/TCPEchoApp.cc \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPAddress.h \
  applications/tcpapp/TCPEchoApp.h \
  base/INETDefs.h \
  networklayer/contract/IPvXAddress.h \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPv6Address.h
$O/applications/tcpapp/TCPGenericCliAppBase.o: applications/tcpapp/TCPGenericCliAppBase.cc \
  networklayer/contract/IPv6Address.h \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPAddressResolver.h \
  applications/tcpapp/GenericAppMsg_m.h \
  applications/tcpapp/TCPGenericCliAppBase.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/contract/TCPCommand_m.h
$O/applications/tcpapp/TCPGenericSrvApp.o: applications/tcpapp/TCPGenericSrvApp.cc \
  applications/tcpapp/GenericAppMsg_m.h \
  networklayer/contract/IPvXAddress.h \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPv6Address.h \
  transport/contract/TCPCommand_m.h \
  base/INETDefs.h \
  applications/tcpapp/TCPGenericSrvApp.h \
  networklayer/contract/IPAddress.h
$O/applications/tcpapp/TCPGenericSrvThread.o: applications/tcpapp/TCPGenericSrvThread.cc \
  base/INETDefs.h \
  applications/tcpapp/TCPGenericSrvThread.h \
  networklayer/contract/IPAddress.h \
  transport/contract/TCPSocketMap.h \
  transport/contract/TCPCommand_m.h \
  applications/tcpapp/TCPSrvHostApp.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/contract/TCPSocket.h \
  applications/tcpapp/GenericAppMsg_m.h
$O/applications/tcpapp/TCPSessionApp.o: applications/tcpapp/TCPSessionApp.cc \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPAddress.h \
  applications/tcpapp/TCPSessionApp.h \
  base/INETDefs.h \
  networklayer/contract/IPAddressResolver.h \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/applications/tcpapp/TCPSinkApp.o: applications/tcpapp/TCPSinkApp.cc \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/contract/TCPSocket.h \
  applications/tcpapp/TCPSinkApp.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/contract/TCPCommand_m.h
$O/applications/tcpapp/TCPSpoof.o: applications/tcpapp/TCPSpoof.cc \
  networklayer/contract/IPAddress.h \
  applications/tcpapp/TCPSpoof.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddressResolver.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  linklayer/contract/MACAddress.h
$O/applications/tcpapp/TCPSrvHostApp.o: applications/tcpapp/TCPSrvHostApp.cc \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  applications/tcpapp/TCPSrvHostApp.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/contract/TCPSocketMap.h \
  transport/contract/TCPCommand_m.h
$O/applications/tcpapp/TelnetApp.o: applications/tcpapp/TelnetApp.cc \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPAddress.h \
  applications/tcpapp/TCPGenericCliAppBase.h \
  base/INETDefs.h \
  applications/tcpapp/TelnetApp.h \
  networklayer/contract/IPvXAddress.h \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPv6Address.h
$O/applications/traci/TraCIDemo.o: applications/traci/TraCIDemo.cc \
  base/INotifiable.h \
  ./mobility/traci/TraCIMobility.h \
  base/Coord.h \
  ./applications/traci/TraCIDemo.h \
  base/BasicModule.h \
  ./world/traci/TraCIScenarioManager.h \
  transport/contract/UDPSocket.h \
  util/FWMath.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  transport/contract/UDPControlInfo_m.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/applications/traci/TraCITestApp.o: applications/traci/TraCITestApp.cc \
  base/BasicModule.h \
  ./world/traci/TraCIScenarioManager.h \
  util/FWMath.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/INotifiable.h \
  ./mobility/traci/TraCIMobility.h \
  base/Coord.h \
  base/ModuleAccess.h \
  ./applications/traci/TraCITestApp.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h
$O/applications/trafgen/TrafGen.o: applications/trafgen/TrafGen.cc \
  ./applications/trafgen/TrafGen.h
$O/applications/trafgen/TrafGenPar.o: applications/trafgen/TrafGenPar.cc \
  applications/trafgen/TrafGenPar.h
$O/applications/udpapp/UDPAppBase.o: applications/udpapp/UDPAppBase.cc \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/contract/UDPControlInfo_m.h \
  applications/udpapp/UDPAppBase.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/contract/UDPSocket.h
$O/applications/udpapp/UDPBasicApp.o: applications/udpapp/UDPBasicApp.cc \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPAddressResolver.h \
  transport/contract/UDPControlInfo_m.h \
  applications/udpapp/UDPAppBase.h \
  applications/udpapp/UDPBasicApp.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/applications/udpapp/UDPBasicBurst.o: applications/udpapp/UDPBasicBurst.cc \
  applications/udpapp/UDPBasicBurst.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPAddressResolver.h \
  base/ModuleAccess.h \
  applications/udpapp/UDPAppBase.h \
  transport/contract/UDPControlInfo_m.h \
  base/INETDefs.h \
  base/MMapBoard.h \
  networklayer/contract/IPAddress.h
$O/applications/udpapp/UDPBasicBurst2.o: applications/udpapp/UDPBasicBurst2.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  applications/udpapp/UDPAppBase.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  applications/udpapp/UDPBasicBurst2.h \
  networklayer/contract/IPAddressResolver.h
$O/applications/udpapp/UDPEchoApp.o: applications/udpapp/UDPEchoApp.cc \
  applications/udpapp/UDPEchoAppMsg_m.h \
  transport/contract/UDPControlInfo_m.h \
  applications/udpapp/UDPAppBase.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  base/INETDefs.h \
  applications/udpapp/UDPBasicApp.h \
  networklayer/contract/IPAddress.h \
  applications/udpapp/UDPEchoApp.h
$O/applications/udpapp/UDPEchoAppMsg_m.o: applications/udpapp/UDPEchoAppMsg_m.cc \
  applications/udpapp/UDPEchoAppMsg_m.h \
  base/INETDefs.h
$O/applications/udpapp/UDPSink.o: applications/udpapp/UDPSink.cc \
  networklayer/contract/IPAddressResolver.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/contract/UDPControlInfo_m.h \
  applications/udpapp/UDPSink.h \
  applications/udpapp/UDPAppBase.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h
$O/applications/udpapp/UDPVideoStreamCli.o: applications/udpapp/UDPVideoStreamCli.cc \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  applications/udpapp/UDPVideoStreamCli.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  applications/udpapp/UDPAppBase.h
$O/applications/udpapp/UDPVideoStreamSvr.o: applications/udpapp/UDPVideoStreamSvr.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  applications/udpapp/UDPAppBase.h \
  applications/udpapp/UDPVideoStreamSvr.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/base/AbstractQueue.o: base/AbstractQueue.cc \
  base/AbstractQueue.h \
  base/INETDefs.h
$O/base/BasicModule.o: base/BasicModule.cc \
  base/INETDefs.h \
  base/BasicModule.h \
  base/INotifiable.h \
  base/ModuleAccess.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h
$O/base/Blackboard.o: base/Blackboard.cc \
  base/Blackboard.h \
  base/ModuleAccess.h \
  base/INETDefs.h
$O/base/ByteArrayMessage.o: base/ByteArrayMessage.cc \
  base/INETDefs.h \
  base/ByteArrayMessage.h \
  base/ByteArrayMessage_m.h
$O/base/ByteArrayMessage_m.o: base/ByteArrayMessage_m.cc \
  base/INETDefs.h \
  base/ByteArrayMessage_m.h
$O/base/Join.o: base/Join.cc \
  base/INETDefs.h
$O/base/MMapBoard.o: base/MMapBoard.cc \
  base/ModuleAccess.h \
  base/INETDefs.h \
  base/MMapBoard.h
$O/base/ModuleAccess.o: base/ModuleAccess.cc \
  base/INETDefs.h \
  base/ModuleAccess.h
$O/base/NotificationBoard.o: base/NotificationBoard.cc \
  base/INotifiable.h \
  base/INETDefs.h \
  base/NotificationBoard.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h
$O/base/NotifierConsts.o: base/NotifierConsts.cc \
  base/INETDefs.h \
  base/NotifierConsts.h
$O/base/PassiveQueueBase.o: base/PassiveQueueBase.cc \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  base/INETDefs.h
$O/base/ProtocolMap.o: base/ProtocolMap.cc \
  base/ProtocolMap.h \
  base/INETDefs.h
$O/base/QueueBase.o: base/QueueBase.cc \
  base/INETDefs.h \
  base/QueueBase.h \
  base/AbstractQueue.h
$O/base/QueueWithQoS.o: base/QueueWithQoS.cc \
  base/INETDefs.h \
  base/EnqueueHook.h \
  base/AbstractQueue.h \
  base/QueueWithQoS.h
$O/base/ReassemblyBuffer.o: base/ReassemblyBuffer.cc \
  base/ReassemblyBuffer.h \
  base/INETDefs.h
$O/base/Sink.o: base/Sink.cc \
  base/INETDefs.h \
  base/Sink.h
$O/battery/BasicBattery.o: battery/BasicBattery.cc \
  base/NotificationBoard.h \
  battery/BasicBattery.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h \
  base/INotifiable.h \
  base/INETDefs.h
$O/battery/InetSimpleBattery.o: battery/InetSimpleBattery.cc \
  base/INotifiable.h \
  battery/Energy.h \
  base/INETDefs.h \
  battery/BasicBattery.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  base/ModuleAccess.h \
  battery/InetSimpleBattery.h \
  linklayer/contract/RadioState.h
$O/experimental/linklayer/ieee80211/hwmp/hwmp-rtable.o: experimental/linklayer/ieee80211/hwmp/hwmp-rtable.cc \
  base/INETDefs.h \
  experimental/linklayer/ieee80211/hwmp/hwmp-rtable.h \
  linklayer/contract/MACAddress.h
$O/experimental/linklayer/ieee80211/hwmp/hwmp.o: experimental/linklayer/ieee80211/hwmp/hwmp.cc \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  experimental/linklayer/ieee80211/mgmt/ETXPacket_m.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  experimental/linklayer/ieee80211/hwmp/hwmp-rtable.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  experimental/linklayer/ieee80211/hwmp/hwmp.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/contract/Radio80211aControlInfo_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h
$O/experimental/linklayer/ieee80211/mgmt/ETXPacket_m.o: experimental/linklayer/ieee80211/mgmt/ETXPacket_m.cc \
  base/INETDefs.h \
  experimental/linklayer/ieee80211/mgmt/ETXPacket_m.h \
  linklayer/contract/MACAddress.h
$O/experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.o: experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.cc \
  networklayer/common/IInterfaceTable.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/contract/Radio80211aControlInfo_m.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  experimental/linklayer/ieee80211/mgmt/ETXPacket_m.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.h \
  base/ModuleAccess.h
$O/experimental/linklayer/ieee80211/mgmt/Ieee80211Mesh.o: experimental/linklayer/ieee80211/mgmt/Ieee80211Mesh.cc \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211Mesh.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  networklayer/ipv6/IPv6Datagram.h \
  base/NotificationBoard.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/mpls/MPLSPacket.h \
  experimental/linklayer/ieee80211/mgmt/LWMPLSPacket_m.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/ted/LinkStatePacket_m.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/aodv/aodv_msg_struct.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  linklayer/contract/ControlInfoBreakLink_m.h \
  experimental/linklayer/ieee80211/mgmt/ETXPacket_m.h \
  experimental/linklayer/ieee80211/mgmt/lwmpls_data.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/contract/MeshControlInfo_m.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/arp/ARPPacket_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dymo/dymo_msg_struct.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ted/TED_m.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/queue/IQoSClassifier.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  networklayer/ipv4/ICMPMessage_m.h
$O/experimental/linklayer/ieee80211/mgmt/Ieee80211MeshLwmpls.o: experimental/linklayer/ieee80211/mgmt/Ieee80211MeshLwmpls.cc \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ted/TED_m.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/queue/IQoSClassifier.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/arp/ARPPacket_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dymo/dymo_msg_struct.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  networklayer/mpls/MPLSPacket.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  experimental/linklayer/ieee80211/mgmt/LWMPLSPacket_m.h \
  networklayer/ted/LinkStatePacket_m.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/aodv/aodv_msg_struct.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  base/PassiveQueueBase.h \
  base/IPassiveQueue.h \
  linklayer/contract/ControlInfoBreakLink_m.h \
  experimental/linklayer/ieee80211/mgmt/ETXPacket_m.h \
  linklayer/contract/MeshControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  experimental/linklayer/ieee80211/mgmt/lwmpls_data.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211Mesh.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  networklayer/ipv6/IPv6Datagram.h \
  base/NotificationBoard.h
$O/experimental/linklayer/ieee80211/mgmt/Ieee80211MgmtAdhocWithEtx.o: experimental/linklayer/ieee80211/mgmt/Ieee80211MgmtAdhocWithEtx.cc \
  networklayer/queue/IQoSClassifier.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtAdhoc.h \
  base/INETDefs.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211MgmtAdhocWithEtx.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  base/INotifiable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h \
  experimental/linklayer/ieee80211/mgmt/ETXPacket_m.h \
  base/PassiveQueueBase.h \
  base/IPassiveQueue.h \
  experimental/linklayer/ieee80211/mgmt/Ieee80211Etx.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  base/NotificationBoard.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h
$O/experimental/linklayer/ieee80211/mgmt/LWMPLSPacket_m.o: experimental/linklayer/ieee80211/mgmt/LWMPLSPacket_m.cc \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  experimental/linklayer/ieee80211/mgmt/LWMPLSPacket_m.h
$O/experimental/linklayer/ieee80211/mgmt/lwmpls_data.o: experimental/linklayer/ieee80211/mgmt/lwmpls_data.cc \
  experimental/linklayer/ieee80211/mgmt/lwmpls_data.h
$O/linklayer/contract/ControlInfoBreakLink_m.o: linklayer/contract/ControlInfoBreakLink_m.cc \
  base/INETDefs.h \
  linklayer/contract/ControlInfoBreakLink_m.h \
  linklayer/contract/MACAddress.h
$O/linklayer/contract/Ieee802Ctrl_m.o: linklayer/contract/Ieee802Ctrl_m.cc \
  linklayer/contract/MACAddress.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  base/INETDefs.h
$O/linklayer/contract/MACAddress.o: linklayer/contract/MACAddress.cc \
  networklayer/common/InterfaceToken.h \
  linklayer/contract/MACAddress.h \
  base/INETDefs.h
$O/linklayer/contract/MeshControlInfo_m.o: linklayer/contract/MeshControlInfo_m.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/base/uint128.h \
  linklayer/contract/MeshControlInfo_m.h \
  linklayer/contract/MACAddress.h
$O/linklayer/contract/PhyControlInfo_m.o: linklayer/contract/PhyControlInfo_m.cc \
  base/INETDefs.h \
  linklayer/contract/PhyControlInfo_m.h
$O/linklayer/contract/Radio80211aControlInfo_m.o: linklayer/contract/Radio80211aControlInfo_m.cc \
  linklayer/mfcore/ModulationType.h \
  linklayer/contract/Radio80211aControlInfo_m.h
$O/linklayer/ethernet/EtherBus.o: linklayer/ethernet/EtherBus.cc \
  linklayer/contract/MACAddress.h \
  linklayer/ethernet/EtherFrame_m.h \
  linklayer/ethernet/Ethernet.h \
  linklayer/ethernet/EtherBus.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  base/INETDefs.h
$O/linklayer/ethernet/EtherEncap.o: linklayer/ethernet/EtherEncap.cc \
  linklayer/contract/TxNotifDetails.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  linklayer/ethernet/EtherMAC.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  linklayer/ethernet/EtherMACBase.h \
  base/ModuleAccess.h \
  util/PowerControlMessages_m.h \
  util/PowerControlManager.h \
  linklayer/ethernet/Ethernet.h \
  networklayer/common/InterfaceToken.h \
  base/NotificationBoard.h \
  linklayer/ethernet/EtherFrame_m.h \
  base/IPowerControl.h \
  base/NotifierConsts.h \
  linklayer/ethernet/EtherEncap.h \
  linklayer/contract/MACAddress.h
$O/linklayer/ethernet/EtherFrame_m.o: linklayer/ethernet/EtherFrame_m.cc \
  base/INETDefs.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  linklayer/ethernet/Ethernet.h \
  linklayer/contract/MACAddress.h \
  linklayer/ethernet/EtherFrame_m.h
$O/linklayer/ethernet/EtherHub.o: linklayer/ethernet/EtherHub.cc \
  linklayer/ethernet/Ethernet.h \
  linklayer/ethernet/EtherFrame_m.h \
  linklayer/ethernet/EtherHub.h \
  linklayer/contract/MACAddress.h \
  base/INETDefs.h \
  linklayer/contract/Ieee802Ctrl_m.h
$O/linklayer/ethernet/EtherLLC.o: linklayer/ethernet/EtherLLC.cc \
  linklayer/contract/Ieee802Ctrl_m.h \
  linklayer/ethernet/EtherLLC.h \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  linklayer/ethernet/EtherFrame_m.h \
  linklayer/ethernet/Ethernet.h
$O/linklayer/ethernet/EtherMAC.o: linklayer/ethernet/EtherMAC.cc \
  linklayer/contract/Ieee802Ctrl_m.h \
  base/INotifiable.h \
  linklayer/ethernet/EtherMAC.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/contract/TxNotifDetails.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  base/IPowerControl.h \
  linklayer/ethernet/EtherFrame_m.h \
  base/NotificationBoard.h \
  networklayer/common/InterfaceToken.h \
  linklayer/ethernet/Ethernet.h \
  util/PowerControlMessages_m.h \
  util/PowerControlManager.h \
  base/ModuleAccess.h \
  base/IPassiveQueue.h \
  linklayer/ethernet/EtherMACBase.h
$O/linklayer/ethernet/EtherMAC2.o: linklayer/ethernet/EtherMAC2.cc \
  base/IPowerControl.h \
  linklayer/ethernet/EtherFrame_m.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  util/PowerControlMessages_m.h \
  util/PowerControlManager.h \
  base/ModuleAccess.h \
  linklayer/ethernet/EtherMACBase.h \
  base/IPassiveQueue.h \
  networklayer/common/InterfaceToken.h \
  linklayer/ethernet/Ethernet.h \
  linklayer/ethernet/EtherMAC2.h \
  base/INotifiable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/contract/TxNotifDetails.h \
  base/INETDefs.h
$O/linklayer/ethernet/EtherMACBase.o: linklayer/ethernet/EtherMACBase.cc \
  linklayer/ethernet/EtherMACBase.h \
  base/IPassiveQueue.h \
  base/ModuleAccess.h \
  util/PowerControlManager.h \
  util/PowerControlMessages_m.h \
  linklayer/ethernet/Ethernet.h \
  networklayer/common/InterfaceToken.h \
  base/NotificationBoard.h \
  base/IPowerControl.h \
  linklayer/ethernet/EtherFrame_m.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  linklayer/contract/TxNotifDetails.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/Ieee802Ctrl_m.h
$O/linklayer/etherswitch/MACRelayUnitBase.o: linklayer/etherswitch/MACRelayUnitBase.cc \
  linklayer/contract/Ieee802Ctrl_m.h \
  base/INETDefs.h \
  linklayer/ethernet/EtherFrame_m.h \
  linklayer/contract/MACAddress.h \
  linklayer/etherswitch/MACRelayUnitBase.h \
  linklayer/ethernet/Ethernet.h
$O/linklayer/etherswitch/MACRelayUnitNP.o: linklayer/etherswitch/MACRelayUnitNP.cc \
  base/INETDefs.h \
  linklayer/etherswitch/MACRelayUnitNP.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  linklayer/ethernet/Ethernet.h \
  linklayer/etherswitch/MACRelayUnitBase.h \
  linklayer/ethernet/EtherFrame_m.h \
  linklayer/contract/MACAddress.h
$O/linklayer/etherswitch/MACRelayUnitPP.o: linklayer/etherswitch/MACRelayUnitPP.cc \
  linklayer/contract/MACAddress.h \
  linklayer/ethernet/EtherFrame_m.h \
  linklayer/ethernet/Ethernet.h \
  linklayer/etherswitch/MACRelayUnitBase.h \
  linklayer/etherswitch/MACRelayUnitPP.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  base/INETDefs.h
$O/linklayer/ext/ExtFrame_m.o: linklayer/ext/ExtFrame_m.cc \
  linklayer/ext/ExtFrame_m.h
$O/linklayer/ext/ExtInterface.o: linklayer/ext/ExtInterface.cc \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  linklayer/ext/ExtInterface.h \
  base/NotificationBoard.h \
  networklayer/common/InterfaceToken.h \
  util/headerserializers/IPSerializer.h \
  base/ModuleAccess.h \
  linklayer/ext/ExtFrame_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  util/headerserializers/headers/bsdint.h \
  networklayer/ipv4/IPDatagram_m.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INETDefs.h \
  linklayer/ext/cSocketRTScheduler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h
$O/linklayer/ext/cSocketRTScheduler.o: linklayer/ext/cSocketRTScheduler.cc \
  base/INETDefs.h \
  linklayer/ext/cSocketRTScheduler.h \
  util/headerserializers/headers/bsdint.h \
  linklayer/ext/ExtFrame_m.h \
  util/headerserializers/headers/ethernet.h
$O/linklayer/ieee80211/mac/BlockAckCache.o: linklayer/ieee80211/mac/BlockAckCache.cc \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  base/INETDefs.h \
  linklayer/ieee80211/mac/BlockAckFrame.h \
  linklayer/ieee80211/mac/BlockAckCache.h \
  linklayer/contract/MACAddress.h
$O/linklayer/ieee80211/mac/BlockAckFrame.o: linklayer/ieee80211/mac/BlockAckFrame.cc \
  linklayer/contract/MACAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  base/INETDefs.h \
  linklayer/ieee80211/mac/BlockAckFrame.h
$O/linklayer/ieee80211/mac/FrameBlock.o: linklayer/ieee80211/mac/FrameBlock.cc \
  linklayer/ieee80211/mac/FrameBlock.h
$O/linklayer/ieee80211/mac/Ieee80211Frame_m.o: linklayer/ieee80211/mac/Ieee80211Frame_m.cc \
  linklayer/contract/MACAddress.h \
  base/INETDefs.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h
$O/linklayer/ieee80211/mac/Ieee80211Mac.o: linklayer/ieee80211/mac/Ieee80211Mac.cc \
  linklayer/contract/MACAddress.h \
  linklayer/mfcore/WirelessMacBase.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  linklayer/ieee80211/mac/Ieee80211Mac.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  base/IPassiveQueue.h \
  linklayer/contract/RadioState.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/PhyControlInfo_m.h \
  base/INotifiable.h \
  base/FSMA.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h
$O/linklayer/ieee80211/mac/Ieee80211NewMac.o: linklayer/ieee80211/mac/Ieee80211NewMac.cc \
  base/Coord.h \
  base/INotifiable.h \
  base/FSMA.h \
  networklayer/common/InterfaceTableAccess.h \
  linklayer/contract/PhyControlInfo_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  util/FWMath.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/queue/IQoSClassifier.h \
  linklayer/radio/errormodel/WifiPreambleType.h \
  linklayer/contract/Radio80211aControlInfo_m.h \
  networklayer/common/InterfaceEntry.h \
  transport/udp/UDPPacket.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  linklayer/mfcore/ModulationType.h \
  base/NotificationBoard.h \
  linklayer/ieee80211/mac/Ieee80211NewMac.h \
  linklayer/mfcore/WirelessMacBase.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/contract/MACAddress.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  networklayer/ipv4/IPDatagram.h \
  base/IPassiveQueue.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  transport/tcp/TCPSegment.h \
  linklayer/radio/errormodel/WifiMode.h \
  transport/udp/UDPPacket_m.h \
  networklayer/common/InterfaceToken.h \
  linklayer/ieee80211/mgmt/Ieee80211eClassifier.h \
  networklayer/ipv4/ICMPMessage.h
$O/linklayer/ieee80211/mac/Ieee80211aMac.o: linklayer/ieee80211/mac/Ieee80211aMac.cc \
  base/ModuleAccess.h \
  base/IPassiveQueue.h \
  linklayer/contract/RadioState.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  networklayer/common/InterfaceToken.h \
  linklayer/ieee80211/mac/Ieee80211aConsts.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/WirelessMacBase.h \
  base/NotifierConsts.h \
  linklayer/contract/Radio80211aControlInfo_m.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  util/FWMath.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  networklayer/common/InterfaceTableAccess.h \
  base/FSMA.h \
  base/INotifiable.h \
  linklayer/ieee80211/mac/Ieee80211aMac.h \
  base/Coord.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/PhyControlInfo_m.h
$O/linklayer/ieee80211/mac/Ieee80211eMac.o: linklayer/ieee80211/mac/Ieee80211eMac.cc \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  linklayer/mfcore/WirelessMacBase.h \
  linklayer/ieee80211/mac/Ieee80211eMac.h \
  base/NotificationBoard.h \
  networklayer/ipv4/ICMPMessage.h \
  transport/udp/UDPPacket_m.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  transport/tcp/TCPSegment.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/contract/RadioState.h \
  base/IPassiveQueue.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/PhyControlInfo_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  base/FSMA.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  transport/udp/UDPPacket.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/contract/IPAddress.h
$O/linklayer/ieee80211/mac/Ieee80211egMac.o: linklayer/ieee80211/mac/Ieee80211egMac.cc \
  base/NotifierConsts.h \
  linklayer/mfcore/WirelessMacBase.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  linklayer/ieee80211/mac/Ieee80211egMac.h \
  networklayer/common/InterfaceToken.h \
  transport/udp/UDPPacket_m.h \
  networklayer/ipv4/ICMPMessage.h \
  linklayer/contract/RadioState.h \
  base/IPassiveQueue.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  transport/tcp/TCPSegment.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPDatagram_m.h \
  linklayer/contract/PhyControlInfo_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  base/FSMA.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  transport/udp/UDPPacket.h \
  networklayer/common/InterfaceEntry.h
$O/linklayer/ieee80211/mac/Ieee80211gMac.o: linklayer/ieee80211/mac/Ieee80211gMac.cc \
  base/Coord.h \
  base/FSMA.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  linklayer/contract/PhyControlInfo_m.h \
  linklayer/ieee80211/mac/Ieee80211gMac.h \
  networklayer/common/IInterfaceTable.h \
  util/FWMath.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  linklayer/contract/Radio80211aControlInfo_m.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  base/NotificationBoard.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  linklayer/mfcore/WirelessMacBase.h \
  linklayer/contract/MACAddress.h \
  linklayer/contract/RadioState.h \
  base/IPassiveQueue.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h
$O/linklayer/ieee80211/mgmt/Ieee80211AgentSTA.o: linklayer/ieee80211/mgmt/Ieee80211AgentSTA.cc \
  base/NotificationBoard.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  linklayer/ieee80211/mgmt/Ieee80211Primitives_m.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/ieee80211/mgmt/Ieee80211AgentSTA.h \
  base/INETDefs.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtAP.o: linklayer/ieee80211/mgmt/Ieee80211MgmtAP.cc \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  base/ProtocolMap.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/queue/IQoSClassifier.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IP.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/IPFragBuf.h \
  base/INETDefs.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtAPBase.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  base/QueueBase.h \
  linklayer/ethernet/EtherFrame_m.h \
  base/NotificationBoard.h \
  base/ReassemblyBuffer.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  base/PassiveQueueBase.h \
  base/IPassiveQueue.h \
  linklayer/contract/RadioState.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPv6Address.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtAP.h \
  base/AbstractQueue.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  linklayer/ethernet/Ethernet.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtAPBase.o: linklayer/ieee80211/mgmt/Ieee80211MgmtAPBase.cc \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/queue/IQoSClassifier.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  networklayer/common/InterfaceToken.h \
  linklayer/ethernet/Ethernet.h \
  base/ModuleAccess.h \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtAPBase.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  linklayer/ethernet/EtherFrame_m.h \
  base/NotificationBoard.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtAPSimplified.o: linklayer/ieee80211/mgmt/Ieee80211MgmtAPSimplified.cc \
  base/INETDefs.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtAPSimplified.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/queue/IQoSClassifier.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  linklayer/ethernet/Ethernet.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  base/PassiveQueueBase.h \
  base/IPassiveQueue.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  linklayer/ethernet/EtherFrame_m.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtAPBase.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  base/NotificationBoard.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtAdhoc.o: linklayer/ieee80211/mgmt/Ieee80211MgmtAdhoc.cc \
  networklayer/queue/IQoSClassifier.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtAdhoc.h \
  base/INETDefs.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  base/NotificationBoard.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtBase.o: linklayer/ieee80211/mgmt/Ieee80211MgmtBase.cc \
  base/ModuleAccess.h \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  networklayer/common/InterfaceToken.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/queue/IQoSClassifier.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  base/INETDefs.h \
  networklayer/common/InterfaceTableAccess.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  base/INotifiable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.o: linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.cc \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  base/INETDefs.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  linklayer/contract/MACAddress.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtSTA.o: linklayer/ieee80211/mgmt/Ieee80211MgmtSTA.cc \
  base/Coord.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  linklayer/contract/PhyControlInfo_m.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  linklayer/ieee80211/mgmt/Ieee80211Primitives_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  util/FWMath.h \
  networklayer/queue/IQoSClassifier.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/NotificationBoard.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtSTA.h \
  world/ChannelControl.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/contract/MACAddress.h \
  base/PassiveQueueBase.h \
  base/IPassiveQueue.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h
$O/linklayer/ieee80211/mgmt/Ieee80211MgmtSTASimplified.o: linklayer/ieee80211/mgmt/Ieee80211MgmtSTASimplified.cc \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h \
  base/INotifiable.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/queue/IQoSClassifier.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  linklayer/contract/MACAddress.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtSTASimplified.h \
  base/NotifierConsts.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtBase.h \
  base/NotificationBoard.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h
$O/linklayer/ieee80211/mgmt/Ieee80211Primitives_m.o: linklayer/ieee80211/mgmt/Ieee80211Primitives_m.cc \
  linklayer/contract/MACAddress.h \
  linklayer/ieee80211/mgmt/Ieee80211MgmtFrames_m.h \
  linklayer/ieee80211/mgmt/Ieee80211Primitives_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  base/INETDefs.h
$O/linklayer/ieee80211/mgmt/Ieee80211eClassifier.o: linklayer/ieee80211/mgmt/Ieee80211eClassifier.cc \
  networklayer/ipv4/IPDatagram.h \
  transport/tcp/TCPSegment.h \
  transport/udp/UDPPacket_m.h \
  linklayer/ieee80211/mgmt/Ieee80211eClassifier.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/ipv6/IPv6Datagram.h \
  networklayer/icmpv6/IPv6NDMessage_m.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/queue/IQoSClassifier.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPProtocolId_m.h
$O/linklayer/mf80211/macLayer/CSMAMacLayer.o: linklayer/mf80211/macLayer/CSMAMacLayer.cc \
  base/NotificationBoard.h \
  linklayer/mf80211/macLayer/CSMAMacLayer.h \
  linklayer/mf80211/macLayer/MacPkt_m.h \
  linklayer/contract/MACAddress.h \
  linklayer/mfcore/WirelessMacBase.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h \
  linklayer/contract/RadioState.h \
  networklayer/common/InterfaceToken.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h
$O/linklayer/mf80211/macLayer/Mac80211.o: linklayer/mf80211/macLayer/Mac80211.cc \
  networklayer/common/InterfaceToken.h \
  linklayer/mf80211/macLayer/Consts80211.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  linklayer/mfcore/WirelessMacBase.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  linklayer/mf80211/macLayer/MacPkt_m.h \
  base/NotificationBoard.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/mf80211/macLayer/Mac80211.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/mf80211/macLayer/Mac80211Pkt_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h
$O/linklayer/mf80211/macLayer/Mac80211Pkt_m.o: linklayer/mf80211/macLayer/Mac80211Pkt_m.cc \
  linklayer/mf80211/macLayer/Mac80211Pkt_m.h \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  linklayer/mf80211/macLayer/MacPkt_m.h
$O/linklayer/mf80211/macLayer/MacPkt_m.o: linklayer/mf80211/macLayer/MacPkt_m.cc \
  linklayer/mf80211/macLayer/MacPkt_m.h \
  linklayer/contract/MACAddress.h \
  base/INETDefs.h
$O/linklayer/mf80211/phyLayer/decider/Decider80211.o: linklayer/mf80211/phyLayer/decider/Decider80211.cc \
  base/INotifiable.h \
  base/Coord.h \
  base/BasicModule.h \
  util/FWMath.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  linklayer/mf80211/phyLayer/decider/Decider80211.h \
  base/NotificationBoard.h \
  linklayer/mfcore/SnrList.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  linklayer/mfcore/BasicDecider.h \
  base/ModuleAccess.h \
  linklayer/mf80211/macLayer/Consts80211.h \
  linklayer/mfcore/SnrControlInfo_m.h
$O/linklayer/mf80211/phyLayer/decider/ErrAndCollDecider.o: linklayer/mf80211/phyLayer/decider/ErrAndCollDecider.cc \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/Coord.h \
  base/INotifiable.h \
  linklayer/mf80211/phyLayer/decider/ErrAndCollDecider.h \
  base/ModuleAccess.h \
  linklayer/mfcore/BasicDecider.h \
  linklayer/mfcore/SnrControlInfo_m.h \
  base/NotificationBoard.h \
  linklayer/mf80211/phyLayer/decider/SnrDecider.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h
$O/linklayer/mf80211/phyLayer/decider/SnrDecider.o: linklayer/mf80211/phyLayer/decider/SnrDecider.cc \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/Coord.h \
  base/INotifiable.h \
  base/ModuleAccess.h \
  linklayer/mfcore/BasicDecider.h \
  linklayer/mfcore/SnrControlInfo_m.h \
  linklayer/mf80211/phyLayer/decider/SnrDecider.h \
  base/NotificationBoard.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h
$O/linklayer/mf80211/phyLayer/snrEval/GilbertElliotSnr.o: linklayer/mf80211/phyLayer/snrEval/GilbertElliotSnr.cc \
  util/FWMath.h \
  base/BasicModule.h \
  world/ChannelAccessExtended.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  world/ChannelControlExtended.h \
  world/ChannelAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  linklayer/mfcore/BasicSnrEval.h \
  linklayer/contract/PhyControlInfo_m.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  linklayer/mf80211/phyLayer/snrEval/SnrEval.h \
  linklayer/mfcore/SnrControlInfo_m.h \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  linklayer/mf80211/phyLayer/snrEval/GilbertElliotSnr.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h
$O/linklayer/mf80211/phyLayer/snrEval/SnrEval.o: linklayer/mf80211/phyLayer/snrEval/SnrEval.cc \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  linklayer/mf80211/phyLayer/snrEval/SnrEval.h \
  linklayer/mfcore/SnrControlInfo_m.h \
  world/ChannelAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  linklayer/mfcore/BasicSnrEval.h \
  linklayer/contract/PhyControlInfo_m.h \
  util/FWMath.h \
  world/ChannelAccessExtended.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  world/ChannelControlExtended.h
$O/linklayer/mf80211/phyLayer/snrEval/SnrEval80211.o: linklayer/mf80211/phyLayer/snrEval/SnrEval80211.cc \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  linklayer/mf80211/phyLayer/snrEval/SnrEval.h \
  linklayer/mf80211/phyLayer/snrEval/SnrEval80211.h \
  linklayer/mfcore/SnrControlInfo_m.h \
  linklayer/mf80211/macLayer/Consts80211.h \
  world/ChannelAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  linklayer/mfcore/BasicSnrEval.h \
  linklayer/contract/PhyControlInfo_m.h \
  util/FWMath.h \
  base/BasicModule.h \
  world/ChannelAccessExtended.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  world/ChannelControlExtended.h
$O/linklayer/mfcore/AirFrame_m.o: linklayer/mfcore/AirFrame_m.cc \
  linklayer/mfcore/AirFrame_m.h \
  base/Coord.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  util/FWMath.h
$O/linklayer/mfcore/BasicDecider.o: linklayer/mfcore/BasicDecider.cc \
  linklayer/mfcore/SnrControlInfo_m.h \
  base/ModuleAccess.h \
  linklayer/mfcore/BasicDecider.h \
  linklayer/mfcore/SnrList.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotificationBoard.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/BasicModule.h \
  util/FWMath.h \
  base/INotifiable.h \
  base/Coord.h
$O/linklayer/mfcore/BasicSnrEval.o: linklayer/mfcore/BasicSnrEval.cc \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  base/ModuleAccess.h \
  linklayer/mfcore/SnrControlInfo_m.h \
  linklayer/mfcore/TransmComplete_m.h \
  world/ChannelAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  linklayer/mfcore/BasicSnrEval.h \
  util/FWMath.h \
  base/BasicModule.h \
  world/ChannelAccessExtended.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  world/ChannelControlExtended.h
$O/linklayer/mfcore/SnrControlInfo_m.o: linklayer/mfcore/SnrControlInfo_m.cc \
  base/INETDefs.h \
  linklayer/mfcore/SnrList.h \
  linklayer/mfcore/SnrControlInfo_m.h
$O/linklayer/mfcore/TransmComplete_m.o: linklayer/mfcore/TransmComplete_m.cc \
  linklayer/mfcore/TransmComplete_m.h \
  base/INETDefs.h
$O/linklayer/mfcore/WirelessMacBase.o: linklayer/mfcore/WirelessMacBase.cc \
  base/ModuleAccess.h \
  linklayer/mfcore/WirelessMacBase.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  base/INETDefs.h \
  base/INotifiable.h
$O/linklayer/ppp/DropsGenerator.o: linklayer/ppp/DropsGenerator.cc \
  linklayer/ppp/DropsGenerator.h \
  base/INETDefs.h
$O/linklayer/ppp/DuplicatesGenerator.o: linklayer/ppp/DuplicatesGenerator.cc \
  base/INETDefs.h \
  linklayer/ppp/DuplicatesGenerator.h
$O/linklayer/ppp/PPP.o: linklayer/ppp/PPP.cc \
  base/NotificationBoard.h \
  base/IPowerControl.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  linklayer/ppp/PPPFrame_m.h \
  base/IPassiveQueue.h \
  base/ModuleAccess.h \
  util/PowerControlManager.h \
  util/PowerControlMessages_m.h \
  networklayer/common/InterfaceToken.h \
  linklayer/ppp/PPP.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  linklayer/contract/TxNotifDetails.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h
$O/linklayer/ppp/PPPFrame_m.o: linklayer/ppp/PPPFrame_m.cc \
  base/INETDefs.h \
  linklayer/ppp/PPPFrame_m.h
$O/linklayer/ppp/ThruputMeter.o: linklayer/ppp/ThruputMeter.cc \
  linklayer/ppp/ThruputMeter.h \
  base/INETDefs.h
$O/linklayer/radio/AbstractRadio.o: linklayer/radio/AbstractRadio.cc \
  util/FWMath.h \
  linklayer/contract/Radio80211aControlInfo_m.h \
  base/BasicModule.h \
  linklayer/radio/IReceptionModel.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  base/Coord.h \
  world/ChannelAccess.h \
  world/obstacles/ObstacleControl.h \
  base/INotifiable.h \
  ./world/obstacles/Obstacle.h \
  linklayer/contract/PhyControlInfo_m.h \
  linklayer/contract/RadioState.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  base/ModuleAccess.h \
  ./world/annotations/AnnotationManager.h \
  linklayer/radio/IRadioModel.h \
  base/NotificationBoard.h \
  linklayer/radio/AbstractRadio.h \
  world/ChannelControl.h \
  linklayer/mfcore/AirFrame_m.h \
  battery/BasicBattery.h \
  base/NotifierConsts.h \
  linklayer/mfcore/SnrList.h
$O/linklayer/radio/AbstractRadioExtended.o: linklayer/radio/AbstractRadioExtended.cc \
  base/IPowerControl.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  linklayer/radio/IRadioModel.h \
  linklayer/mfcore/SnrList.h \
  base/NotifierConsts.h \
  battery/BasicBattery.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  util/PowerControlManager.h \
  util/PowerControlMessages_m.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/contract/RadioState.h \
  ./world/annotations/AnnotationManager.h \
  base/INotifiable.h \
  world/obstacles/ObstacleControl.h \
  world/ChannelAccess.h \
  base/Coord.h \
  ./world/obstacles/Obstacle.h \
  linklayer/contract/PhyControlInfo_m.h \
  world/ChannelAccessExtended.h \
  base/BasicModule.h \
  linklayer/contract/Radio80211aControlInfo_m.h \
  util/FWMath.h \
  linklayer/radio/AbstractRadioExtended.h \
  world/ChannelControlExtended.h \
  base/INETDefs.h \
  linklayer/radio/IReceptionModel.h \
  linklayer/mfcore/ModulationType.h
$O/linklayer/radio/BerParseFile.o: linklayer/radio/BerParseFile.cc \
  linklayer/radio/BerParseFile.h
$O/linklayer/radio/GenericRadio.o: linklayer/radio/GenericRadio.cc \
  base/Coord.h \
  world/ChannelAccess.h \
  world/obstacles/ObstacleControl.h \
  base/INotifiable.h \
  ./world/obstacles/Obstacle.h \
  util/FWMath.h \
  base/BasicModule.h \
  linklayer/radio/IReceptionModel.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  linklayer/radio/GenericRadio.h \
  linklayer/radio/IRadioModel.h \
  base/NotificationBoard.h \
  linklayer/radio/AbstractRadio.h \
  world/ChannelControl.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  linklayer/mfcore/SnrList.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  ./world/annotations/AnnotationManager.h
$O/linklayer/radio/GenericRadioModel.o: linklayer/radio/GenericRadioModel.cc \
  linklayer/mfcore/SnrList.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/radio/GenericRadioModel.h \
  linklayer/radio/Modulation.h \
  linklayer/radio/IRadioModel.h \
  base/Coord.h \
  linklayer/radio/IModulation.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  util/FWMath.h
$O/linklayer/radio/Ieee80211NewRadio.o: linklayer/radio/Ieee80211NewRadio.cc \
  world/ChannelAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  world/obstacles/ObstacleControl.h \
  ./world/obstacles/Obstacle.h \
  util/FWMath.h \
  linklayer/radio/AbstractRadioExtended.h \
  base/BasicModule.h \
  world/ChannelAccessExtended.h \
  base/INETDefs.h \
  linklayer/radio/IReceptionModel.h \
  linklayer/mfcore/ModulationType.h \
  world/ChannelControlExtended.h \
  base/NotificationBoard.h \
  linklayer/radio/IRadioModel.h \
  world/ChannelControl.h \
  base/IPowerControl.h \
  linklayer/radio/AbstractRadio.h \
  linklayer/radio/Ieee80211NewRadio.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  linklayer/contract/RadioState.h \
  base/ModuleAccess.h \
  util/PowerControlMessages_m.h \
  util/PowerControlManager.h \
  ./world/annotations/AnnotationManager.h
$O/linklayer/radio/Ieee80211NewRadioModel.o: linklayer/radio/Ieee80211NewRadioModel.cc \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  linklayer/radio/BerParseFile.h \
  util/FWMath.h \
  linklayer/radio/errormodel/WifiPreambleType.h \
  base/Coord.h \
  linklayer/radio/errormodel/dsss-error-rate-model.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/radio/errormodel/yans-error-rate-model.h \
  linklayer/radio/errormodel/WifiMode.h \
  linklayer/radio/Ieee80211NewRadioModel.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  linklayer/radio/IRadioModel.h
$O/linklayer/radio/Ieee80211Radio.o: linklayer/radio/Ieee80211Radio.cc \
  ./world/obstacles/Obstacle.h \
  base/Coord.h \
  world/ChannelAccess.h \
  base/INotifiable.h \
  world/obstacles/ObstacleControl.h \
  linklayer/mfcore/ModulationType.h \
  linklayer/radio/IReceptionModel.h \
  base/INETDefs.h \
  world/ChannelControlExtended.h \
  linklayer/radio/AbstractRadioExtended.h \
  util/FWMath.h \
  base/BasicModule.h \
  world/ChannelAccessExtended.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  linklayer/radio/Ieee80211Radio.h \
  linklayer/mfcore/SnrList.h \
  linklayer/radio/IRadioModel.h \
  base/NotificationBoard.h \
  linklayer/radio/AbstractRadio.h \
  world/ChannelControl.h \
  base/IPowerControl.h \
  ./world/annotations/AnnotationManager.h \
  linklayer/contract/RadioState.h \
  util/PowerControlMessages_m.h \
  util/PowerControlManager.h \
  base/ModuleAccess.h
$O/linklayer/radio/Ieee80211RadioModel.o: linklayer/radio/Ieee80211RadioModel.cc \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  linklayer/radio/BerParseFile.h \
  util/FWMath.h \
  base/Coord.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  linklayer/radio/Ieee80211RadioModel.h \
  linklayer/radio/IRadioModel.h
$O/linklayer/radio/Ieee80211aRadio.o: linklayer/radio/Ieee80211aRadio.cc \
  world/ChannelControl.h \
  base/IPowerControl.h \
  linklayer/radio/AbstractRadio.h \
  linklayer/radio/IRadioModel.h \
  base/NotificationBoard.h \
  linklayer/mfcore/SnrList.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  linklayer/radio/Ieee80211aRadio.h \
  util/PowerControlMessages_m.h \
  util/PowerControlManager.h \
  linklayer/contract/RadioState.h \
  ./world/annotations/AnnotationManager.h \
  world/obstacles/ObstacleControl.h \
  base/INotifiable.h \
  world/ChannelAccess.h \
  base/Coord.h \
  ./world/obstacles/Obstacle.h \
  base/BasicModule.h \
  world/ChannelAccessExtended.h \
  util/FWMath.h \
  linklayer/radio/AbstractRadioExtended.h \
  world/ChannelControlExtended.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  linklayer/radio/IReceptionModel.h
$O/linklayer/radio/Ieee80211aRadioModel.o: linklayer/radio/Ieee80211aRadioModel.cc \
  linklayer/radio/IRadioModel.h \
  linklayer/radio/qamMode.h \
  linklayer/ieee80211/mac/Ieee80211aConsts.h \
  linklayer/radio/Modulation.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  base/Coord.h \
  linklayer/radio/Ieee80211aRadioModel.h \
  util/FWMath.h \
  linklayer/radio/bpskMode.h \
  linklayer/radio/transmissionMode.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  linklayer/radio/BerParseFile.h \
  linklayer/radio/IModulation.h
$O/linklayer/radio/Ieee80211gRadio.o: linklayer/radio/Ieee80211gRadio.cc \
  linklayer/mfcore/SnrList.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  linklayer/radio/AbstractRadio.h \
  linklayer/radio/Ieee80211gRadio.h \
  world/ChannelControl.h \
  base/IPowerControl.h \
  base/NotificationBoard.h \
  linklayer/radio/IRadioModel.h \
  ./world/annotations/AnnotationManager.h \
  util/PowerControlMessages_m.h \
  util/PowerControlManager.h \
  base/ModuleAccess.h \
  linklayer/contract/RadioState.h \
  ./world/obstacles/Obstacle.h \
  base/INotifiable.h \
  world/obstacles/ObstacleControl.h \
  base/Coord.h \
  world/ChannelAccess.h \
  world/ChannelControlExtended.h \
  linklayer/radio/IReceptionModel.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  world/ChannelAccessExtended.h \
  base/BasicModule.h \
  linklayer/radio/AbstractRadioExtended.h \
  util/FWMath.h
$O/linklayer/radio/Ieee80211gRadioModel.o: linklayer/radio/Ieee80211gRadioModel.cc \
  linklayer/radio/IRadioModel.h \
  linklayer/mfcore/AirFrame_m.h \
  linklayer/mfcore/SnrList.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  base/Coord.h \
  linklayer/radio/Ieee80211gRadioModel.h \
  util/FWMath.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  linklayer/radio/BerParseFile.h
$O/linklayer/radio/Modulation.o: linklayer/radio/Modulation.cc \
  linklayer/radio/Modulation.h \
  linklayer/radio/IModulation.h \
  base/INETDefs.h
$O/linklayer/radio/PathLossReceptionModel.o: linklayer/radio/PathLossReceptionModel.cc \
  linklayer/radio/IReceptionModel.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  linklayer/radio/PathLossReceptionModel.h \
  util/FWMath.h \
  base/Coord.h \
  linklayer/mfcore/AirFrame_m.h \
  world/ChannelControl.h
$O/linklayer/radio/TwoRayModel.o: linklayer/radio/TwoRayModel.cc \
  base/INETDefs.h \
  linklayer/radio/IReceptionModel.h \
  util/FWMath.h \
  linklayer/radio/TwoRayModel.h
$O/linklayer/radio/bpskMode.o: linklayer/radio/bpskMode.cc \
  linklayer/radio/bpskMode.h \
  linklayer/radio/IModulation.h \
  base/INETDefs.h \
  linklayer/radio/transmissionMode.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/radio/qamMode.h
$O/linklayer/radio/qamMode.o: linklayer/radio/qamMode.cc \
  linklayer/radio/IModulation.h \
  base/INETDefs.h \
  linklayer/radio/transmissionMode.h \
  linklayer/radio/qamMode.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/ieee80211/mac/Ieee80211Consts.h
$O/linklayer/radio/transmissionMode.o: linklayer/radio/transmissionMode.cc \
  linklayer/ieee80211/mac/Ieee80211Consts.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/radio/IModulation.h \
  base/INETDefs.h \
  linklayer/radio/transmissionMode.h
$O/linklayer/radio/errormodel/WifiMode.o: linklayer/radio/errormodel/WifiMode.cc \
  linklayer/radio/errormodel/WifiMode.h \
  linklayer/radio/errormodel/Ieee80211DataRate.h \
  linklayer/radio/errormodel/WifiPreambleType.h \
  linklayer/mfcore/ModulationType.h
$O/linklayer/radio/errormodel/dsss-error-rate-model.o: linklayer/radio/errormodel/dsss-error-rate-model.cc \
  linklayer/radio/errormodel/dsss-error-rate-model.h
$O/linklayer/radio/errormodel/yans-error-rate-model.o: linklayer/radio/errormodel/yans-error-rate-model.cc \
  linklayer/radio/errormodel/yans-error-rate-model.h \
  linklayer/radio/errormodel/WifiMode.h \
  linklayer/radio/errormodel/dsss-error-rate-model.h \
  linklayer/radio/errormodel/WifiPreambleType.h \
  linklayer/mfcore/ModulationType.h
$O/linklayer/radio/propagation/FreeSpaceModel.o: linklayer/radio/propagation/FreeSpaceModel.cc \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  world/ChannelAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  util/FWMath.h \
  linklayer/radio/propagation/FreeSpaceModel.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  linklayer/radio/IReceptionModel.h
$O/linklayer/radio/propagation/LogNormalShadowingModel.o: linklayer/radio/propagation/LogNormalShadowingModel.cc \
  linklayer/radio/propagation/LogNormalShadowingModel.h \
  base/INETDefs.h \
  linklayer/radio/IReceptionModel.h \
  linklayer/radio/propagation/FreeSpaceModel.h
$O/linklayer/radio/propagation/NakagamiModel.o: linklayer/radio/propagation/NakagamiModel.cc \
  linklayer/radio/IReceptionModel.h \
  base/INETDefs.h \
  linklayer/radio/propagation/FreeSpaceModel.h \
  linklayer/radio/propagation/NakagamiModel.h
$O/linklayer/radio/propagation/RayleighModel.o: linklayer/radio/propagation/RayleighModel.cc \
  util/FWMath.h \
  linklayer/radio/propagation/FreeSpaceModel.h \
  base/INETDefs.h \
  linklayer/radio/IReceptionModel.h \
  linklayer/radio/propagation/RayleighModel.h
$O/linklayer/radio/propagation/RiceModel.o: linklayer/radio/propagation/RiceModel.cc \
  linklayer/radio/IReceptionModel.h \
  base/INETDefs.h \
  linklayer/radio/propagation/FreeSpaceModel.h \
  util/FWMath.h \
  linklayer/radio/propagation/RiceModel.h
$O/linklayer/radio/propagation/TwoRayGroundModel.o: linklayer/radio/propagation/TwoRayGroundModel.cc \
  linklayer/radio/IReceptionModel.h \
  base/INETDefs.h \
  linklayer/radio/propagation/FreeSpaceModel.h \
  linklayer/radio/propagation/TwoRayGroundModel.h
$O/linklayer/radio/propagation/TwoRayInterferenceModel.o: linklayer/radio/propagation/TwoRayInterferenceModel.cc \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  linklayer/radio/propagation/TwoRayInterferenceModel.h \
  base/ModuleAccess.h \
  base/Coord.h \
  world/ChannelAccess.h \
  base/INotifiable.h \
  linklayer/radio/propagation/FreeSpaceModel.h \
  util/FWMath.h \
  base/BasicModule.h \
  linklayer/mfcore/ModulationType.h \
  linklayer/radio/IReceptionModel.h \
  base/INETDefs.h
$O/mobility/ANSimMobility.o: mobility/ANSimMobility.cc \
  base/BasicModule.h \
  util/FWMath.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/INotifiable.h \
  base/Coord.h \
  mobility/ANSimMobility.h \
  mobility/LineSegmentsMobilityBase.h \
  base/ModuleAccess.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h
$O/mobility/BasicMobility.o: mobility/BasicMobility.cc \
  base/Coord.h \
  base/INotifiable.h \
  util/FWMath.h \
  base/BasicModule.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  mobility/BasicMobility.h \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h
$O/mobility/BonnMotionFileCache.o: mobility/BonnMotionFileCache.cc \
  base/Coord.h \
  base/INotifiable.h \
  mobility/BonnMotionFileCache.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  util/FWMath.h \
  base/BasicModule.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  mobility/BasicMobility.h \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  base/ModuleAccess.h
$O/mobility/BonnMotionMobility.o: mobility/BonnMotionMobility.cc \
  base/ModuleAccess.h \
  mobility/LineSegmentsMobilityBase.h \
  mobility/BonnMotionMobility.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/Coord.h \
  base/INotifiable.h \
  mobility/BonnMotionFileCache.h
$O/mobility/ChiangMobility.o: mobility/ChiangMobility.cc \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  util/FWMath.h \
  base/BasicModule.h \
  mobility/ChiangMobility.h \
  base/Coord.h \
  base/INotifiable.h \
  base/ModuleAccess.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h
$O/mobility/CircleMobility.o: mobility/CircleMobility.cc \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  base/ModuleAccess.h \
  mobility/CircleMobility.h \
  base/INotifiable.h \
  base/Coord.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/BasicModule.h \
  util/FWMath.h
$O/mobility/ConstSpeedMobility.o: mobility/ConstSpeedMobility.cc \
  base/ModuleAccess.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  mobility/ConstSpeedMobility.h \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/Coord.h \
  base/INotifiable.h
$O/mobility/GaussMarkovMobility.o: mobility/GaussMarkovMobility.cc \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/BasicModule.h \
  util/FWMath.h \
  base/INotifiable.h \
  base/Coord.h \
  base/ModuleAccess.h \
  mobility/GaussMarkovMobility.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h
$O/mobility/LineSegmentsMobilityBase.o: mobility/LineSegmentsMobilityBase.cc \
  mobility/LineSegmentsMobilityBase.h \
  base/ModuleAccess.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/BasicModule.h \
  util/FWMath.h \
  base/INotifiable.h \
  base/Coord.h
$O/mobility/LinearMobility.o: mobility/LinearMobility.cc \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  mobility/LinearMobility.h
$O/mobility/MassMobility.o: mobility/MassMobility.cc \
  base/BasicModule.h \
  util/FWMath.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  base/INotifiable.h \
  base/Coord.h \
  mobility/MassMobility.h \
  base/ModuleAccess.h \
  world/ChannelControl.h \
  mobility/BasicMobility.h \
  base/NotificationBoard.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h
$O/mobility/Ns2MotionMobility.o: mobility/Ns2MotionMobility.cc \
  base/INotifiable.h \
  base/Coord.h \
  base/BasicModule.h \
  util/FWMath.h \
  mobility/Ns2MotionMobility.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  mobility/LineSegmentsMobilityBase.h
$O/mobility/NullMobility.o: mobility/NullMobility.cc \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  mobility/NullMobility.h \
  base/INotifiable.h \
  base/Coord.h \
  base/BasicModule.h \
  util/FWMath.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h
$O/mobility/RandomWPMobility.o: mobility/RandomWPMobility.cc \
  mobility/LineSegmentsMobilityBase.h \
  base/ModuleAccess.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/Coord.h \
  base/INotifiable.h \
  mobility/RandomWPMobility.h
$O/mobility/RandomWPMobilityTrace.o: mobility/RandomWPMobilityTrace.cc \
  mobility/RandomWPMobility.h \
  base/Coord.h \
  base/INotifiable.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  mobility/RandomWPMobilityTrace.h \
  util/FWMath.h \
  base/BasicModule.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  mobility/BasicMobility.h \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  mobility/LineSegmentsMobilityBase.h \
  base/ModuleAccess.h
$O/mobility/RectangleMobility.o: mobility/RectangleMobility.cc \
  base/ModuleAccess.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/BasicModule.h \
  util/FWMath.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/INotifiable.h \
  mobility/RectangleMobility.h \
  base/Coord.h
$O/mobility/RestrictedConstSpeedMobility.o: mobility/RestrictedConstSpeedMobility.cc \
  mobility/RestrictedConstSpeedMobility.h \
  base/ModuleAccess.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  mobility/ConstSpeedMobility.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  util/FWMath.h \
  base/BasicModule.h \
  base/Coord.h \
  base/INotifiable.h
$O/mobility/RestrictedLinearMobility.o: mobility/RestrictedLinearMobility.cc \
  base/ModuleAccess.h \
  mobility/RestrictedLinearMobility.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  mobility/LinearMobility.h \
  base/Coord.h \
  base/INotifiable.h
$O/mobility/RestrictedRandomWPMobility.o: mobility/RestrictedRandomWPMobility.cc \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  mobility/RestrictedRandomWPMobility.h \
  world/ChannelControl.h \
  mobility/BasicMobility.h \
  base/NotificationBoard.h \
  mobility/LineSegmentsMobilityBase.h \
  base/ModuleAccess.h \
  mobility/RandomWPMobility.h \
  base/INotifiable.h \
  base/Coord.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  base/BasicModule.h \
  util/FWMath.h
$O/mobility/StaticGridMobility.o: mobility/StaticGridMobility.cc \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  ./mobility/StaticGridMobility.h \
  ./mobility/BasicMobility.h \
  base/ModuleAccess.h \
  base/Coord.h \
  base/INotifiable.h \
  util/FWMath.h \
  base/BasicModule.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h
$O/mobility/TurtleMobility.o: mobility/TurtleMobility.cc \
  world/ChannelControl.h \
  mobility/BasicMobility.h \
  base/NotificationBoard.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h \
  mobility/LineSegmentsMobilityBase.h \
  mobility/TurtleMobility.h \
  base/INotifiable.h \
  base/Coord.h \
  base/BasicModule.h \
  util/FWMath.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h
$O/mobility/traci/TraCIMobility.o: mobility/traci/TraCIMobility.cc \
  base/Coord.h \
  ./mobility/traci/TraCIMobility.h \
  base/INotifiable.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  util/FWMath.h \
  ./world/traci/TraCIScenarioManager.h \
  base/BasicModule.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  mobility/BasicMobility.h \
  base/NotificationBoard.h \
  world/ChannelControl.h \
  base/ModuleAccess.h
$O/networklayer/arp/ARP.o: networklayer/arp/ARP.cc \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/arp/ARPPacket_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/arp/ARP.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotificationBoard.h
$O/networklayer/arp/ARPPacket_m.o: networklayer/arp/ARPPacket_m.cc \
  linklayer/contract/MACAddress.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  networklayer/arp/ARPPacket_m.h
$O/networklayer/autorouting/FlatNetworkConfigurator.o: networklayer/autorouting/FlatNetworkConfigurator.cc \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/autorouting/FlatNetworkConfigurator.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h
$O/networklayer/autorouting/FlatNetworkConfigurator6.o: networklayer/autorouting/FlatNetworkConfigurator6.cc \
  networklayer/ipv6/RoutingTable6.h \
  base/ModuleAccess.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/ipv6/IPv6InterfaceData.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/common/InterfaceToken.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/autorouting/FlatNetworkConfigurator6.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  base/INotifiable.h \
  networklayer/common/IInterfaceTable.h
$O/networklayer/autorouting/HostAutoConfigurator.o: networklayer/autorouting/HostAutoConfigurator.cc \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/autorouting/HostAutoConfigurator.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPAddressResolver.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h
$O/networklayer/autorouting/HostAutoConfigurator2.o: networklayer/autorouting/HostAutoConfigurator2.cc \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/autorouting/HostAutoConfigurator2.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h
$O/networklayer/autorouting/NetworkConfigurator.o: networklayer/autorouting/NetworkConfigurator.cc \
  networklayer/contract/IPAddressResolver.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/autorouting/NetworkConfigurator.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h
$O/networklayer/common/InterfaceEntry.o: networklayer/common/InterfaceEntry.cc \
  networklayer/common/IInterfaceTable.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/contract/IPv6Address.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv6/IPv6InterfaceData.h
$O/networklayer/common/InterfaceTable.o: networklayer/common/InterfaceTable.cc \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/common/IInterfaceTable.h \
  base/INotifiable.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceTable.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h
$O/networklayer/contract/IPAddress.o: networklayer/contract/IPAddress.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/networklayer/contract/IPAddressResolver.o: networklayer/contract/IPAddressResolver.cc \
  base/NotificationBoard.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv6/RoutingTable6.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/ipv6/IPv6InterfaceData.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  base/INotifiable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h
$O/networklayer/contract/IPControlInfo.o: networklayer/contract/IPControlInfo.cc \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h
$O/networklayer/contract/IPControlInfo_m.o: networklayer/contract/IPControlInfo_m.cc \
  networklayer/contract/IPProtocolId_m.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  linklayer/contract/MACAddress.h \
  networklayer/contract/IPControlInfo_m.h
$O/networklayer/contract/IPProtocolId_m.o: networklayer/contract/IPProtocolId_m.cc \
  base/INETDefs.h \
  networklayer/contract/IPProtocolId_m.h
$O/networklayer/contract/IPv6Address.o: networklayer/contract/IPv6Address.cc \
  base/INETDefs.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6Address.h
$O/networklayer/contract/IPv6ControlInfo.o: networklayer/contract/IPv6ControlInfo.cc \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/ipv6/IPv6Datagram.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/contract/IPProtocolId_m.h
$O/networklayer/contract/IPv6ControlInfo_m.o: networklayer/contract/IPv6ControlInfo_m.cc \
  networklayer/contract/IPProtocolId_m.h \
  base/INETDefs.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPv6ControlInfo_m.h
$O/networklayer/contract/IPvXAddress.o: networklayer/contract/IPvXAddress.cc \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h
$O/networklayer/extras/Dummy.o: networklayer/extras/Dummy.cc
$O/networklayer/extras/FailureManager.o: networklayer/extras/FailureManager.cc \
  base/INETDefs.h \
  base/IScriptable.h \
  networklayer/extras/FailureManager.h
$O/networklayer/extras/NetworkInfo.o: networklayer/extras/NetworkInfo.cc \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/extras/NetworkInfo.h \
  base/INETDefs.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  base/IScriptable.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPAddressResolver.h
$O/networklayer/icmpv6/ICMPv6.o: networklayer/icmpv6/ICMPv6.cc \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv6/RoutingTable6Access.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INotifiable.h \
  networklayer/icmpv6/ICMPv6.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv6/RoutingTable6.h \
  base/ModuleAccess.h \
  networklayer/contract/IPv6Address.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/ipv6/IPv6Datagram.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h
$O/networklayer/icmpv6/ICMPv6Message_m.o: networklayer/icmpv6/ICMPv6Message_m.cc \
  networklayer/icmpv6/ICMPv6Message_m.h \
  base/INETDefs.h
$O/networklayer/icmpv6/IPv6NDMessage_m.o: networklayer/icmpv6/IPv6NDMessage_m.cc \
  base/INETDefs.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/icmpv6/IPv6NDMessage_m.h \
  linklayer/contract/MACAddress.h
$O/networklayer/icmpv6/IPv6NeighbourCache.o: networklayer/icmpv6/IPv6NeighbourCache.cc \
  networklayer/contract/IPv6Address.h \
  linklayer/contract/MACAddress.h \
  base/INETDefs.h \
  networklayer/icmpv6/IPv6NeighbourCache.h
$O/networklayer/icmpv6/IPv6NeighbourDiscovery.o: networklayer/icmpv6/IPv6NeighbourDiscovery.cc \
  networklayer/ipv6/RoutingTable6Access.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/icmpv6/ICMPv6Access.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/icmpv6/IPv6NeighbourDiscovery.h \
  base/INotifiable.h \
  networklayer/icmpv6/ICMPv6.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/icmpv6/IPv6NeighbourCache.h \
  networklayer/contract/IPProtocolId_m.h \
  base/ModuleAccess.h \
  networklayer/ipv6/IPv6InterfaceData.h \
  networklayer/ipv6/RoutingTable6.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  base/NotificationBoard.h \
  networklayer/ipv6/IPv6Datagram.h \
  networklayer/icmpv6/IPv6NDMessage_m.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h
$O/networklayer/ipv4/ControlManetRouting_m.o: networklayer/ipv4/ControlManetRouting_m.cc \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/networklayer/ipv4/ErrorHandling.o: networklayer/ipv4/ErrorHandling.cc \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/contract/MACAddress.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ErrorHandling.h
$O/networklayer/ipv4/ICMP.o: networklayer/ipv4/ICMP.cc \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h
$O/networklayer/ipv4/ICMPMessage_m.o: networklayer/ipv4/ICMPMessage_m.cc \
  base/INETDefs.h \
  networklayer/ipv4/ICMPMessage_m.h
$O/networklayer/ipv4/IP.o: networklayer/ipv4/IP.cc \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  base/QueueBase.h \
  base/ReassemblyBuffer.h \
  base/AbstractQueue.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/base/uint128.h \
  transport/udp/UDPPacket_m.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  transport/tcp/TCPSegment.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/arp/ARPPacket_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/ProtocolMap.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IPFragBuf.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/IP.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h
$O/networklayer/ipv4/IPDatagram_m.o: networklayer/ipv4/IPDatagram_m.cc \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h
$O/networklayer/ipv4/IPFragBuf.o: networklayer/ipv4/IPFragBuf.cc \
  base/ReassemblyBuffer.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ipv4/IPDatagram.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/IPFragBuf.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h
$O/networklayer/ipv4/IPRoute.o: networklayer/ipv4/IPRoute.cc \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  networklayer/common/InterfaceToken.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h
$O/networklayer/ipv4/IPRouteRule.o: networklayer/ipv4/IPRouteRule.cc \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h
$O/networklayer/ipv4/IPv4InterfaceData.o: networklayer/ipv4/IPv4InterfaceData.cc \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  networklayer/common/InterfaceToken.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h
$O/networklayer/ipv4/RoutingTable.o: networklayer/ipv4/RoutingTable.cc \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  base/ModuleAccess.h \
  networklayer/ipv4/RoutingTableParser.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  base/NotificationBoard.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h
$O/networklayer/ipv4/RoutingTableParser.o: networklayer/ipv4/RoutingTableParser.cc \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/RoutingTableParser.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  base/INotifiable.h
$O/networklayer/ipv6/IPv6.o: networklayer/ipv6/IPv6.cc \
  networklayer/icmpv6/IPv6NeighbourDiscoveryAccess.h \
  base/INotifiable.h \
  networklayer/icmpv6/ICMPv6.h \
  base/ProtocolMap.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/icmpv6/IPv6NeighbourCache.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/ipv6/RoutingTable6Access.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/icmpv6/ICMPv6Access.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  networklayer/icmpv6/IPv6NeighbourDiscovery.h \
  base/NotificationBoard.h \
  base/ReassemblyBuffer.h \
  networklayer/ipv6/IPv6Datagram.h \
  base/QueueBase.h \
  networklayer/ipv6/IPv6.h \
  networklayer/icmpv6/IPv6NDMessage_m.h \
  base/NotifierConsts.h \
  networklayer/ipv6/IPv6FragBuf.h \
  linklayer/contract/MACAddress.h \
  base/ModuleAccess.h \
  networklayer/ipv6/IPv6InterfaceData.h \
  networklayer/ipv6/RoutingTable6.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  base/AbstractQueue.h \
  networklayer/contract/IPv6Address.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/ipv6/IPv6Datagram_m.h
$O/networklayer/ipv6/IPv6Datagram.o: networklayer/ipv6/IPv6Datagram.cc \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv6/IPv6Datagram.h \
  networklayer/ipv6/IPv6ExtensionHeaders_m.h \
  base/INETDefs.h \
  networklayer/contract/IPProtocolId_m.h
$O/networklayer/ipv6/IPv6Datagram_m.o: networklayer/ipv6/IPv6Datagram_m.cc \
  base/INETDefs.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv6/IPv6Datagram_m.h
$O/networklayer/ipv6/IPv6ErrorHandling.o: networklayer/ipv6/IPv6ErrorHandling.cc \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv6/IPv6ErrorHandling.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/ipv6/IPv6Datagram.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPv6ControlInfo_m.h
$O/networklayer/ipv6/IPv6FragBuf.o: networklayer/ipv6/IPv6FragBuf.cc \
  networklayer/ipv6/RoutingTable6Access.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  networklayer/icmpv6/ICMPv6.h \
  base/INotifiable.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  base/ModuleAccess.h \
  networklayer/ipv6/RoutingTable6.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/common/InterfaceToken.h \
  base/ReassemblyBuffer.h \
  networklayer/ipv6/IPv6ExtensionHeaders_m.h \
  base/NotificationBoard.h \
  networklayer/ipv6/IPv6Datagram.h \
  networklayer/ipv6/IPv6FragBuf.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h
$O/networklayer/ipv6/IPv6InterfaceData.o: networklayer/ipv6/IPv6InterfaceData.cc \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv6/IPv6InterfaceData.h
$O/networklayer/ipv6/RoutingTable6.o: networklayer/ipv6/RoutingTable6.cc \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  networklayer/contract/IPv6Address.h \
  networklayer/common/InterfaceToken.h \
  base/ModuleAccess.h \
  networklayer/ipv6/IPv6InterfaceData.h \
  networklayer/ipv6/RoutingTable6.h \
  networklayer/common/IInterfaceTable.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  util/opp_utils.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h
$O/networklayer/ldp/LDP.o: networklayer/ldp/LDP.cc \
  networklayer/ipv4/IRoutingTable.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  networklayer/ted/TED_m.h \
  networklayer/common/InterfaceEntry.h \
  transport/udp/UDPPacket.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  networklayer/ted/TEDAccess.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  transport/contract/TCPSocketMap.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  networklayer/mpls/IClassifier.h \
  networklayer/mpls/LIBTable.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/contract/TCPSocket.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  transport/udp/UDPPacket_m.h \
  networklayer/mpls/LIBTableAccess.h \
  transport/tcp/TCPSegment.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/rsvp_te/IntServ.h \
  linklayer/contract/MACAddress.h \
  networklayer/ldp/LDP.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ldp/LDPPacket_m.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/rsvp_te/IntServ_m.h \
  networklayer/ted/TED.h \
  networklayer/mpls/ConstType.h \
  base/NotificationBoard.h
$O/networklayer/manetrouting/aodv/aodv_msg_struct.o: networklayer/manetrouting/aodv/aodv_msg_struct.cc \
  networklayer/common/InterfaceTable.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/aodv/aodv_msg_struct.h \
  transport/udp/UDPPacket_m.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  base/NotificationBoard.h \
  base/NotifierConsts.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  transport/udp/UDPPacket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h
$O/networklayer/manetrouting/aodv/aodv_uu_omnet.o: networklayer/manetrouting/aodv/aodv_uu_omnet.cc \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/manetrouting/aodv/aodv_uu_omnet.h \
  networklayer/manetrouting/base/uint128.h \
  transport/udp/UDPPacket_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/aodv/aodv_msg_struct.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/common/InterfaceEntry.h \
  transport/udp/UDPPacket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/locality.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/packet_queue_omnet.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  base/INotifiable.h \
  base/ProtocolMap.h
$O/networklayer/manetrouting/aodv/packet_queue_omnet.o: networklayer/manetrouting/aodv/packet_queue_omnet.cc \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/aodv/aodv_uu_omnet.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/aodv/aodv_msg_struct.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/packet_queue_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  base/INETDefs.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/locality.h
$O/networklayer/manetrouting/aodv/aodv-uu/aodv_hello.o: networklayer/manetrouting/aodv/aodv-uu/aodv_hello.cc \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/ipv4/IPRoute.h \
  base/INETDefs.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h
$O/networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.o: networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.cc \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h
$O/networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.o: networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.cc \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/ipv4/ICMP.h
$O/networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.o: networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.cc \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h
$O/networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.o: networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.cc \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/manetrouting/aodv/aodv-uu/locality.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h
$O/networklayer/manetrouting/aodv/aodv-uu/aodv_socket.o: networklayer/manetrouting/aodv/aodv-uu/aodv_socket.cc \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  base/INotifiable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h
$O/networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.o: networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.cc \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/nl.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  base/INotifiable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h
$O/networklayer/manetrouting/aodv/aodv-uu/debug_aodv.o: networklayer/manetrouting/aodv/aodv-uu/debug_aodv.cc \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h
$O/networklayer/manetrouting/aodv/aodv-uu/list.o: networklayer/manetrouting/aodv/aodv-uu/list.cc \
  networklayer/manetrouting/aodv/aodv-uu/list.h
$O/networklayer/manetrouting/aodv/aodv-uu/locality.o: networklayer/manetrouting/aodv/aodv-uu/locality.cc \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h
$O/networklayer/manetrouting/aodv/aodv-uu/routing_table.o: networklayer/manetrouting/aodv/aodv-uu/routing_table.cc \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/nl.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/routing_table.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_rrep.h \
  networklayer/ipv4/RoutingTableAccess.h
$O/networklayer/manetrouting/aodv/aodv-uu/seek_list.o: networklayer/manetrouting/aodv/aodv-uu/seek_list.cc \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/manetrouting/aodv/aodv-uu/params.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/aodv_timeout.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/seek_list.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h
$O/networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.o: networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.cc \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/routing_table.h \
  base/INotifiable.h \
  networklayer/manetrouting/aodv/aodv-uu/debug_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/seek_list.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rerr.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_timeout.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_msg_struct.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/timer_queue_aodv.h \
  base/INETDefs.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rrep.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/locality.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_neighbor.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/list.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/params.h \
  networklayer/ipv4/ICMPAccess.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_rreq.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/aodv/aodv-uu/../packet_queue_omnet.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv_uu_omnet.h \
  networklayer/manetrouting/aodv/aodv-uu/list.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/defs_aodv.h \
  networklayer/manetrouting/aodv/aodv-uu/defs_aodv.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/aodv/aodv-uu/timer_queue_aodv.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/aodv/aodv-uu/../aodv-uu/aodv_hello.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/networklayer/manetrouting/base/ManetRoutingBase.o: networklayer/manetrouting/base/ManetRoutingBase.cc \
  networklayer/common/InterfaceEntry.h \
  transport/udp/UDPPacket.h \
  networklayer/ipv4/IPRoute.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  util/FWMath.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  base/Coord.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/contract/IPControlInfo_m.h \
  linklayer/contract/ControlInfoBreakLink_m.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/base/uint128.h \
  transport/udp/UDPPacket_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  base/NotificationBoard.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h
$O/networklayer/manetrouting/base/int128.o: networklayer/manetrouting/base/int128.cc \
  networklayer/manetrouting/base/int128.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/networklayer/manetrouting/base/uint128.o: networklayer/manetrouting/base/uint128.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h
$O/networklayer/manetrouting/batman/BatmanData.o: networklayer/manetrouting/batman/BatmanData.cc \
  networklayer/manetrouting/batman/BatmanMsg_m.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/batman/batman.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  base/NotificationBoard.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/batman/BatmanData.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  base/INETDefs.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INotifiable.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h
$O/networklayer/manetrouting/batman/BatmanMain.o: networklayer/manetrouting/batman/BatmanMain.cc \
  transport/udp/UDPPacket_m.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/batman/BatmanMsg_m.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/batman/batman.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/batman/BatmanData.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  base/INotifiable.h
$O/networklayer/manetrouting/batman/batmanHna.o: networklayer/manetrouting/batman/batmanHna.cc \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/batman/BatmanData.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/batman/BatmanMsg_m.h \
  networklayer/manetrouting/batman/batman.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  base/INotifiable.h \
  base/INETDefs.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h
$O/networklayer/manetrouting/dsdv/DSDV_2.o: networklayer/manetrouting/dsdv/DSDV_2.cc \
  base/NotificationBoard.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsdv/DSDV_2.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsdv/DSDVhello_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h
$O/networklayer/manetrouting/dsr/dsr-pkt_omnet.o: networklayer/manetrouting/dsr/dsr-pkt_omnet.cc \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  util/headerserializers/headers/ip.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h
$O/networklayer/manetrouting/dsr/dsr-uu-omnetpp.o: networklayer/manetrouting/dsr/dsr-uu-omnetpp.cc \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/ipv4/ControlManetRouting.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  networklayer/common/InterfaceToken.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-ack.o: networklayer/manetrouting/dsr/dsr-uu/dsr-ack.cc \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  base/Blackboard.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-dev.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-io.o: networklayer/manetrouting/dsr/dsr-uu/dsr-io.cc \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  base/ProtocolMap.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/INotifiable.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rtc.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  base/NotifierConsts.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-dev.h \
  base/NotificationBoard.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-opt.o: networklayer/manetrouting/dsr/dsr-uu/dsr-opt.cc \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  util/headerserializers/headers/ip.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  base/NotificationBoard.h \
  base/NotifierConsts.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/RoutingTableAccess.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.o: networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.cc \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  base/Blackboard.h \
  base/NotifierConsts.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/ICMP.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  base/INETDefs.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.o: networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.cc \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-dev.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.o: networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.cc \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/ProtocolMap.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-dev.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.o: networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.cc \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  util/headerserializers/headers/ip.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-dev.h \
  base/NotificationBoard.h \
  base/Blackboard.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h
$O/networklayer/manetrouting/dsr/dsr-uu/dsr-srt.o: networklayer/manetrouting/dsr/dsr-uu/dsr-srt.cc \
  base/NotificationBoard.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  base/Blackboard.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  util/headerserializers/headers/ip.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h
$O/networklayer/manetrouting/dsr/dsr-uu/link-cache.o: networklayer/manetrouting/dsr/dsr-uu/link-cache.cc \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  base/NotifierConsts.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/ICMP.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rtc.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  base/INETDefs.h
$O/networklayer/manetrouting/dsr/dsr-uu/maint-buf.o: networklayer/manetrouting/dsr/dsr-uu/maint-buf.cc \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-dev.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/ICMP.h
$O/networklayer/manetrouting/dsr/dsr-uu/neigh.o: networklayer/manetrouting/dsr/dsr-uu/neigh.cc \
  base/Blackboard.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotificationBoard.h \
  util/headerserializers/headers/ip.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h
$O/networklayer/manetrouting/dsr/dsr-uu/path-cache.o: networklayer/manetrouting/dsr/dsr-uu/path-cache.cc \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rtc.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  base/NotifierConsts.h \
  base/Blackboard.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  networklayer/ipv4/IPv4InterfaceData.h
$O/networklayer/manetrouting/dsr/dsr-uu/send-buf.o: networklayer/manetrouting/dsr/dsr-uu/send-buf.cc \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-ack.h \
  networklayer/manetrouting/dsr/dsr-pkt_omnet.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-pkt.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/link-cache.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rreq.h \
  base/Blackboard.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  util/headerserializers/headers/ip.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-srt.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/dsr/dsr-uu/tbl.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/path-cache.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr_list.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dsr/dsr-uu/timer.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rrep.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dsr/dsr-uu/maint-buf.h \
  networklayer/manetrouting/dsr/dsr-uu-omnetpp.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/INotifiable.h \
  networklayer/manetrouting/base/compatibility_dsr.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-opt.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dsr/dsr-uu/neigh.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dsr/dsr-uu/ns-agent.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-io.h \
  networklayer/manetrouting/dsr/dsr-uu/debug_dsr.h \
  base/INETDefs.h \
  networklayer/manetrouting/dsr/dsr-uu/dsr-rerr.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dsr/dsr-uu/send-buf.h \
  networklayer/ipv4/ICMPMessage_m.h
$O/networklayer/manetrouting/dymo/dymo_msg_struct.o: networklayer/manetrouting/dymo/dymo_msg_struct.cc \
  networklayer/manetrouting/dymo/dymo_msg_struct.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/base/uint128.h
$O/networklayer/manetrouting/dymo/dymo_packet_queue_omnet.o: networklayer/manetrouting/dymo/dymo_packet_queue_omnet.cc \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dymo/dymoum/blacklist.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymo_msg_struct.h \
  networklayer/manetrouting/dymo/dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymo_um_omnet.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dymo/dymoum/dymo_hello.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo/dymoum/pending_rreq.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo/dymoum/dymo_nb.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo/dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/dymo_rerr.h \
  base/NotificationBoard.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/dymo_timeout.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymo_packet_queue_omnet.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/dymo_socket.h
$O/networklayer/manetrouting/dymo/dymo_um_omnet.o: networklayer/manetrouting/dymo/dymo_um_omnet.cc \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/ControlManetRouting.h \
  networklayer/manetrouting/dymo/dymoum/dymo_nb.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo/dymoum/icmp_socket.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/dymo_rerr.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/dymo_timeout.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo/dymo_packet_queue_omnet.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  transport/udp/UDPPacket_m.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/dymo_socket.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo/dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymo_msg_struct.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/INotifiable.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/dymo/dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymo_um_omnet.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo/dymoum/dymo_hello.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/udp/UDPPacket.h \
  networklayer/manetrouting/dymo/dymoum/pending_rreq.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h
$O/networklayer/manetrouting/dymo/dymoum/blacklist.o: networklayer/manetrouting/dymo/dymoum/blacklist.cc \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/blacklist.h \
  base/INotifiable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h
$O/networklayer/manetrouting/dymo/dymoum/debug_dymo.o: networklayer/manetrouting/dymo/dymoum/debug_dymo.cc \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  base/INETDefs.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_generic.o: networklayer/manetrouting/dymo/dymoum/dymo_generic.cc \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/dymo/dymoum/dymo_hello.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/dymo_rerr.h \
  base/NotificationBoard.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_hello.o: networklayer/manetrouting/dymo/dymoum/dymo_hello.cc \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/dymo_nb.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/manetrouting/dymo/dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dymo/dymoum/dymo_hello.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_nb.o: networklayer/manetrouting/dymo/dymoum/dymo_nb.cc \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo/dymoum/dymo_nb.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/dymo_timeout.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_re.o: networklayer/manetrouting/dymo/dymoum/dymo_re.cc \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo/dymoum/icmp_socket.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/dymo_timeout.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/contract/MeshControlInfo_m.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/manetrouting/dymo/dymoum/pending_rreq.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_rerr.o: networklayer/manetrouting/dymo/dymoum/dymo_rerr.cc \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/dymo_socket.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_socket.o: networklayer/manetrouting/dymo/dymoum/dymo_socket.cc \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/dymo_socket.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_timeout.o: networklayer/manetrouting/dymo/dymoum/dymo_timeout.cc \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dymo/dymoum/pending_rreq.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/dymo_netlink.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/manetrouting/dymo/dymoum/dymo_timeout.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dymo/dymoum/dymo_nb.h
$O/networklayer/manetrouting/dymo/dymoum/dymo_uerr.o: networklayer/manetrouting/dymo/dymoum/dymo_uerr.cc \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/dymo_generic.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/dymo_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo/dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h
$O/networklayer/manetrouting/dymo/dymoum/icmp_socket.o: networklayer/manetrouting/dymo/dymoum/icmp_socket.cc \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo/dymoum/icmp_socket.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h
$O/networklayer/manetrouting/dymo/dymoum/pending_rreq.o: networklayer/manetrouting/dymo/dymoum/pending_rreq.cc \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dymo/dymoum/pending_rreq.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h
$O/networklayer/manetrouting/dymo/dymoum/rtable.o: networklayer/manetrouting/dymo/dymoum/rtable.cc \
  networklayer/manetrouting/dymo/dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/dymo/dymoum/dymo_timeout.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/dymo_netlink.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  networklayer/manetrouting/dymo/dymoum/k_route.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  base/INETDefs.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/dymo/dymoum/pending_rreq.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo/dymoum/rtable.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h
$O/networklayer/manetrouting/dymo/dymoum/timer_queue.o: networklayer/manetrouting/dymo/dymoum/timer_queue.cc \
  networklayer/manetrouting/dymo/dymoum/../dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_nb.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_socket.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_rerr.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_hello.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_packet_queue_omnet.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_msg_struct.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_generic.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo/dymoum/defs_dymo.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo/dymoum/dlist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/debug_dymo.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/blacklist.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_timeout.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  linklayer/ieee80211/mac/Ieee80211Frame_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/pending_rreq.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo/dymoum/timer_queue.h \
  networklayer/manetrouting/dymo/dymoum/../dymo_um_omnet.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/rtable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/icmp_socket.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_re.h \
  base/INotifiable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/defs_dymo.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dymo_uerr.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo/dymoum/../dymoum/dlist.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h
$O/networklayer/manetrouting/dymo_fau/DYMO.o: networklayer/manetrouting/dymo_fau/DYMO.cc \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingEntry.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timer.h \
  networklayer/manetrouting/dymo_fau/DYMO.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/ipv4/IPFragBuf.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo_fau/DYMO_Packet_m.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  transport/udp/UDPPacket.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_DataQueue.h \
  networklayer/ipv4/IP.h \
  networklayer/manetrouting/dymo_fau/DYMO_UERR_m.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREP_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREQ_m.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/dymo_fau/InspectorHelpers.h \
  networklayer/manetrouting/dymo_fau/DYMO_AddressBlock.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  base/AbstractQueue.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  transport/udp/UDPPacket_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timeout_m.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingTable.h \
  networklayer/manetrouting/dymo_fau/DYMO_TokenBucket.h \
  networklayer/common/InterfaceTable.h \
  networklayer/ipv4/IPDatagram.h \
  linklayer/contract/MACAddress.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/dymo_fau/DYMO_RM_m.h \
  base/QueueBase.h \
  networklayer/manetrouting/dymo_fau/DYMO_PacketBBMessage_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_RERR_m.h \
  base/ReassemblyBuffer.h \
  base/NotificationBoard.h
$O/networklayer/manetrouting/dymo_fau/DYMO_DataQueue.o: networklayer/manetrouting/dymo_fau/DYMO_DataQueue.cc \
  base/NotificationBoard.h \
  base/ReassemblyBuffer.h \
  base/QueueBase.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/common/InterfaceTable.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  base/AbstractQueue.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/ProtocolMap.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_DataQueue.h \
  networklayer/ipv4/IP.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/ipv4/IPFragBuf.h \
  base/INETDefs.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h
$O/networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.o: networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.cc \
  networklayer/manetrouting/dymo_fau/DYMO_Timeout_m.h \
  base/ModuleAccess.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  base/INETDefs.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timer.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.h \
  base/INotifiable.h
$O/networklayer/manetrouting/dymo_fau/DYMO_RoutingEntry.o: networklayer/manetrouting/dymo_fau/DYMO_RoutingEntry.cc \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_TokenBucket.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingTable.h \
  networklayer/common/InterfaceTable.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  base/AbstractQueue.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timeout_m.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo_fau/DYMO_RM_m.h \
  base/QueueBase.h \
  networklayer/manetrouting/dymo_fau/DYMO_PacketBBMessage_m.h \
  base/ReassemblyBuffer.h \
  networklayer/manetrouting/dymo_fau/DYMO_RERR_m.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IP.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_DataQueue.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingEntry.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timer.h \
  networklayer/manetrouting/dymo_fau/DYMO.h \
  base/INETDefs.h \
  networklayer/ipv4/IPFragBuf.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/manetrouting/dymo_fau/DYMO_Packet_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREP_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREQ_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/ProtocolMap.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/manetrouting/dymo_fau/InspectorHelpers.h \
  networklayer/manetrouting/dymo_fau/DYMO_AddressBlock.h \
  networklayer/manetrouting/dymo_fau/DYMO_UERR_m.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h
$O/networklayer/manetrouting/dymo_fau/DYMO_RoutingTable.o: networklayer/manetrouting/dymo_fau/DYMO_RoutingTable.cc \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.h \
  networklayer/manetrouting/dymo_fau/DYMO_UERR_m.h \
  networklayer/contract/IPProtocolId_m.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/manetrouting/dymo_fau/InspectorHelpers.h \
  networklayer/manetrouting/dymo_fau/DYMO_AddressBlock.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREQ_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREP_m.h \
  base/INotifiable.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/ProtocolMap.h \
  networklayer/manetrouting/dymo_fau/DYMO.h \
  networklayer/manetrouting/dymo_fau/DYMO_Packet_m.h \
  base/INETDefs.h \
  networklayer/ipv4/IPFragBuf.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingEntry.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timer.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IP.h \
  networklayer/manetrouting/dymo_fau/DYMO_DataQueue.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_PacketBBMessage_m.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo_fau/DYMO_RERR_m.h \
  base/ReassemblyBuffer.h \
  networklayer/manetrouting/dymo_fau/DYMO_RM_m.h \
  base/QueueBase.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timeout_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  base/AbstractQueue.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingTable.h \
  networklayer/manetrouting/dymo_fau/DYMO_TokenBucket.h \
  networklayer/common/InterfaceTable.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h
$O/networklayer/manetrouting/dymo_fau/DYMO_Timer.o: networklayer/manetrouting/dymo_fau/DYMO_Timer.cc \
  networklayer/manetrouting/dymo_fau/DYMO_Timeout_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timer.h
$O/networklayer/manetrouting/dymo_fau/DYMO_TokenBucket.o: networklayer/manetrouting/dymo_fau/DYMO_TokenBucket.cc \
  networklayer/manetrouting/dymo_fau/InspectorHelpers.h \
  networklayer/manetrouting/dymo_fau/DYMO_AddressBlock.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREP_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_RREQ_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/ProtocolMap.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  base/INotifiable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_UERR_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_OutstandingRREQList.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IP.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_DataQueue.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  networklayer/ipv4/IPFragBuf.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/manetrouting/dymo_fau/DYMO_Packet_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingEntry.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timer.h \
  networklayer/manetrouting/dymo_fau/DYMO_PacketBBMessage_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_RERR_m.h \
  base/ReassemblyBuffer.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/dymo_fau/DYMO_RM_m.h \
  base/QueueBase.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/dymo_fau/DYMO_TokenBucket.h \
  networklayer/manetrouting/dymo_fau/DYMO_RoutingTable.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/common/InterfaceTable.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/dymo_fau/DYMO_Timeout_m.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  base/AbstractQueue.h
$O/networklayer/manetrouting/dymo_fau/InspectorHelpers.o: networklayer/manetrouting/dymo_fau/InspectorHelpers.cc \
  networklayer/manetrouting/dymo_fau/InspectorHelpers.h \
  networklayer/manetrouting/dymo_fau/DYMO_AddressBlock.h \
  networklayer/manetrouting/dymo_fau/DYMO_PacketBBMessage_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_RERR_m.h \
  networklayer/manetrouting/dymo_fau/DYMO_RM_m.h
$O/networklayer/manetrouting/manager/ManetManager.o: networklayer/manetrouting/manager/ManetManager.cc \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/manager/ManetManager.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ipv4/ControlManetRouting_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/ipv4/IPDatagram.h \
  base/ModuleAccess.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h
$O/networklayer/manetrouting/olsr/OLSR.o: networklayer/manetrouting/olsr/OLSR.cc \
  base/NotificationBoard.h \
  networklayer/manetrouting/olsr/OLSR_state.h \
  networklayer/manetrouting/olsr/OLSR_rtable.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/olsr/OLSR.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/olsr/OLSR_repositories.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/udp/UDPPacket_m.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  linklayer/contract/Ieee802Ctrl_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/IPRoute.h \
  transport/udp/UDPPacket.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h
$O/networklayer/manetrouting/olsr/OLSR_ETX.o: networklayer/manetrouting/olsr/OLSR_ETX.cc \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  networklayer/manetrouting/olsr/OLSR_ETX_state.h \
  networklayer/manetrouting/olsr/OLSR_ETX_repositories.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  networklayer/manetrouting/olsr/OLSR_ETX_dijkstra.h \
  networklayer/ipv4/IPRoute.h \
  transport/udp/UDPPacket.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/olsr/OLSR.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/olsr/OLSR_state.h \
  networklayer/manetrouting/olsr/OLSR_rtable.h \
  base/NotificationBoard.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  transport/udp/UDPPacket_m.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/olsr/OLSR_ETX.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/olsr/OLSR_repositories.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h
$O/networklayer/manetrouting/olsr/OLSR_ETX_dijkstra.o: networklayer/manetrouting/olsr/OLSR_ETX_dijkstra.cc \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/olsr/OLSR.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/olsr/OLSR_state.h \
  networklayer/manetrouting/olsr/OLSR_rtable.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/olsr/OLSR_ETX.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/olsr/OLSR_repositories.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h \
  base/INotifiable.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/manetrouting/olsr/OLSR_ETX_state.h \
  networklayer/manetrouting/olsr/OLSR_ETX_repositories.h \
  base/INETDefs.h \
  networklayer/manetrouting/olsr/OLSR_ETX_dijkstra.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  networklayer/ipv4/ICMPMessage_m.h
$O/networklayer/manetrouting/olsr/OLSR_ETX_state.o: networklayer/manetrouting/olsr/OLSR_ETX_state.cc \
  base/NotificationBoard.h \
  networklayer/manetrouting/olsr/OLSR_rtable.h \
  networklayer/manetrouting/olsr/OLSR_state.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/manetrouting/olsr/OLSR.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/olsr/OLSR_repositories.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/manetrouting/olsr/OLSR_ETX.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  base/INotifiable.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  base/INETDefs.h \
  networklayer/manetrouting/olsr/OLSR_ETX_state.h \
  networklayer/manetrouting/olsr/OLSR_ETX_repositories.h
$O/networklayer/manetrouting/olsr/OLSR_rtable.o: networklayer/manetrouting/olsr/OLSR_rtable.cc \
  base/INotifiable.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  base/INETDefs.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  networklayer/ipv4/IRoutingTable.h \
  base/NotificationBoard.h \
  networklayer/manetrouting/olsr/OLSR_rtable.h \
  networklayer/manetrouting/olsr/OLSR_state.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/manetrouting/olsr/OLSR.h \
  base/NotifierConsts.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/olsr/OLSR_repositories.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/base/compatibility.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h
$O/networklayer/manetrouting/olsr/OLSR_state.o: networklayer/manetrouting/olsr/OLSR_state.cc \
  networklayer/manetrouting/base/compatibility.h \
  base/ModuleAccess.h \
  networklayer/manetrouting/olsr/OLSR_repositories.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/manetrouting/olsr/OLSR_state.h \
  networklayer/manetrouting/olsr/OLSR_rtable.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/manetrouting/olsr/OLSR.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/manetrouting/base/ManetRoutingBase.h \
  base/INETDefs.h \
  base/INotifiable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/ipv4/ICMP.h
$O/networklayer/manetrouting/olsr/OLSRpkt_m.o: networklayer/manetrouting/olsr/OLSRpkt_m.cc \
  networklayer/contract/IPAddress.h \
  networklayer/manetrouting/olsr/OLSRpkt_m.h \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/manetrouting/olsr/OLSR_ETX_parameter.h \
  networklayer/manetrouting/base/uint128.h
$O/networklayer/mpls/LIBTable.o: networklayer/mpls/LIBTable.cc \
  networklayer/mpls/LIBTable.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  base/INETDefs.h \
  util/XMLUtils.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/mpls/ConstType.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ipv4/IPDatagram.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h
$O/networklayer/mpls/MPLS.o: networklayer/mpls/MPLS.cc \
  transport/contract/TCPSocketMap.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  transport/contract/TCPCommand_m.h \
  base/INotifiable.h \
  networklayer/mpls/IClassifier.h \
  networklayer/mpls/MPLS.h \
  networklayer/mpls/LIBTable.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/rsvp_te/Utils.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  base/NotifierConsts.h \
  networklayer/ldp/LDPPacket_m.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  networklayer/ldp/LDP.h \
  base/NotificationBoard.h \
  networklayer/mpls/ConstType.h \
  networklayer/rsvp_te/IntServ_m.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6Address.h \
  networklayer/mpls/MPLSPacket.h \
  networklayer/contract/IPvXAddress.h \
  transport/contract/TCPSocket.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/rsvp_te/IntServ.h \
  base/ModuleAccess.h \
  networklayer/mpls/LIBTableAccess.h \
  transport/tcp/TCPSegment.h
$O/networklayer/mpls/MPLSPacket.o: networklayer/mpls/MPLSPacket.cc \
  networklayer/mpls/MPLSPacket.h \
  base/INETDefs.h
$O/networklayer/ospfv2/OSPFPacket_m.o: networklayer/ospfv2/OSPFPacket_m.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/OSPFPacket_m.h
$O/networklayer/ospfv2/OSPFRouting.o: networklayer/ospfv2/OSPFRouting.cc \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/ospfv2/OSPFRouting.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/contract/IPAddressResolver.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h
$O/networklayer/ospfv2/interface/OSPFInterface.o: networklayer/ospfv2/interface/OSPFInterface.cc \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h
$O/networklayer/ospfv2/interface/OSPFInterfaceState.o: networklayer/ospfv2/interface/OSPFInterfaceState.cc \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateNotDesignatedRouter.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDesignatedRouter.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateBackup.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h
$O/networklayer/ospfv2/interface/OSPFInterfaceStateBackup.o: networklayer/ospfv2/interface/OSPFInterfaceStateBackup.cc \
  networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateBackup.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h
$O/networklayer/ospfv2/interface/OSPFInterfaceStateDesignatedRouter.o: networklayer/ospfv2/interface/OSPFInterfaceStateDesignatedRouter.cc \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDesignatedRouter.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h
$O/networklayer/ospfv2/interface/OSPFInterfaceStateDown.o: networklayer/ospfv2/interface/OSPFInterfaceStateDown.cc \
  networklayer/ospfv2/interface/OSPFInterfaceStateWaiting.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateNotDesignatedRouter.h \
  networklayer/ospfv2/interface/OSPFInterfaceStatePointToPoint.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h
$O/networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.o: networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.cc \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  base/INETDefs.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/contract/IPAddress.h
$O/networklayer/ospfv2/interface/OSPFInterfaceStateNotDesignatedRouter.o: networklayer/ospfv2/interface/OSPFInterfaceStateNotDesignatedRouter.cc \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateNotDesignatedRouter.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h
$O/networklayer/ospfv2/interface/OSPFInterfaceStatePointToPoint.o: networklayer/ospfv2/interface/OSPFInterfaceStatePointToPoint.cc \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/interface/OSPFInterfaceStatePointToPoint.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h
$O/networklayer/ospfv2/interface/OSPFInterfaceStateWaiting.o: networklayer/ospfv2/interface/OSPFInterfaceStateWaiting.cc \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateLoopback.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/interface/OSPFInterfaceState.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateWaiting.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/interface/OSPFInterfaceStateDown.h
$O/networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.o: networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.cc \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h
$O/networklayer/ospfv2/messagehandler/HelloHandler.o: networklayer/ospfv2/messagehandler/HelloHandler.cc \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h
$O/networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.o: networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.cc \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h
$O/networklayer/ospfv2/messagehandler/LinkStateRequestHandler.o: networklayer/ospfv2/messagehandler/LinkStateRequestHandler.cc \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h
$O/networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.o: networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.cc \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h
$O/networklayer/ospfv2/messagehandler/MessageHandler.o: networklayer/ospfv2/messagehandler/MessageHandler.cc \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h
$O/networklayer/ospfv2/neighbor/OSPFNeighbor.o: networklayer/ospfv2/neighbor/OSPFNeighbor.cc \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborState.o: networklayer/ospfv2/neighbor/OSPFNeighborState.cc \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateAttempt.o: networklayer/ospfv2/neighbor/OSPFNeighborStateAttempt.cc \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateAttempt.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateDown.o: networklayer/ospfv2/neighbor/OSPFNeighborStateDown.cc \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateAttempt.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateExchange.o: networklayer/ospfv2/neighbor/OSPFNeighborStateExchange.cc \
  networklayer/ospfv2/neighbor/OSPFNeighborStateLoading.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchange.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateFull.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.o: networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.cc \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchange.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateFull.o: networklayer/ospfv2/neighbor/OSPFNeighborStateFull.cc \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateFull.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateInit.o: networklayer/ospfv2/neighbor/OSPFNeighborStateInit.cc \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateLoading.o: networklayer/ospfv2/neighbor/OSPFNeighborStateLoading.cc \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateFull.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateLoading.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h
$O/networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.o: networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.cc \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateTwoWay.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateExchangeStart.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ospfv2/neighbor/OSPFNeighborState.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateInit.h \
  networklayer/ospfv2/neighbor/OSPFNeighborStateDown.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h
$O/networklayer/ospfv2/router/ASExternalLSA.o: networklayer/ospfv2/router/ASExternalLSA.cc \
  base/INETDefs.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/router/LSA.h
$O/networklayer/ospfv2/router/NetworkLSA.o: networklayer/ospfv2/router/NetworkLSA.cc \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  base/INETDefs.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/OSPFPacket_m.h
$O/networklayer/ospfv2/router/OSPFArea.o: networklayer/ospfv2/router/OSPFArea.cc \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/OSPFTimer_m.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/common/InterfaceEntry.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/router/OSPFcommon.h
$O/networklayer/ospfv2/router/OSPFRouter.o: networklayer/ospfv2/router/OSPFRouter.cc \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/messagehandler/IMessageHandler.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ospfv2/neighbor/OSPFNeighbor.h \
  networklayer/ospfv2/router/OSPFRoutingTableEntry.h \
  networklayer/ospfv2/router/LSA.h \
  linklayer/contract/MACAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateAcknowledgementHandler.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/ospfv2/interface/OSPFInterface.h \
  base/NotifierConsts.h \
  networklayer/ospfv2/messagehandler/HelloHandler.h \
  networklayer/ospfv2/router/OSPFArea.h \
  networklayer/ospfv2/router/OSPFRouter.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/ospfv2/messagehandler/LinkStateRequestHandler.h \
  networklayer/ospfv2/messagehandler/MessageHandler.h \
  base/INETDefs.h \
  networklayer/ospfv2/messagehandler/DatabaseDescriptionHandler.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ospfv2/messagehandler/LinkStateUpdateHandler.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ospfv2/OSPFTimer_m.h
$O/networklayer/ospfv2/router/RouterLSA.o: networklayer/ospfv2/router/RouterLSA.cc \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/router/LSA.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/networklayer/ospfv2/router/SummaryLSA.o: networklayer/ospfv2/router/SummaryLSA.cc \
  networklayer/ospfv2/OSPFPacket_m.h \
  networklayer/ospfv2/router/LSA.h \
  base/INETDefs.h \
  networklayer/ospfv2/router/OSPFcommon.h \
  networklayer/contract/IPAddress.h
$O/networklayer/queue/BasicDSCPClassifier.o: networklayer/queue/BasicDSCPClassifier.cc \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/queue/IQoSClassifier.h \
  base/INETDefs.h \
  networklayer/queue/BasicDSCPClassifier.h \
  networklayer/ipv6/IPv6Datagram.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  networklayer/contract/IPv6Address.h
$O/networklayer/queue/DropTailQoSQueue.o: networklayer/queue/DropTailQoSQueue.cc \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  networklayer/queue/DropTailQoSQueue.h \
  base/INETDefs.h \
  networklayer/queue/IQoSClassifier.h
$O/networklayer/queue/DropTailQueue.o: networklayer/queue/DropTailQueue.cc \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  networklayer/queue/DropTailQueue.h \
  base/INETDefs.h
$O/networklayer/queue/REDQueue.o: networklayer/queue/REDQueue.cc \
  base/IPassiveQueue.h \
  base/PassiveQueueBase.h \
  networklayer/queue/REDQueue.h \
  base/INETDefs.h
$O/networklayer/queue/WeightedFairQueue.o: networklayer/queue/WeightedFairQueue.cc \
  base/INETDefs.h \
  networklayer/queue/IQoSClassifier.h \
  networklayer/queue/WeightedFairQueue.h \
  base/PassiveQueueBase.h \
  base/IPassiveQueue.h
$O/networklayer/rsvp_te/IntServ_m.o: networklayer/rsvp_te/IntServ_m.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  networklayer/rsvp_te/IntServ_m.h
$O/networklayer/rsvp_te/RSVP.o: networklayer/rsvp_te/RSVP.cc \
  networklayer/mpls/ConstType.h \
  networklayer/ted/TED.h \
  base/NotificationBoard.h \
  base/IScriptable.h \
  networklayer/rsvp_te/RSVP.h \
  networklayer/rsvp_te/SignallingMsg_m.h \
  networklayer/rsvp_te/RSVPResvMsg_m.h \
  networklayer/rsvp_te/IntServ_m.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/rsvp_te/RSVPPathMsg.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/mpls/LIBTableAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/contract/IPAddressResolver.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/rsvp_te/IRSVPClassifier.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/rsvp_te/RSVPPacket_m.h \
  util/common.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/mpls/LIBTable.h \
  networklayer/mpls/IClassifier.h \
  base/INotifiable.h \
  networklayer/rsvp_te/RSVPResvMsg.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/rsvp_te/RSVPPacket.h \
  networklayer/rsvp_te/RSVPHelloMsg.h \
  networklayer/rsvp_te/RSVPHello_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPAddress.h \
  networklayer/rsvp_te/RSVPPathMsg_m.h \
  networklayer/rsvp_te/Utils.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ted/TEDAccess.h \
  networklayer/ted/TED_m.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  util/XMLUtils.h
$O/networklayer/rsvp_te/SimpleClassifier.o: networklayer/rsvp_te/SimpleClassifier.cc \
  base/IScriptable.h \
  networklayer/rsvp_te/SignallingMsg_m.h \
  networklayer/rsvp_te/RSVPResvMsg_m.h \
  networklayer/rsvp_te/IntServ_m.h \
  networklayer/rsvp_te/RSVP.h \
  networklayer/mpls/ConstType.h \
  base/NotificationBoard.h \
  networklayer/rsvp_te/RSVPPathMsg.h \
  networklayer/ipv4/RoutingTableAccess.h \
  base/NotifierConsts.h \
  networklayer/rsvp_te/SimpleClassifier.h \
  networklayer/mpls/LIBTableAccess.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/rsvp_te/IRSVPClassifier.h \
  networklayer/rsvp_te/RSVPResvMsg.h \
  networklayer/mpls/IClassifier.h \
  base/INotifiable.h \
  networklayer/mpls/LIBTable.h \
  networklayer/rsvp_te/RSVPPacket_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/rsvp_te/RSVPHelloMsg.h \
  networklayer/rsvp_te/RSVPHello_m.h \
  networklayer/rsvp_te/RSVPPacket.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/rsvp_te/RSVPAccess.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/rsvp_te/RSVPPathMsg_m.h \
  networklayer/contract/IPAddress.h \
  util/XMLUtils.h \
  networklayer/ipv4/IRoutingTable.h \
  base/INETDefs.h
$O/networklayer/rsvp_te/Utils.o: networklayer/rsvp_te/Utils.cc \
  networklayer/rsvp_te/IntServ.h \
  networklayer/rsvp_te/IntServ_m.h \
  networklayer/rsvp_te/Utils.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h
$O/networklayer/ted/LinkStatePacket_m.o: networklayer/ted/LinkStatePacket_m.cc \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  networklayer/ted/TED_m.h \
  networklayer/ted/LinkStatePacket_m.h
$O/networklayer/ted/LinkStateRouting.o: networklayer/ted/LinkStateRouting.cc \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ted/TED_m.h \
  base/INETDefs.h \
  networklayer/ted/TEDAccess.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ted/LinkStatePacket_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/ted/LinkStateRouting.h \
  networklayer/ipv4/RoutingTableAccess.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  networklayer/rsvp_te/IntServ_m.h \
  base/NotificationBoard.h \
  networklayer/ted/TED.h
$O/networklayer/ted/TED.o: networklayer/ted/TED.cc \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/ted/TED.h \
  base/NotificationBoard.h \
  networklayer/rsvp_te/IntServ_m.h \
  base/NotifierConsts.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ted/TED_m.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h
$O/networklayer/ted/TED_m.o: networklayer/ted/TED_m.cc \
  base/INETDefs.h \
  networklayer/ted/TED_m.h \
  networklayer/contract/IPAddress.h
$O/transport/contract/SCTPCommand_m.o: transport/contract/SCTPCommand_m.cc \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  util/common.h \
  networklayer/contract/IPControlInfo.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  transport/sctp/SCTPMessage.h \
  transport/contract/SCTPCommand.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/contract/SCTPCommand_m.h \
  transport/sctp/SCTP.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h
$O/transport/contract/SCTPSocket.o: transport/contract/SCTPSocket.cc \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  transport/contract/SCTPSocket.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  transport/contract/SCTPCommand_m.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  transport/sctp/SCTPMessage.h \
  transport/contract/SCTPCommand.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  util/common.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h
$O/transport/contract/TCPCommand_m.o: transport/contract/TCPCommand_m.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/transport/contract/TCPSocket.o: transport/contract/TCPSocket.cc \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h
$O/transport/contract/TCPSocketMap.o: transport/contract/TCPSocketMap.cc \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/contract/TCPSocket.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  transport/contract/TCPCommand_m.h \
  transport/contract/TCPSocketMap.h
$O/transport/contract/UDPControlInfo_m.o: transport/contract/UDPControlInfo_m.cc \
  transport/contract/UDPControlInfo_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/transport/contract/UDPSocket.o: transport/contract/UDPSocket.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/contract/UDPSocket.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h
$O/transport/rtp/RTCP.o: transport/rtp/RTCP.cc \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/rtp/RTCPPacket.h \
  transport/rtp/RTPReceiverInfo.h \
  transport/contract/UDPControlInfo_m.h \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTPSenderStatusMessage_m.h \
  base/INETDefs.h \
  transport/rtp/RTPSenderInfo.h \
  networklayer/contract/IPAddress.h \
  transport/contract/UDPSocket.h \
  transport/rtp/sdes.h \
  transport/rtp/reports.h \
  transport/rtp/RTCP.h \
  transport/rtp/RTPInnerPacket.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  transport/rtp/RTPParticipantInfo.h
$O/transport/rtp/RTCPPacket.o: transport/rtp/RTCPPacket.cc \
  transport/rtp/RTCPPacket.h \
  transport/rtp/reports.h \
  transport/rtp/sdes.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h
$O/transport/rtp/RTP.o: transport/rtp/RTP.cc \
  transport/contract/UDPControlInfo_m.h \
  transport/rtp/RTP.h \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTPSenderStatusMessage_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  base/INETDefs.h \
  transport/rtp/RTPProfile.h \
  transport/rtp/RTPInterfacePacket.h \
  networklayer/contract/IPAddress.h \
  transport/contract/UDPSocket.h \
  transport/rtp/RTPInnerPacket.h
$O/transport/rtp/RTPInnerPacket.o: transport/rtp/RTPInnerPacket.cc \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/RTPInnerPacket.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  transport/rtp/RTPSenderStatusMessage_m.h \
  transport/rtp/RTPPacket.h
$O/transport/rtp/RTPInterfacePacket.o: transport/rtp/RTPInterfacePacket.cc \
  transport/rtp/RTPSenderStatusMessage_m.h \
  transport/rtp/RTPInterfacePacket.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  transport/rtp/RTPSenderControlMessage_m.h
$O/transport/rtp/RTPPacket.o: transport/rtp/RTPPacket.cc \
  base/INETDefs.h \
  transport/rtp/RTPPacket.h
$O/transport/rtp/RTPParticipantInfo.o: transport/rtp/RTPParticipantInfo.cc \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTCPPacket.h \
  transport/rtp/RTPParticipantInfo.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/reports.h \
  transport/rtp/sdes.h
$O/transport/rtp/RTPPayloadReceiver.o: transport/rtp/RTPPayloadReceiver.cc \
  transport/rtp/RTPPayloadReceiver.h \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTPSenderStatusMessage_m.h \
  base/INETDefs.h \
  transport/rtp/RTPInnerPacket.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/RTPSenderControlMessage_m.h
$O/transport/rtp/RTPPayloadSender.o: transport/rtp/RTPPayloadSender.cc \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTPSenderStatusMessage_m.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/RTPPayloadSender.h \
  transport/rtp/RTPInnerPacket.h \
  base/INETDefs.h
$O/transport/rtp/RTPProfile.o: transport/rtp/RTPProfile.cc \
  transport/rtp/RTPPayloadSender.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/sdes.h \
  transport/rtp/reports.h \
  transport/rtp/RTPInnerPacket.h \
  base/INETDefs.h \
  transport/rtp/RTPProfile.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  transport/rtp/RTPParticipantInfo.h \
  transport/rtp/RTCPPacket.h \
  transport/rtp/RTPSenderStatusMessage_m.h \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTPPayloadReceiver.h
$O/transport/rtp/RTPReceiverInfo.o: transport/rtp/RTPReceiverInfo.cc \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTCPPacket.h \
  transport/rtp/RTPReceiverInfo.h \
  transport/rtp/RTPParticipantInfo.h \
  base/INETDefs.h \
  transport/rtp/sdes.h \
  transport/rtp/reports.h \
  networklayer/contract/IPAddress.h
$O/transport/rtp/RTPSenderControlMessage_m.o: transport/rtp/RTPSenderControlMessage_m.cc \
  transport/rtp/RTPSenderControlMessage_m.h \
  base/INETDefs.h
$O/transport/rtp/RTPSenderInfo.o: transport/rtp/RTPSenderInfo.cc \
  transport/rtp/RTCPPacket.h \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTPSenderInfo.h \
  base/INETDefs.h \
  transport/rtp/sdes.h \
  transport/rtp/reports.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/RTPParticipantInfo.h
$O/transport/rtp/RTPSenderStatusMessage_m.o: transport/rtp/RTPSenderStatusMessage_m.cc \
  base/INETDefs.h \
  transport/rtp/RTPSenderStatusMessage_m.h
$O/transport/rtp/reports.o: transport/rtp/reports.cc \
  base/INETDefs.h \
  transport/rtp/reports.h \
  networklayer/contract/IPAddress.h
$O/transport/rtp/sdes.o: transport/rtp/sdes.cc \
  base/INETDefs.h \
  transport/rtp/sdes.h
$O/transport/rtp/profiles/avprofile/RTPAVProfile.o: transport/rtp/profiles/avprofile/RTPAVProfile.cc \
  transport/rtp/RTPSenderStatusMessage_m.h \
  transport/rtp/RTPPacket.h \
  transport/rtp/profiles/avprofile/RTPAVProfile.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  transport/rtp/RTPProfile.h \
  base/INETDefs.h \
  transport/rtp/RTPInnerPacket.h \
  networklayer/contract/IPAddress.h
$O/transport/rtp/profiles/avprofile/RTPAVProfilePayload32Receiver.o: transport/rtp/profiles/avprofile/RTPAVProfilePayload32Receiver.cc \
  transport/rtp/RTPPacket.h \
  transport/rtp/profiles/avprofile/RTPAVProfilePayload32Receiver.h \
  transport/rtp/RTPPayloadReceiver.h \
  transport/rtp/profiles/avprofile/RTPMpegPacket_m.h \
  base/INETDefs.h
$O/transport/rtp/profiles/avprofile/RTPAVProfilePayload32Sender.o: transport/rtp/profiles/avprofile/RTPAVProfilePayload32Sender.cc \
  transport/rtp/RTPPacket.h \
  transport/rtp/RTPSenderStatusMessage_m.h \
  transport/rtp/profiles/avprofile/RTPAVProfilePayload32Sender.h \
  transport/rtp/profiles/avprofile/RTPMpegPacket_m.h \
  transport/rtp/RTPSenderControlMessage_m.h \
  base/INETDefs.h \
  transport/rtp/RTPPayloadSender.h \
  networklayer/contract/IPAddress.h \
  transport/rtp/RTPInnerPacket.h
$O/transport/sctp/SCTP.o: transport/sctp/SCTP.cc \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  util/common.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/contract/IPProtocolId_m.h \
  transport/sctp/SCTPMessage.h \
  transport/contract/SCTPCommand.h \
  networklayer/ipv4/IPDatagram_m.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/rsvp_te/IntServ.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTP.h \
  transport/contract/SCTPCommand_m.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h
$O/transport/sctp/SCTPAlg.o: transport/sctp/SCTPAlg.cc \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  util/common.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  transport/sctp/SCTPMessage.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  transport/sctp/SCTPAlg.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTP.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/sctp/SCTPAlgorithm.h \
  linklayer/contract/MACAddress.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h
$O/transport/sctp/SCTPAssociationBase.o: transport/sctp/SCTPAssociationBase.cc \
  transport/contract/SCTPCommand.h \
  transport/sctp/SCTPMessage.h \
  networklayer/contract/IPProtocolId_m.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  util/common.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/common/InterfaceEntry.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  linklayer/contract/MACAddress.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/sctp/SCTPAlgorithm.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  transport/sctp/SCTP.h \
  transport/contract/SCTPCommand_m.h \
  networklayer/rsvp_te/IntServ_m.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h
$O/transport/sctp/SCTPAssociationEventProc.o: transport/sctp/SCTPAssociationEventProc.cc \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  networklayer/common/InterfaceEntry.h \
  transport/sctp/SCTPMessage.h \
  transport/contract/SCTPCommand.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  util/common.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/sctp/SCTPAlgorithm.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/contract/SCTPCommand_m.h \
  transport/sctp/SCTP.h
$O/transport/sctp/SCTPAssociationRcvMessage.o: transport/sctp/SCTPAssociationRcvMessage.cc \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  util/common.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  transport/contract/SCTPCommand.h \
  transport/sctp/SCTPMessage.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ipv6/IPv6InterfaceData.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/sctp/SCTPMessage_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  transport/sctp/SCTP.h \
  transport/contract/SCTPCommand_m.h \
  networklayer/rsvp_te/IntServ_m.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  transport/sctp/SCTPAlgorithm.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h
$O/transport/sctp/SCTPAssociationSendAll.o: transport/sctp/SCTPAssociationSendAll.cc \
  transport/sctp/SCTPMessage.h \
  transport/contract/SCTPCommand.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/contract/IPProtocolId_m.h \
  util/common.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  transport/sctp/SCTPAlgorithm.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/contract/SCTPCommand_m.h \
  transport/sctp/SCTP.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/common/InterfaceTable.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h
$O/transport/sctp/SCTPAssociationUtil.o: transport/sctp/SCTPAssociationUtil.cc \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  transport/contract/SCTPCommand.h \
  transport/sctp/SCTPMessage.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  util/common.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ipv6/IPv6InterfaceData.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  linklayer/contract/MACAddress.h \
  transport/sctp/SCTPAlgorithm.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/ipv4/IPv4InterfaceData.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  transport/contract/SCTPCommand_m.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h
$O/transport/sctp/SCTPCCFunctions.o: transport/sctp/SCTPCCFunctions.cc \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/sctp/SCTPMessage_m.h \
  util/common.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  transport/sctp/SCTPMessage.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h
$O/transport/sctp/SCTPMessage.o: transport/sctp/SCTPMessage.cc \
  util/common.h \
  base/INotifiable.h \
  networklayer/common/InterfaceTableAccess.h \
  transport/sctp/SCTPMessage.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  base/NotificationBoard.h \
  transport/sctp/SCTPSendStream.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTP.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/common/InterfaceTable.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/common/InterfaceToken.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h
$O/transport/sctp/SCTPMessage_m.o: transport/sctp/SCTPMessage_m.cc \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/transport/sctp/SCTPQueue.o: transport/sctp/SCTPQueue.cc \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  transport/sctp/SCTPMessage.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  util/common.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h
$O/transport/sctp/SCTPReceiveStream.o: transport/sctp/SCTPReceiveStream.cc \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPMessage.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  util/common.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/common/InterfaceTable.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h
$O/transport/sctp/SCTPSSFunctions.o: transport/sctp/SCTPSSFunctions.cc \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  base/NotifierConsts.h \
  transport/sctp/SCTPQueue.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPControlInfo_m.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/sctp/SCTPMessage_m.h \
  util/common.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  transport/sctp/SCTPMessage.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPAddress.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/sctp/SCTPAssociation.h
$O/transport/sctp/SCTPSendStream.o: transport/sctp/SCTPSendStream.cc \
  transport/contract/UDPSocket.h \
  networklayer/contract/IPAddress.h \
  transport/sctp/SCTPAssociation.h \
  base/INETDefs.h \
  transport/sctp/SCTP.h \
  transport/sctp/SCTPSendStream.h \
  transport/contract/UDPControlInfo_m.h \
  transport/sctp/SCTPQueue.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/transport/tcp/TCP.o: transport/tcp/TCP.cc \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/tcp/TCP.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPControlInfo_m.h \
  transport/tcp/TCPSegment.h \
  linklayer/contract/MACAddress.h \
  transport/tcp/TCPConnection.h
$O/transport/tcp/TCPConnectionBase.o: transport/tcp/TCPConnectionBase.cc \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/TCPSendQueue.h \
  transport/tcp/TCPConnection.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  transport/contract/TCPCommand_m.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPSACKRexmitQueue.h \
  transport/tcp/TCPReceiveQueue.h
$O/transport/tcp/TCPConnectionEventProc.o: transport/tcp/TCPConnectionEventProc.cc \
  transport/tcp/TCPAlgorithm.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCPSendQueue.h \
  transport/tcp/TCPConnection.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPReceiveQueue.h \
  transport/contract/TCPCommand_m.h
$O/transport/tcp/TCPConnectionRcvSegment.o: transport/tcp/TCPConnectionRcvSegment.cc \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/contract/TCPCommand_m.h \
  transport/tcp/TCPReceiveQueue.h \
  transport/tcp/TCPSACKRexmitQueue.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCPAlgorithm.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/TCPConnection.h \
  transport/tcp/TCPSendQueue.h
$O/transport/tcp/TCPConnectionUtil.o: transport/tcp/TCPConnectionUtil.cc \
  networklayer/contract/IPControlInfo_m.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCPAlgorithm.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/TCPConnection.h \
  linklayer/contract/MACAddress.h \
  transport/tcp/TCPSendQueue.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  transport/tcp/TCPSegment_m.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/tcp/TCPSACKRexmitQueue.h \
  transport/tcp/TCP.h \
  networklayer/contract/IPProtocolId_m.h \
  transport/tcp/TCPReceiveQueue.h
$O/transport/tcp/TCPSACKRexmitQueue.o: transport/tcp/TCPSACKRexmitQueue.cc \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp/TCPSACKRexmitQueue.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/TCPConnection.h
$O/transport/tcp/TCPSegment.o: transport/tcp/TCPSegment.cc \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp/TCPSegment_m.o: transport/tcp/TCPSegment_m.cc \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h
$O/transport/tcp/flavours/DumbTCP.o: transport/tcp/flavours/DumbTCP.cc \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/TCPConnection.h \
  transport/tcp/flavours/DumbTCP.h
$O/transport/tcp/flavours/TCPBaseAlg.o: transport/tcp/flavours/TCPBaseAlg.cc \
  transport/tcp/TCPConnection.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/flavours/TCPBaseAlg.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPSACKRexmitQueue.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPAddress.h
$O/transport/tcp/flavours/TCPNewReno.o: transport/tcp/flavours/TCPNewReno.cc \
  transport/tcp/TCPConnection.h \
  transport/tcp/flavours/TCPTahoeRenoFamily.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/flavours/TCPBaseAlg.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCP.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp/flavours/TCPNewReno.h \
  networklayer/contract/IPAddress.h
$O/transport/tcp/flavours/TCPNoCongestionControl.o: transport/tcp/flavours/TCPNoCongestionControl.cc \
  transport/tcp/TCPConnection.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/flavours/TCPBaseAlg.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/flavours/TCPNoCongestionControl.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/transport/tcp/flavours/TCPReno.o: transport/tcp/flavours/TCPReno.cc \
  transport/tcp/TCPConnection.h \
  transport/tcp/flavours/TCPTahoeRenoFamily.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/flavours/TCPBaseAlg.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCP.h \
  transport/tcp/flavours/TCPReno.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/transport/tcp/flavours/TCPTahoe.o: transport/tcp/flavours/TCPTahoe.cc \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/flavours/TCPBaseAlg.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/flavours/TCPTahoeRenoFamily.h \
  transport/tcp/TCPConnection.h \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp/flavours/TCPTahoe.h \
  transport/tcp/TCP.h
$O/transport/tcp/flavours/TCPTahoeRenoFamily.o: transport/tcp/flavours/TCPTahoeRenoFamily.cc \
  transport/tcp/flavours/TCPTahoeRenoFamily.h \
  transport/tcp/TCPConnection.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/flavours/TCPBaseAlg.h \
  transport/tcp/TCPAlgorithm.h \
  transport/tcp/TCP.h \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h
$O/transport/tcp/queues/TCPMsgBasedRcvQueue.o: transport/tcp/queues/TCPMsgBasedRcvQueue.cc \
  transport/tcp/TCPReceiveQueue.h \
  transport/tcp/TCP.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPConnection.h \
  transport/tcp/queues/TCPMsgBasedRcvQueue.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/queues/TCPVirtualDataRcvQueue.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp/queues/TCPMsgBasedSendQueue.o: transport/tcp/queues/TCPMsgBasedSendQueue.cc \
  transport/tcp/TCPSendQueue.h \
  transport/tcp/TCPConnection.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/queues/TCPMsgBasedSendQueue.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCP.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPAddress.h
$O/transport/tcp/queues/TCPVirtualDataRcvQueue.o: transport/tcp/queues/TCPVirtualDataRcvQueue.cc \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPReceiveQueue.h \
  transport/tcp/TCP.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/queues/TCPVirtualDataRcvQueue.h \
  transport/tcp/TCPSegment.h \
  transport/tcp/TCPConnection.h
$O/transport/tcp/queues/TCPVirtualDataSendQueue.o: transport/tcp/queues/TCPVirtualDataSendQueue.cc \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/TCPSendQueue.h \
  transport/tcp/TCPConnection.h \
  networklayer/contract/IPAddress.h \
  transport/tcp/queues/TCPVirtualDataSendQueue.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp/TCP.h
$O/transport/tcp_nsc/TCP_NSC.o: transport/tcp_nsc/TCP_NSC.cc \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPProtocolId_m.h \
  util/headerserializers/headers/tcp.h \
  transport/tcp_nsc/TCP_NSC.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  util/headerserializers/headers/defs.h \
  util/headerserializers/TCPSerializer.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_nsc/TCP_NSC_Connection.h \
  linklayer/contract/MACAddress.h \
  transport/tcp_nsc/queues/TCP_NSC_Queues.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPControlInfo_m.h \
  util/headerserializers/TCPIPchecksum.h
$O/transport/tcp_nsc/TCP_NSC_Connection.o: transport/tcp_nsc/TCP_NSC_Connection.cc \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  util/headerserializers/headers/defs.h \
  util/headerserializers/TCPSerializer.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_nsc/TCP_NSC.h \
  util/headerserializers/headers/tcp.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  util/headerserializers/TCPIPchecksum.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  transport/tcp_nsc/TCP_NSC_Connection.h \
  transport/tcp_nsc/queues/TCP_NSC_Queues.h
$O/transport/tcp_nsc/queues/TCP_NSC_VirtualDataQueues.o: transport/tcp_nsc/queues/TCP_NSC_VirtualDataQueues.cc \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  util/headerserializers/TCPSerializer.h \
  util/headerserializers/headers/defs.h \
  transport/tcp/TCPSegment_m.h \
  transport/contract/TCPCommand_m.h \
  util/headerserializers/headers/tcp.h \
  transport/tcp/TCP.h \
  transport/tcp_nsc/queues/TCP_NSC_VirtualDataQueues.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp_nsc/queues/TCP_NSC_Queues.h \
  transport/tcp_nsc/TCP_NSC_Connection.h \
  transport/tcp/TCPConnection.h
$O/transport/tcp_old/TCP.o: transport/tcp_old/TCP.cc \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  networklayer/contract/IPv6ControlInfo.h \
  transport/tcp_old/TCPConnection_old.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/contract/IPControlInfo_m.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  linklayer/contract/MACAddress.h
$O/transport/tcp_old/TCPConnectionBase.o: transport/tcp_old/TCPConnectionBase.cc \
  transport/tcp_old/TCPConnection_old.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  transport/tcp_old/TCPReceiveQueue_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPSendQueue_old.h \
  transport/contract/TCPCommand_m.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp_old/TCPConnectionEventProc.o: transport/tcp_old/TCPConnectionEventProc.cc \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/contract/TCPCommand_m.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  transport/tcp_old/TCPSendQueue_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPReceiveQueue_old.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  transport/tcp_old/TCPConnection_old.h
$O/transport/tcp_old/TCPConnectionRcvSegment.o: transport/tcp_old/TCPConnectionRcvSegment.cc \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp_old/TCP_old.h \
  transport/tcp_old/TCPConnection_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPReceiveQueue_old.h \
  transport/tcp_old/TCPSendQueue_old.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp_old/TCPConnectionUtil.o: transport/tcp_old/TCPConnectionUtil.cc \
  linklayer/contract/MACAddress.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  transport/contract/TCPCommand_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  transport/tcp_old/TCPSendQueue_old.h \
  transport/tcp_old/TCPReceiveQueue_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPConnection_old.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  transport/tcp_old/TCP_old.h
$O/transport/tcp_old/flavours/DumbTCP.o: transport/tcp_old/flavours/DumbTCP.cc \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp_old/flavours/DumbTCP_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPConnection_old.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/TCPAlgorithm_old.h
$O/transport/tcp_old/flavours/TCPBaseAlg.o: transport/tcp_old/flavours/TCPBaseAlg.cc \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPConnection_old.h \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp_old/TCP_old.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  transport/tcp_old/flavours/TCPBaseAlg_old.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h
$O/transport/tcp_old/flavours/TCPNoCongestionControl.o: transport/tcp_old/flavours/TCPNoCongestionControl.cc \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  transport/tcp_old/TCPConnection_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  transport/tcp_old/flavours/TCPBaseAlg_old.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp_old/flavours/TCPNoCongestionControl_old.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp_old/flavours/TCPReno.o: transport/tcp_old/flavours/TCPReno.cc \
  base/INETDefs.h \
  transport/tcp_old/TCP_old.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/flavours/TCPReno_old.h \
  transport/tcp_old/TCPConnection_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/flavours/TCPBaseAlg_old.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp_old/flavours/TCPTahoeRenoFamily_old.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp_old/flavours/TCPTahoe.o: transport/tcp_old/flavours/TCPTahoe.cc \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp_old/TCP_old.h \
  transport/tcp_old/TCPConnection_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/flavours/TCPTahoe_old.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  transport/tcp_old/flavours/TCPBaseAlg_old.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp_old/flavours/TCPTahoeRenoFamily_old.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp_old/flavours/TCPTahoeRenoFamily.o: transport/tcp_old/flavours/TCPTahoeRenoFamily.cc \
  transport/tcp_old/flavours/TCPTahoeRenoFamily_old.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp_old/flavours/TCPBaseAlg_old.h \
  transport/tcp_old/TCPAlgorithm_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPConnection_old.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h
$O/transport/tcp_old/queues/TCPMsgBasedRcvQueue.o: transport/tcp_old/queues/TCPMsgBasedRcvQueue.cc \
  transport/tcp_old/TCPReceiveQueue_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPConnection_old.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  transport/tcp_old/queues/TCPMsgBasedRcvQueue_old.h \
  transport/tcp/TCPSegment.h \
  transport/tcp_old/queues/TCPVirtualDataRcvQueue_old.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h
$O/transport/tcp_old/queues/TCPMsgBasedSendQueue.o: transport/tcp_old/queues/TCPMsgBasedSendQueue.cc \
  transport/tcp_old/queues/TCPMsgBasedSendQueue_old.h \
  transport/tcp/TCPSegment.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp_old/TCPSendQueue_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPConnection_old.h \
  transport/tcp_old/TCP_old.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h
$O/transport/tcp_old/queues/TCPVirtualDataRcvQueue.o: transport/tcp_old/queues/TCPVirtualDataRcvQueue.cc \
  transport/tcp/TCPSegment_m.h \
  base/INETDefs.h \
  transport/tcp_old/TCP_old.h \
  transport/tcp_old/TCPConnection_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/TCPReceiveQueue_old.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  transport/tcp_old/queues/TCPVirtualDataRcvQueue_old.h \
  transport/tcp/TCPSegment.h
$O/transport/tcp_old/queues/TCPVirtualDataSendQueue.o: transport/tcp_old/queues/TCPVirtualDataSendQueue.cc \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/TCPSegment.h \
  transport/tcp_old/TCPSendQueue_old.h \
  base/INETDefs.h \
  transport/tcp_old/TCP_old.h \
  transport/tcp/TCPSegment_m.h \
  transport/tcp_old/TCPConnection_old.h \
  networklayer/contract/IPAddress.h \
  transport/tcp_old/queues/TCPVirtualDataSendQueue_old.h
$O/transport/udp/UDP.o: transport/udp/UDP.cc \
  base/NotificationBoard.h \
  networklayer/ipv6/IPv6Datagram.h \
  base/NotifierConsts.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  linklayer/contract/MACAddress.h \
  transport/udp/UDP.h \
  base/ModuleAccess.h \
  networklayer/contract/IPControlInfo_m.h \
  networklayer/ipv6/RoutingTable6.h \
  transport/udp/UDPPacket_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  networklayer/contract/IPv6ControlInfo_m.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/icmpv6/ICMPv6Message_m.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  base/INotifiable.h \
  networklayer/icmpv6/ICMPv6.h \
  networklayer/ipv4/ICMP.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv6/RoutingTable6Access.h \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/udp/UDPPacket.h \
  networklayer/icmpv6/ICMPv6Access.h \
  networklayer/contract/IPv6ControlInfo.h \
  base/INETDefs.h \
  networklayer/ipv4/ICMPAccess.h \
  networklayer/ipv4/IRoutingTable.h
$O/transport/udp/UDPPacket_m.o: transport/udp/UDPPacket_m.cc \
  transport/udp/UDPPacket_m.h \
  base/INETDefs.h
$O/util/NAMTraceWriter.o: util/NAMTraceWriter.cc \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  util/NAMTraceWriter.h \
  base/NotifierConsts.h \
  world/NAMTrace.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceToken.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  networklayer/common/IInterfaceTable.h \
  networklayer/common/InterfaceEntry.h \
  linklayer/contract/TxNotifDetails.h \
  base/INETDefs.h
$O/util/NetAnimTrace.o: util/NetAnimTrace.cc \
  util/NetAnimTrace.h \
  base/INETDefs.h
$O/util/PowerControlManager.o: util/PowerControlManager.cc \
  base/IPowerControl.h \
  util/PowerControlManager.h \
  util/PowerControlMessages_m.h
$O/util/PowerControlMessages_m.o: util/PowerControlMessages_m.cc \
  util/PowerControlMessages_m.h
$O/util/TCPDump.o: util/TCPDump.cc \
  networklayer/common/InterfaceEntry.h \
  networklayer/ipv4/IPRoute.h \
  transport/contract/UDPSocket.h \
  networklayer/contract/IPAddress.h \
  networklayer/ipv4/ICMPMessage_m.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/ipv4/RoutingTable.h \
  base/INETDefs.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  util/common.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  networklayer/ipv4/IPDatagram_m.h \
  transport/sctp/SCTPMessage.h \
  networklayer/contract/IPControlInfo_m.h \
  transport/tcp/TCPSegment.h \
  util/TCPDump.h \
  base/ModuleAccess.h \
  util/headerserializers/IPSerializer.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/ipv4/IPDatagram.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv6/IPv6Datagram_m.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/ipv4/ICMPMessage.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  transport/udp/UDPPacket_m.h \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h
$O/util/ThruputMeteringChannel.o: util/ThruputMeteringChannel.cc \
  util/ThruputMeteringChannel.h \
  base/INETDefs.h
$O/util/XMLUtils.o: util/XMLUtils.cc \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPAddressResolver.h \
  util/XMLUtils.h \
  base/INETDefs.h \
  networklayer/contract/IPAddress.h
$O/util/common.o: util/common.cc \
  networklayer/rsvp_te/IntServ_m.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  util/common.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h
$O/util/dijktraKShortest.o: util/dijktraKShortest.cc \
  util/dijktraKShortest.h \
  networklayer/contract/IPAddress.h \
  base/INETDefs.h \
  linklayer/contract/MACAddress.h \
  networklayer/manetrouting/base/uint128.h \
  networklayer/contract/IPv6Address.h
$O/util/opp_utils.o: util/opp_utils.cc \
  util/opp_utils.h \
  base/INETDefs.h
$O/util/headerserializers/ICMPSerializer.o: util/headerserializers/ICMPSerializer.cc \
  util/headerserializers/headers/defs.h \
  base/INETDefs.h \
  applications/pingapp/PingPayload_m.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  util/headerserializers/headers/tcp.h \
  util/headerserializers/headers/bsdint.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPProtocolId_m.h \
  util/headerserializers/headers/in_systm.h \
  util/headerserializers/headers/ip.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h \
  networklayer/ipv4/IPDatagram.h \
  util/headerserializers/TCPIPchecksum.h \
  util/headerserializers/IPSerializer.h \
  util/headerserializers/ICMPSerializer.h \
  util/headerserializers/headers/in.h \
  util/headerserializers/headers/ip_icmp.h \
  transport/tcp/TCPSegment.h
$O/util/headerserializers/IPSerializer.o: util/headerserializers/IPSerializer.cc \
  util/headerserializers/headers/in_systm.h \
  util/headerserializers/headers/bsdint.h \
  util/headerserializers/headers/tcp.h \
  transport/sctp/SCTPMessage.h \
  networklayer/ipv4/IPDatagram_m.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/ipv4/ICMPMessage_m.h \
  networklayer/contract/IPAddress.h \
  transport/udp/UDPPacket.h \
  util/headerserializers/UDPSerializer.h \
  base/INETDefs.h \
  util/headerserializers/TCPSerializer.h \
  util/headerserializers/headers/defs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/ipv4/IPDatagram.h \
  util/headerserializers/IPSerializer.h \
  transport/tcp/TCPSegment.h \
  util/headerserializers/ICMPSerializer.h \
  util/headerserializers/headers/in.h \
  transport/udp/UDPPacket_m.h \
  util/headerserializers/headers/ip.h \
  transport/sctp/SCTPMessage_m.h \
  util/headerserializers/SCTPSerializer.h \
  networklayer/contract/IPvXAddress.h \
  networklayer/contract/IPv6Address.h
$O/util/headerserializers/SCTPSerializer.o: util/headerserializers/SCTPSerializer.cc \
  transport/sctp/SCTP.h \
  networklayer/rsvp_te/IntServ_m.h \
  transport/sctp/SCTPSendStream.h \
  base/NotificationBoard.h \
  linklayer/contract/MACAddress.h \
  networklayer/ipv4/RoutingTableAccess.h \
  transport/contract/UDPControlInfo_m.h \
  transport/sctp/SCTPQueue.h \
  base/NotifierConsts.h \
  networklayer/contract/IPControlInfo_m.h \
  util/headerserializers/headers/in.h \
  base/ModuleAccess.h \
  networklayer/common/InterfaceTable.h \
  networklayer/rsvp_te/IntServ.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  util/headerserializers/SCTPSerializer.h \
  transport/sctp/SCTPMessage_m.h \
  networklayer/common/InterfaceToken.h \
  networklayer/ipv4/IPRouteRule.h \
  util/headerserializers/headers/ip.h \
  util/headerserializers/headers/sctp.h \
  util/headerserializers/headers/in_systm.h \
  networklayer/common/InterfaceTableAccess.h \
  base/INotifiable.h \
  util/common.h \
  networklayer/contract/IPProtocolId_m.h \
  networklayer/contract/IPControlInfo.h \
  networklayer/common/IInterfaceTable.h \
  transport/sctp/SCTPReceiveStream.h \
  transport/sctp/SCTPMessage.h \
  util/headerserializers/headers/bsdint.h \
  networklayer/common/InterfaceEntry.h \
  transport/contract/UDPSocket.h \
  networklayer/ipv4/IPRoute.h \
  networklayer/contract/IPAddress.h \
  transport/sctp/SCTPAssociation.h \
  networklayer/ipv4/IRoutingTable.h \
  util/headerserializers/headers/defs.h \
  base/INETDefs.h \
  networklayer/ipv4/RoutingTable.h
$O/util/headerserializers/TCPIPchecksum.o: util/headerserializers/TCPIPchecksum.cc \
  util/headerserializers/headers/defs.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPAddress.h \
  util/headerserializers/headers/tcp.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/tcp/TCPSegment.h \
  util/headerserializers/TCPIPchecksum.h
$O/util/headerserializers/TCPSerializer.o: util/headerserializers/TCPSerializer.cc \
  util/headerserializers/TCPSerializer.h \
  util/headerserializers/headers/defs.h \
  base/INETDefs.h \
  transport/tcp/TCPSegment_m.h \
  networklayer/contract/IPAddress.h \
  networklayer/contract/IPProtocolId_m.h \
  util/headerserializers/headers/tcp.h \
  util/headerserializers/headers/bsdint.h \
  util/headerserializers/headers/in_systm.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  util/headerserializers/headers/in.h \
  transport/tcp/TCPSegment.h \
  util/headerserializers/TCPIPchecksum.h
$O/util/headerserializers/UDPSerializer.o: util/headerserializers/UDPSerializer.cc \
  util/headerserializers/UDPSerializer.h \
  transport/udp/UDPPacket.h \
  networklayer/contract/IPAddress.h \
  transport/tcp/TCPSegment_m.h \
  util/headerserializers/headers/defs.h \
  base/INETDefs.h \
  util/headerserializers/headers/in_systm.h \
  util/headerserializers/headers/tcp.h \
  util/headerserializers/headers/bsdint.h \
  util/headerserializers/headers/in.h \
  transport/tcp/TCPSegment.h \
  util/headerserializers/headers/udp.h \
  util/headerserializers/TCPIPchecksum.h \
  networklayer/contract/IPv6Address.h \
  networklayer/contract/IPvXAddress.h \
  transport/udp/UDPPacket_m.h
$O/world/ChannelAccess.o: world/ChannelAccess.cc \
  base/INotifiable.h \
  world/ChannelAccess.h \
  base/Coord.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  base/BasicModule.h \
  util/FWMath.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  world/ChannelControl.h \
  base/NotificationBoard.h \
  base/ModuleAccess.h
$O/world/ChannelAccessExtended.o: world/ChannelAccessExtended.cc \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  world/ChannelControlExtended.h \
  util/FWMath.h \
  world/ChannelAccessExtended.h \
  base/BasicModule.h \
  base/Coord.h \
  world/ChannelAccess.h \
  base/INotifiable.h \
  base/ModuleAccess.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotifierConsts.h \
  base/NotificationBoard.h \
  world/ChannelControl.h
$O/world/ChannelControl.o: world/ChannelControl.cc \
  linklayer/mfcore/AirFrame_m.h \
  world/ChannelControl.h \
  base/Coord.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  util/FWMath.h
$O/world/ChannelControlExtended.o: world/ChannelControlExtended.cc \
  world/ChannelControl.h \
  linklayer/mfcore/AirFrame_m.h \
  util/FWMath.h \
  world/ChannelControlExtended.h \
  linklayer/mfcore/ModulationType.h \
  base/INETDefs.h \
  base/Coord.h
$O/world/ChannelInstaller.o: world/ChannelInstaller.cc \
  world/ChannelInstaller.h \
  base/INETDefs.h
$O/world/NAMTrace.o: world/NAMTrace.cc \
  world/NAMTrace.h \
  base/INETDefs.h
$O/world/ScenarioManager.o: world/ScenarioManager.cc \
  world/ScenarioManager.h \
  base/IScriptable.h \
  base/INETDefs.h
$O/world/annotations/AnnotationDummy.o: world/annotations/AnnotationDummy.cc \
  base/INETDefs.h \
  ./world/annotations/AnnotationDummy.h
$O/world/annotations/AnnotationManager.o: world/annotations/AnnotationManager.cc \
  base/ModuleAccess.h \
  ./world/annotations/AnnotationManager.h \
  base/Coord.h \
  util/FWMath.h \
  base/INETDefs.h
$O/world/obstacles/Obstacle.o: world/obstacles/Obstacle.cc \
  ./world/annotations/AnnotationManager.h \
  base/ModuleAccess.h \
  ./world/obstacles/Obstacle.h \
  base/Coord.h \
  base/INETDefs.h \
  util/FWMath.h
$O/world/obstacles/ObstacleControl.o: world/obstacles/ObstacleControl.cc \
  ./world/annotations/AnnotationManager.h \
  ./world/obstacles/ObstacleControl.h \
  base/ModuleAccess.h \
  base/INETDefs.h \
  util/FWMath.h \
  ./world/obstacles/Obstacle.h \
  base/Coord.h
$O/world/shadowing/DiffractionPropagationLossModel.o: world/shadowing/DiffractionPropagationLossModel.cc \
  world/shadowing/IShadowingModel.h \
  world/shadowing/MapManager.h \
  base/Coord.h \
  base/INETDefs.h \
  util/FWMath.h \
  world/shadowing/DiffractionPropagationLossModel.h
$O/world/shadowing/MapManager.o: world/shadowing/MapManager.cc \
  world/shadowing/MapManager.h \
  base/Coord.h \
  base/INETDefs.h \
  util/FWMath.h
$O/world/shadowing/PenetrationPropagationLossModel.o: world/shadowing/PenetrationPropagationLossModel.cc \
  base/Coord.h \
  world/shadowing/MapManager.h \
  world/shadowing/IShadowingModel.h \
  util/FWMath.h \
  base/INETDefs.h \
  world/shadowing/PenetrationPropagationLossModel.h
$O/world/shadowing/ShadowPropagationLossModel.o: world/shadowing/ShadowPropagationLossModel.cc \
  util/FWMath.h \
  world/shadowing/ShadowPropagationLossModel.h \
  base/INETDefs.h \
  base/Coord.h \
  world/shadowing/MapManager.h \
  world/shadowing/IShadowingModel.h
$O/world/traci/TraCIScenarioManager.o: world/traci/TraCIScenarioManager.cc \
  ./world/traci/TraCIConstants.h \
  base/ModuleAccess.h \
  base/NotifierConsts.h \
  linklayer/mfcore/AirFrame_m.h \
  base/NotificationBoard.h \
  mobility/BasicMobility.h \
  world/ChannelControl.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  util/FWMath.h \
  base/BasicModule.h \
  ./world/traci/TraCIScenarioManager.h \
  base/Coord.h \
  base/INotifiable.h \
  ./mobility/traci/TraCIMobility.h
$O/world/traci/TraCIScenarioManagerLaunchd.o: world/traci/TraCIScenarioManagerLaunchd.cc \
  world/ChannelControl.h \
  linklayer/mfcore/AirFrame_m.h \
  base/ModuleAccess.h \
  ./world/traci/TraCIConstants.h \
  base/Coord.h \
  util/FWMath.h \
  ./world/traci/TraCIScenarioManager.h \
  base/INETDefs.h \
  linklayer/mfcore/ModulationType.h \
  ./world/traci/TraCIScenarioManagerLaunchd.h
