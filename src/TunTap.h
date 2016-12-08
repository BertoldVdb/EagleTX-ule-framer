/*
 * Copyright (c) 2016, Bertold Van den Bergh, Yuri Murillo Mange
 * All rights reserved.
 * This work has been developed to support research funded by
 * "Fund for Scientific Research, Flanders" (F.W.O.-Vlaanderen).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR DISTRIBUTOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SRC_TUNTAP_H_
#define SRC_TUNTAP_H_

#include <string>
#include <cstdint>
#include <vector>
#include <sys/socket.h>
#include <tuple>

class TunTap
{
public:
    enum PacketType { UNKNOWN, UNICAST_LOCAL, UNICAST_REMOTE, MULTICAST, BROADCAST, UNICAST_MYADDR };

    TunTap(bool tap);
    TunTap(std::string name, bool tap);
    ~TunTap();

    PacketType ipIsLocal(struct sockaddr& sockaddr);
    void printIfStats(std::ostream& cout);

    int readPacket(uint8_t* buffer, int bufferLen, PacketType& packetTypeOut);
    int writePacket(uint8_t* buffer, int bufferLen);

    int getFD() {
        return tunfd_;
    }

    bool setMTU(unsigned long mtu);
    unsigned long getMTU();

    void doBookkeeping();

private:
    bool updateNetinfo();
    void cleanup();

    int tunfd_ = -1;
    std::string ifname_;
    bool tap_;
    bool validAddressV4_ = false;
    bool validAddressV6_ = false;
    bool validAddressTAP_ = false;

    uint8_t networkMAC_[6];
    std::vector<std::tuple<struct sockaddr_storage,struct sockaddr_storage>> networkAddrs_;

    struct timeval lastAction_;

    unsigned long packetsReceived = 0;
    unsigned long packetsSent = 0;
    unsigned long packetsUnicastLinkLocal = 0;
    unsigned long packetsUnicastRemote = 0;
    unsigned long packetsMulticast = 0;
    unsigned long packetsBroadcast = 0;
    unsigned long packetsUnicastToMe = 0;
    unsigned long packetsUnknown = 0;

    unsigned long mtu_;
};



#endif /* SRC_TUNTAP_H_ */
