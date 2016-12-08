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


#ifndef SRC_ULEFRAMER_H_
#define SRC_ULEFRAMER_H_

#include <cstdint>
#include <unordered_map>
#include "PacketBuffer.h"
#include "Util.h"

class ULEFramer
{
public:
    ULEFramer(PacketBuffer& packetBuffer, unsigned long mtu, uint16_t pid, bool sendNULL):
        packetBuffer_(packetBuffer),
        mtu_(mtu),
        pid_(pid),
        framerState_(FRAMER_WAITBEGIN),
        sendNULL_(sendNULL) {

        pid_ &= 0x1FFF;

        /* Worst case: header + mtu + dest addr + CRC */
        mtu += sizeof(struct ULEHeader) + 6 + 4;
        localBuffer_ = new uint8_t[mtu]();
    }

    void clearDesintationAddressList() {
        targetMacsIP_.clear();
    }

    bool AddDestinationAddress(struct sockaddr_storage& dest, uint8_t mac[6]) {
        std::array<uint8_t, 6> macA;
        std::array<uint8_t, 17> ip;

        memset(ip.data(), 0, 17);
        memcpy(macA.data(), mac, 6);

        if(dest.ss_family == AF_INET) {
            struct sockaddr_in& dest4 = reinterpret_cast<struct sockaddr_in&>(dest);
            ip[0] = 4;
            memcpy(ip.data()+1, &dest4.sin_addr, 4);

        } else if(dest.ss_family == AF_INET6) {
            struct sockaddr_in6& dest6 = reinterpret_cast<struct sockaddr_in6&>(dest);
            ip[0] = 6;
            memcpy(ip.data()+1, &dest6.sin6_addr, 16);

        } else {
            return false;
        }

        targetMacsIP_[ip] = macA;

        return true;
    }

    void setGatewayMac(uint8_t* gatewayMac) {
        gatewayMacValid_ = true;
        memcpy(gatewayMac_, gatewayMac, 6);
    }

    /* Buffer should be 188 bytes and will get filled with one TS frame */
    int getMpegPacket(uint8_t* buffer, uint8_t frameLen);

private:
    int prepareULEPacket();


    struct  __attribute__((packed, aligned(1))) ULEHeader {
        uint16_t payloadLength;
        uint16_t payloadType;
    };
    struct  __attribute__((packed, aligned(1))) MPEGHeader {
        uint8_t sync;
        uint16_t tei_pusi_tp_pid;
        uint8_t tsc_af_counter;
    };

    PacketBuffer& packetBuffer_;
    unsigned long mtu_;
    uint16_t pid_;

    enum {
        FRAMER_WAITBEGIN, FRAMER_INPACKET
    } framerState_;

    uint8_t* localBuffer_;
    unsigned long packetInBuffer_ = 0;
    unsigned long packetBufferSize = 0;

    unsigned int mpegContCounter_ = 0;
    /* NULL packet does not really need the counter, but it is handy for debugging */
    unsigned int mpegNullContCounter_ = 0;
    unsigned long packetBufferMpegIndex_ = 0;

    bool sendNULL_;

    uint8_t gatewayMac_[6];
    bool gatewayMacValid_ = false;

    std::unordered_map<std::array<uint8_t, 17>, std::array<uint8_t, 6>, array_hasher> targetMacsIP_;
};



#endif /* SRC_ULEFRAMER_H_ */
