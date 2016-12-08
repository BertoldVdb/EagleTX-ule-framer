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


#include <arpa/inet.h>
#include <cstring>

#include "ULEFramer.h"
#include "TunTap.h"
#include "Util.h"

int ULEFramer::prepareULEPacket()
{
    if(packetInBuffer_) return packetInBuffer_;
    /* Read the metadata of the packet */
    uint8_t metadata[4];
    uint16_t packetSize = packetBuffer_.peekPacket(metadata, sizeof(metadata));

    if(packetSize >= 4) {
        packetSize -= 4;
        bool usingTap = metadata[0];
        TunTap::PacketType packetType = (TunTap::PacketType)metadata[1];
        uint16_t etherType = (metadata[2] << 8) | metadata[3];

        struct ULEHeader* uleHeader = reinterpret_cast<struct ULEHeader*>(localBuffer_);
        uint8_t* destAddr = localBuffer_ + sizeof(struct ULEHeader);

        bool destinationAbsent;
        if(!usingTap) {
            /*
             * Destination is needed for:
             * 	Unicast frames with known receiver MAC, otherwise broadcast
             * 	Remote unicast frames if gateway MAC is known, otherwise drop
             * And not needed for:
             * 	Broadcast, multicast
             */
            if(packetType == TunTap::BROADCAST) {
                destinationAbsent = 1;
            } else if(packetType == TunTap::MULTICAST) {
                destinationAbsent = 1;
            } else if(packetType == TunTap::UNICAST_LOCAL) {
                /* Check if a mapping between IP and MAC is provided, otherwise broadcast it */
                if(targetMacsIP_.size() == 0) {
                    destinationAbsent = 1;
                } else {
                    destinationAbsent = 0;
                }
            } else if(packetType == TunTap::UNICAST_REMOTE) {
                /*
                 * Broadcasting a routed frame is not allowed by the standard.
                 * If you want to do it anyway you can set the gateway to
                 * FF:FF:FF:FF:FF:FF
                 */
                if(gatewayMacValid_) {
                    if(gatewayMac_[0] & 1) {
                        destinationAbsent = 1;
                    } else {
                        destinationAbsent = 0;
                    }
                } else {
                    packetBuffer_.getPacket(NULL, 0);
                    return -EAGAIN;
                }
            } else {
                packetBuffer_.getPacket(NULL, 0);
                return -EAGAIN;
            }

            uleHeader->payloadType = htons(etherType);
        } else {
            if(packetType == TunTap::BROADCAST) {
                destinationAbsent = 1;
            } else if(packetType == TunTap::MULTICAST) {
                destinationAbsent = 1;
            } else if(packetType == TunTap::UNICAST_LOCAL) {
                destinationAbsent = 0;
            } else {
                packetBuffer_.getPacket(NULL, 0);
                return -EAGAIN;
            }

            uleHeader->payloadType = htons(0x0001);
        }

        uint8_t* payloadBuffer = localBuffer_ + sizeof(struct ULEHeader);
        uleHeader->payloadLength = packetSize + 4;
        if(!destinationAbsent) {
            uleHeader->payloadLength += 6;
            payloadBuffer += 6;
        }

        /* Read the data to the buffer */
        packetBuffer_.getPacket(payloadBuffer, mtu_, 4);

        if(!destinationAbsent) {
            if(!usingTap) {
                if(packetType == TunTap::UNICAST_LOCAL) {
                    /* Get the IP from the packet */
                    if(etherType == 0x0800) {
                        std::array<uint8_t, 17> ip4 = {4};
                        memset(ip4.data()+1+4, 0, 12);
                        memcpy(ip4.data()+1, &payloadBuffer[16], 4);

                        auto ret = targetMacsIP_.find(ip4);
                        if (ret != targetMacsIP_.end()) {
                            memcpy(destAddr, ret->second.data(), 6);
                        } else {
                            /* Drop the frame */
                            return -EAGAIN;
                        }

                    } else if(etherType == 0x86DD) {
                        std::array<uint8_t, 17> ip6 = {6};
                        memcpy(ip6.data()+1, &payloadBuffer[24], 16);

                        auto ret = targetMacsIP_.find(ip6);
                        if (ret != targetMacsIP_.end()) {
                            memcpy(destAddr, ret->second.data(), 6);
                        } else {
                            /* Drop the frame */
                            return -EAGAIN;
                        }

                    } else {
                        /* Send it to everybody */
                        memset(destAddr, 0xFF, 6);
                    }
                } else if(packetType == TunTap::UNICAST_REMOTE) {
                    memcpy(destAddr, gatewayMac_, 6);
                } else {
                    memset(destAddr, 0xFF, 6);
                }

            } else {
                memcpy(destAddr, payloadBuffer, 6);
            }
        }

        packetInBuffer_ = sizeof(struct ULEHeader) + uleHeader->payloadLength;
        uint8_t* crcBuffer = payloadBuffer + packetSize;

        uleHeader->payloadLength = htons(uleHeader->payloadLength | (destinationAbsent << 15));

        uint32_t crc = htonl(crc32(0xFFFFFFFF, localBuffer_, packetInBuffer_ - 4));
        memcpy(crcBuffer, &crc, sizeof(crc));

        return packetInBuffer_;
    } else {
        return 0;
    }

}


int ULEFramer::getMpegPacket(uint8_t* buffer, uint8_t frameLen)
{
    uint8_t bytesRemaining = frameLen;
    struct MPEGHeader* mpegHeader = reinterpret_cast<struct MPEGHeader*>(buffer);

    if(frameLen < sizeof(struct MPEGHeader) + 2) {
        return 0;
    }

    while(prepareULEPacket() == -EAGAIN);

    if(!packetInBuffer_) {
        /* Make a null packet if desired */
        if(sendNULL_) {
            mpegHeader->sync = 0x47;
            mpegHeader->tei_pusi_tp_pid = htons(0x1FFF);
            mpegHeader->tsc_af_counter = ((mpegNullContCounter_++) & 0xF);
            memset(buffer + sizeof(struct MPEGHeader), 0x0, frameLen - sizeof(struct MPEGHeader));
            return frameLen;
        }
        return 0;
    }

    mpegHeader->sync = 0x47;
    mpegHeader->tei_pusi_tp_pid = pid_;
    mpegHeader->tsc_af_counter = (0x1 << 4) | ((mpegContCounter_++) & 0xF);

    uint8_t* payloadBuffer = buffer + sizeof(struct MPEGHeader);
    bytesRemaining -= sizeof(struct MPEGHeader);

    /* Check if we should set PUSI */
    if(!packetBufferMpegIndex_) {
        /* Yes, to 0 */
        mpegHeader->tei_pusi_tp_pid |= (1<<14);
        payloadBuffer[0] = 0;
        payloadBuffer++;
        bytesRemaining--;
    } else if(packetInBuffer_ - packetBufferMpegIndex_ < bytesRemaining - 2U) {
        /*
         * Maybe a packet will end during this frame, and there is enough
         * room to start a new packet. Check if one is in the queue.
         */
        if(packetBuffer_.packetsInQueue()) {
            /* Yes, set PUSI to byte following this one */
            mpegHeader->tei_pusi_tp_pid |= (1<<14);
            /* The PUSI itself does not count, so no +1 */
            payloadBuffer[0] = packetInBuffer_ - packetBufferMpegIndex_;
            payloadBuffer++;
            bytesRemaining--;
        }
    }

    while(bytesRemaining) {
        if(packetInBuffer_ - packetBufferMpegIndex_ >= bytesRemaining) {
            memcpy(payloadBuffer, localBuffer_ + packetBufferMpegIndex_, bytesRemaining);
            /* TS Packet is full, and we need to continue with it */
            packetBufferMpegIndex_+= bytesRemaining;
            payloadBuffer += bytesRemaining;
            bytesRemaining = 0;
            break;
        } else {
            memcpy(payloadBuffer, localBuffer_ + packetBufferMpegIndex_, packetInBuffer_ - packetBufferMpegIndex_);
            bytesRemaining -= packetInBuffer_ - packetBufferMpegIndex_;
            payloadBuffer += packetInBuffer_ - packetBufferMpegIndex_;

            packetInBuffer_ = 0;
            packetBufferMpegIndex_ = 0;
            if(bytesRemaining <= 2) {
                memset(payloadBuffer, 0xFF, bytesRemaining);
                break;
            }
            /* We can start a new frame */
            while(prepareULEPacket() == -EAGAIN);
            if(!packetInBuffer_) {
                /* No packet, put an end marker */
                memset(payloadBuffer, 0xFF, bytesRemaining);
                break;
            }
            /* Got a new packet, go on until all bytes are used */
        }
    }

    mpegHeader->tei_pusi_tp_pid = htons(mpegHeader->tei_pusi_tp_pid);
    return frameLen;
}

