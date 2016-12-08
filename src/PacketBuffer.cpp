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


#include "PacketBuffer.h"

PacketBuffer::PacketBuffer(unsigned long size):
    size_(size),
    bufferWriteIndex_(0),
    bufferReadIndex_(0),
    bufferPackets_(0),
    bufferPacketsBytes_(0),
    bufferPacketsDropped_(0)
{

    buffer_ =  new uint8_t[size]();
}

PacketBuffer::~PacketBuffer()
{
    delete buffer_;
}

int PacketBuffer::peekPacket(uint8_t* buffer, uint16_t bufferSize)
{
    unsigned long readIndex = bufferReadIndex_;

    uint16_t packetSize = getPacket(buffer, bufferSize);

    bufferReadIndex_= readIndex;

    if(packetSize) {
        bufferPackets_++;
        bufferPacketsBytes_ += packetSize;
    }

    return packetSize;
}

int PacketBuffer::getPacket(uint8_t* buffer, uint16_t bufferSize)
{
    return getPacket(buffer, bufferSize, 0);
}

int PacketBuffer::getPacket(uint8_t* buffer, uint16_t bufferSize, uint16_t skip)
{
    uint16_t packetSize;
    unsigned long bufferFinalReadIndex;

    if(!bufferPackets_) {
        return 0;
    }

    packetSize = buffer_[bufferReadIndex_] << 8;
    incReadIndex();
    packetSize |= buffer_[bufferReadIndex_] & 0xFF;
    incReadIndex();

    bufferFinalReadIndex = bufferReadIndex_ + packetSize;
    while (bufferFinalReadIndex >= size_) {
        bufferFinalReadIndex -= size_;
    }

    if(buffer) {
        for(uint16_t i=0; i<packetSize; i++) {
            if(i>=skip) {
                if(i-skip >= bufferSize) {
                    break;
                }
                buffer[i-skip] = buffer_[bufferReadIndex_];
            }
            incReadIndex();
        }
    }

    bufferReadIndex_ = bufferFinalReadIndex;

    bufferPackets_--;
    bufferPacketsBytes_ -= packetSize;

    return packetSize;
}

int PacketBuffer::putPacket(uint8_t* buffer, uint16_t bufferSize)
{
    /* Check if the packet is larger than the entire buffer */
    if(bufferSize > size_-2) {
        return -1;
    }


    int dropped = bufferPackets_;
    checkCanWrite();
    buffer_[bufferWriteIndex_] = bufferSize >> 8;
    incWriteIndex();

    checkCanWrite();
    buffer_[bufferWriteIndex_] = bufferSize & 0xFF;
    incWriteIndex();

    unsigned int i;
    for(i=0; i<bufferSize; i++) {
        checkCanWrite();
        buffer_[bufferWriteIndex_] = buffer[i];
        incWriteIndex();
    }

    dropped -= bufferPackets_;
    bufferPacketsDropped_+= dropped;

    bufferPackets_++;
    bufferPacketsBytes_+=bufferSize;

    return dropped;
}
