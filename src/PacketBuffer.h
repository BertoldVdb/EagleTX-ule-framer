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


#ifndef SRC_PACKETBUFFER_H_
#define SRC_PACKETBUFFER_H_
#include <iostream>
#include <cstdint>
#include <cstdlib>

class PacketBuffer
{
public:
    PacketBuffer(unsigned long size);
    ~PacketBuffer();

    int getPacket(uint8_t* buffer, uint16_t bufferSize);
    int getPacket(uint8_t* buffer, uint16_t bufferSize, uint16_t skip);
    int peekPacket(uint8_t* buffer, uint16_t bufferSize);
    int putPacket(uint8_t* buffer, uint16_t bufferSize);

    void printStatus(std::ostream& cout) {
        cout << "Queue:\n";
        cout << "\tPackets: "<<bufferPackets_<<"\n";
        cout << "\tBytes: "<<bufferPacketsBytes_<<"/"<<size_<<"\n";
        cout << "\tPackets Dropped: "<<bufferPacketsDropped_<<"\n";
    }

    unsigned long packetsInQueue() {
        return bufferPackets_;
    }

private:
    void inline checkCanWrite() {
        if(bufferWriteIndex_ == bufferReadIndex_) {
            /*
             * We are going to overwrite the tag of the packet that would be read next,
             * so dump it.
             */
            getPacket(NULL, 0);
        }
    }
    void inline incWriteIndex() {
        //	std::cout << "Wrote index:" <<bufferWriteIndex_<<" Readindex: "<<bufferReadIndex_<<" packets: "<<bufferPackets_<<"\n";

        bufferWriteIndex_ ++ ;
        if(bufferWriteIndex_ >= size_) {
            bufferWriteIndex_ = 0;
        }
    }

    void inline incReadIndex() {
        bufferReadIndex_ ++ ;
        if(bufferReadIndex_ >= size_) {
            bufferReadIndex_ = 0;
        }
    }

    unsigned long size_;

    uint8_t* buffer_;
    unsigned long bufferWriteIndex_;
    unsigned long bufferReadIndex_;
    unsigned long bufferPackets_;

    unsigned long bufferPacketsBytes_;
    unsigned long bufferPacketsDropped_;
};



#endif /* SRC_PACKETBUFFER_H_ */
