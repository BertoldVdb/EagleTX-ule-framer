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
#ifndef SRC_UDPOUTPUT_H_
#define SRC_UDPOUTPUT_H_

#include <stdint.h>
#include <string>
#include <ostream>
#include "OutputModule.h"

#include <arpa/inet.h>
#include <sys/socket.h>

class UDPOutput : public OutputModule
{
public:
    UDPOutput(std::string& destination, unsigned long bufferSize);

    ~UDPOutput() {
        cleanup();
    }

    long getBuffer() {
        return 0;
    }

    int putBuffer(unsigned long buffer, unsigned long bytesUsed) {
        return sendto(sock_, sockBuffer_, bytesUsed, 0, reinterpret_cast<const sockaddr*>(&serveraddr_), sizeof(serveraddr_));
    }

    void writeStats(std::ostream& cout) {
        cout << "UDP:\n\tDestination: "+host_+":"+port_+"\n";
    }

    uint8_t* accessBuffer(unsigned long buffer, unsigned long& size) {
        size = sockBufferSize_;
        return (uint8_t*)sockBuffer_;
    }

    int getFD() {
        return sock_;
    }

    /* The socket has no path */
    std::string getDevicePath() {
        return "/dev/fakepath/socket_udp/" + std::to_string(sock_);
    }

private:
    void cleanup();

    int sock_ = -1;

    uint8_t* sockBuffer_ = NULL;
    unsigned long sockBufferSize_;

    struct sockaddr_in serveraddr_;

    std::string host_, port_;
};



#endif /* SRC_UDPOUTPUT_H_ */
