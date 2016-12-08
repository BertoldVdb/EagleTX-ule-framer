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

#include "UDPOutput.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <exception>
#include <cstring>
#include <stdexcept>
#include <cstring>
#include <unistd.h>

#include "Util.h"

UDPOutput::UDPOutput(std::string& destination, unsigned long bufferSize)
{
    try {
        if(bufferSize == 0) {
            bufferSize = 1400;
        }
        sockBufferSize_ = bufferSize;

        sockBuffer_ = new uint8_t[sockBufferSize_];

        uint16_t port;
        std::string hostname;

        if(destination == "") {
            destination = "127.0.0.1";
        }

        auto portIndex = destination.find(":");
        if(portIndex == std::string::npos) {
            port = 12121;
            hostname = destination;
        } else {
            hostname = destination.substr(0, portIndex);
            port = atoi(destination.substr(portIndex+1, std::string::npos).c_str());
        }

        port_ = std::to_string(port);

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if(sock_ < 0) {
            throw std::runtime_error("Could not create socket");
        }

        struct hostent *server;
        server = gethostbyname(hostname.c_str());
        if (server == NULL) {
            throw std::runtime_error("Could not resolve hostname");
        }

        memset(&serveraddr_, 0, sizeof(serveraddr_));
        serveraddr_.sin_family = AF_INET;
        memcpy(&serveraddr_.sin_addr.s_addr, server->h_addr, server->h_length);
        serveraddr_.sin_port = htons(port);

        netAddrToString(host_, serveraddr_.sin_addr);

    } catch(...) {
        cleanup();
        throw;
    }
}

void UDPOutput::cleanup()
{
    if(sockBuffer_) {
        delete sockBuffer_;
    }
    if(sock_ >= 0) {
        close(sock_);
    }
}
