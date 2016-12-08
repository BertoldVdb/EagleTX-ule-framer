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



#ifndef SRC_UTIL_UTIL_HPP_
#define SRC_UTIL_UTIL_HPP_

#include <cmath>
#include <cstdlib>
#include <vector>
#include <utility>
#include <functional>
#include <cstdlib>
#include <ctime>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

bool netSockaddrEqual(const sockaddr_in& addr1, const sockaddr_in& addr2);
bool netSockaddrEqual(const sockaddr_in6& addr1, const sockaddr_in6& addr2);
bool netSockaddrEqual(const sockaddr_storage& addr1, const sockaddr_storage& addr2);
void netAddrToString (std::string& out, in6_addr& sin6_addr);
void netAddrToString (std::string& out, in_addr& sin_addr);
void netAddrToString (std::string& out, sockaddr_storage* sockaddr, bool includePort = true);
void netAddrToString (std::string& out, sockaddr_storage& sockaddr, bool includePort = true);
int netSetFdBlocking(int s, bool blocking);

void clearConsole(void);
void strLowercase(std::string& input);

uint32_t crc32(uint32_t crc, uint8_t* buf, size_t size);
void netMacToArray(uint8_t* mac, std::string const& in);
pid_t goBackground();

struct array_hasher {
    template<class T> std::size_t operator()(const T& c) const {
        return crc32(0xDEADBEEF, (uint8_t*)c.data(), c.size() * sizeof(*c.data()));
    }
};

#endif /* SRC_UTIL_UTIL_HPP_ */
