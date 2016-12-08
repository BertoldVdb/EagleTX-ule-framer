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

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <limits.h>
#include <cstring>
#include <stdlib.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <fcntl.h>
#include <iostream>


#include "Util.h"

bool netSockaddrEqual(const sockaddr_in& addr1, const sockaddr_in& addr2)
{
    if(addr1.sin_port != addr2.sin_port) return false;
    if(addr1.sin_addr.s_addr != addr2.sin_addr.s_addr) return false;
    return true;
}

bool netSockaddrEqual(const sockaddr_in6& addr1, const sockaddr_in6& addr2)
{
    if(addr1.sin6_port != addr2.sin6_port) return false;
    if(memcmp(&addr1.sin6_addr.s6_addr, &addr2.sin6_addr.s6_addr, 16)) return false;
    return true;
}

bool netSockaddrEqual(const sockaddr_storage& addr1, const sockaddr_storage& addr2)
{
    if(addr1.ss_family!=addr2.ss_family) return 0;

    switch(addr1.ss_family) {
        case AF_INET:
            return netSockaddrEqual(reinterpret_cast<const sockaddr_in&>(addr1), reinterpret_cast<const sockaddr_in&>(addr2));
        case AF_INET6:
            return netSockaddrEqual(reinterpret_cast<const sockaddr_in6&>(addr1), reinterpret_cast<const sockaddr_in6&>(addr2));
    }

    return false;
}

void netAddrToString (std::string& out, in6_addr& sin6_addr)
{
    char tmp[50] = {0};
    inet_ntop(AF_INET6, &sin6_addr, tmp, sizeof(tmp));
    out = tmp;
}

void netAddrToString (std::string& out, in_addr& sin_addr)
{
    char tmp[50] = {0};
    inet_ntop(AF_INET, &sin_addr, tmp, sizeof(tmp));
    out = tmp;
}

void netAddrToString (std::string& out, sockaddr_storage* sockaddr, bool includePort)
{
    unsigned int port = 0;
    if(sockaddr->ss_family == AF_INET6) {
        netAddrToString(out, reinterpret_cast<sockaddr_in6*>(sockaddr)->sin6_addr);
        out="["+out+"]";
        port = ntohs(reinterpret_cast<sockaddr_in6*>(sockaddr)->sin6_port);
    } else if(sockaddr->ss_family == AF_INET) {
        netAddrToString(out, reinterpret_cast<sockaddr_in*>(sockaddr)->sin_addr);
        port = ntohs(reinterpret_cast<sockaddr_in*>(sockaddr)->sin_port);
    } else {
        out = "Unknown";
        return;
    }
    if(includePort) {
        out+=":";
        out+=std::to_string(port);
    }
}

void netAddrToString (std::string& out, sockaddr_storage& sockaddr, bool includePort)
{
    netAddrToString(out, &sockaddr, includePort);
}

int netSetFdBlocking(int s, bool blocking)
{
    int flags = fcntl(s, F_GETFL, 0);
    if(flags < 0) {
        return -1;
    }

    flags &=~ O_NONBLOCK;
    if(!blocking) flags |= O_NONBLOCK;

    if(fcntl(s, F_SETFL, flags) < 0) {
        return -1;
    }
    return 0;
}

void netMacToArray(uint8_t* mac, std::string const& in)
{
    unsigned int macI[6];
    if (std::sscanf(in.c_str(),
                    "%02x:%02x:%02x:%02x:%02x:%02x",
                    &macI[0], &macI[1], &macI[2],
                    &macI[3], &macI[4], &macI[5]) != 6) {
        throw std::runtime_error(in + std::string(" could not be parsed"));
    }

    unsigned int i;
    for(i=0; i<6; i++) {
        mac[i]=macI[i];
    }
}

pid_t goBackground()
{
    pid_t pid;

    pid = fork();

    if (pid < 0)
        return pid;

    if (pid > 0)
        return pid;

    if (setsid() < 0)
        return -1;

    signal(SIGCHLD, SIG_IGN);
    signal(SIGHUP, SIG_IGN);

    pid = fork();

    if (pid < 0)
        return pid;

    if (pid > 0)
        return pid;

    umask(0);
    if(chdir("/") <0 )
        return -1;

    fclose(stdin);
    fclose(stdout);
    fclose(stderr);

    open("/dev/null", O_RDONLY);
    open("/dev/null", O_RDWR);
    open("/dev/null", O_RDWR);

    return 0;
}

void clearConsole()
{
    std::cout << "\x1B[2J\x1B[H";
}

void strLowercase(std::string& input)
{
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);
}
