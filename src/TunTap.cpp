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

#include "TunTap.h"

#include <stdexcept>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <iomanip>

#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <netdb.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <linux/if_link.h>
#include <sys/time.h>

#include "Util.h"

TunTap::TunTap(bool tap):
    TunTap("", tap)
{
}

TunTap::TunTap(std::string name, bool tap)
{
    try {
        lastAction_.tv_sec = 0;
        lastAction_.tv_usec = 0;

        if((tunfd_ = open("/dev/net/tun", O_RDWR)) < 0) {
            throw std::runtime_error("Could not connect to TUN module");
        }

        struct ifreq ifreq;
        memset(&ifreq, 0, sizeof(ifreq));

        if(tap) {
            ifreq.ifr_flags = IFF_TAP;
        } else {
            ifreq.ifr_flags = IFF_TUN;
        }

        /* Request a specific interface */
        if (name != "") {
            strncpy(ifreq.ifr_name, name.c_str(), IFNAMSIZ);
        }

        if(ioctl(tunfd_, TUNSETIFF, (void*)&ifreq) < 0) {
            throw std::runtime_error("Failed to request interface");
        }

        ifname_ = ifreq.ifr_name;
        tap_ = tap;
        mtu_ = getMTU();

        doBookkeeping();

    } catch(...) {
        cleanup();
        throw;
    }
}

void TunTap::cleanup()
{
    if(tunfd_ >= 0) close(tunfd_);
}

unsigned long TunTap::getMTU()
{
    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname_.c_str(), sizeof(ifr.ifr_name));

    int dummyfd = socket(AF_INET, SOCK_DGRAM, 0);

    if(ioctl(dummyfd, SIOCGIFMTU, (caddr_t)&ifr) == -1) {
        close(dummyfd);
        return 0;
    }

    close(dummyfd);
    return ifr.ifr_mtu;
}

bool TunTap::setMTU(unsigned long mtu)
{
    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname_.c_str(), sizeof(ifr.ifr_name));
    ifr.ifr_mtu = mtu;

    int dummyfd = socket(AF_INET, SOCK_DGRAM, 0);

    if(ioctl(dummyfd, SIOCSIFMTU, (caddr_t)&ifr) == -1) {
        close(dummyfd);
        return false;
    }
    close(dummyfd);

    mtu_ = getMTU();

    return true;
}

TunTap::PacketType TunTap::ipIsLocal(struct sockaddr& sockaddr)
{
    /*
     * As long as we have no valid addresses we cannot do anything,
     * so keep trying to get them. The fact that we got a packet means
     * that an address must be set, so this is a good time to try updating
     * the lists.
     */
    if (sockaddr.sa_family == AF_INET) {
        if(!validAddressV4_) {
            updateNetinfo();
        }
    }
    if (sockaddr.sa_family == AF_INET6) {
        if(!validAddressV6_) {
            updateNetinfo();
        }
    }

    /* Check if it is a special address */
    if (sockaddr.sa_family == AF_INET) {
        uint32_t ip = ntohl(reinterpret_cast<struct sockaddr_in&>(sockaddr).sin_addr.s_addr);
        if(ip == 0xFFFFFFFF) {
            return BROADCAST;
        }
        if((ip & 0xF0000000) == 0xE0000000) {
            return MULTICAST;
        }
    } else if(sockaddr.sa_family == AF_INET6) {
        uint8_t* ip = reinterpret_cast<uint8_t*>(&reinterpret_cast<struct sockaddr_in6&>(sockaddr).sin6_addr);
        if(ip[0] == 0xFF) {
            return MULTICAST;
        }
    } else {
        return UNKNOWN;
    }

    struct sockaddr_storage sip, smask;
    for(auto tmp : networkAddrs_) {
        std::tie(sip, smask) = tmp;
        if(sip.ss_family != sockaddr.sa_family)
            continue;

        if(sip.ss_family == AF_INET) {
            uint32_t ip   = ntohl(reinterpret_cast<struct sockaddr_in&>(sockaddr).sin_addr.s_addr);
            uint32_t addr = ntohl(reinterpret_cast<struct sockaddr_in&>(sip).sin_addr.s_addr);
            uint32_t mask = ntohl(reinterpret_cast<struct sockaddr_in&>(smask).sin_addr.s_addr);
            /* Is it my address */
            if(addr == ip) {
                return UNICAST_MYADDR;
            }
            /* Is it local */
            if((addr & mask) == (ip & mask)) {
                /* Is it a broadcast address */
                if((ip & ~mask) == (0xFFFFFFFF & ~mask)) {
                    return BROADCAST;
                }
                return UNICAST_LOCAL;
            }
        } else if(sip.ss_family == AF_INET6) {
            uint8_t* ip   = reinterpret_cast<uint8_t*>(&reinterpret_cast<struct sockaddr_in6&>(sockaddr).sin6_addr);
            uint8_t* addr = reinterpret_cast<uint8_t*>(&reinterpret_cast<struct sockaddr_in6&>(sip).sin6_addr);
            uint8_t* mask = reinterpret_cast<uint8_t*>(&reinterpret_cast<struct sockaddr_in6&>(smask).sin6_addr);
            int i;
            bool canBeLocal = true;
            bool canBeMine = true;
            for(i=0; i<16; i++) {
                if((addr[i]) != (ip[i])) {
                    canBeMine = false;
                    break;
                }
                if((addr[i] & mask[i]) != (ip[i] & mask[i])) {
                    canBeLocal = false;
                    break;
                }
            }
            if(canBeMine) {
                return UNICAST_MYADDR;
            }
            if(canBeLocal) {
                return UNICAST_LOCAL;
            }
        }
    }

    return UNICAST_REMOTE;
}

bool TunTap::updateNetinfo()
{
    struct ifreq ifreq;
    uint8_t mac[6];
    memset(mac, 0, sizeof(mac));
    validAddressV4_ = false;
    validAddressV6_ = false;
    validAddressTAP_ = false;

    /* Get the MAC address, for a TAP interface */
    if(tap_) {
        memset(&ifreq, 0, sizeof(ifreq));
        strncpy(ifreq.ifr_name, ifname_.c_str(), IFNAMSIZ);

        int dummyfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (ioctl(dummyfd, SIOCGIFHWADDR, &ifreq) == -1) {
            close(dummyfd);
            return false;
        }
        close(dummyfd);

        int i;
        for(i=0; i<6; i++) {
            mac[i] = ifreq.ifr_addr.sa_data[i];
        }
        validAddressTAP_ = true;
    }

    /* Get IPv4 addr & netmask, same for IPv6 */
    struct ifaddrs *ifaddr, *ifa;
    int n;

    if (getifaddrs(&ifaddr) == -1) {
        return false;
    }

    networkAddrs_.clear();

    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
        if (ifa->ifa_addr == NULL)
            continue;
        if (strcmp(ifa->ifa_name, ifname_.c_str()))
            continue;

        if(ifa->ifa_addr->sa_family == AF_INET || ifa->ifa_addr->sa_family == AF_INET6) {
            struct sockaddr_storage* ifa_addr = reinterpret_cast<sockaddr_storage*>(ifa->ifa_addr);
            struct sockaddr_storage* ifa_mask = reinterpret_cast<sockaddr_storage*>(ifa->ifa_netmask);

            networkAddrs_.push_back(std::tie(*ifa_addr, *ifa_mask));
            if(ifa->ifa_addr->sa_family == AF_INET) {
                validAddressV4_ = true;
            }
            if(ifa->ifa_addr->sa_family == AF_INET6) {
                validAddressV6_ = true;
            }
        }
    }

    freeifaddrs(ifaddr);

    memcpy(networkMAC_, mac, sizeof(networkMAC_));
    return true;
}

void TunTap::printIfStats(std::ostream& cout)
{
    std::ios oldState(nullptr);
    oldState.copyfmt(cout);

    unsigned int addr = 0;
    cout <<"Interface "<<ifname_<<" ";
    if(tap_) {
        cout<<"[TAP, "+std::to_string(mtu_)+"]:\n";
    } else {
        cout<<"[TUN, "+std::to_string(mtu_)+"]:\n";
    }

    if(validAddressV4_ || validAddressV6_ || validAddressTAP_) {
        cout <<"\tLocal addresses:\n";
        if(tap_) {
            cout <<"\t\t{";
            for (uint i = 0; i < 6; i++) {
                cout << std::setfill('0') << std::setw(2) << std::hex << static_cast<unsigned int>(networkMAC_[i]);
                if(i!=5) {
                    cout <<":";
                }
            }
            cout << "}\n";
            addr++;
        }

        cout.copyfmt(oldState);

        struct sockaddr_storage sip, smask;
        for(auto tmp : networkAddrs_) {
            std::tie(sip, smask) = tmp;
            std::string ip;
            netAddrToString(ip, sip, false);
            cout << "\t\t" << ip <<"/";
            netAddrToString(ip, smask, false);
            cout << ip <<"\n";
            addr++;
        }
    } else {
        cout <<"\tWARNING: Interface has no valid local address. Things will not work!\n";
    }

    cout <<"\tStatistics:\n";
    cout <<"\t\tPackets received: "<<packetsReceived<<"\n";
    cout <<"\t\tPackets to link-local destinations: "<<packetsUnicastLinkLocal<<"\n";
    cout <<"\t\tPackets to remote destinations: "<<packetsUnicastRemote<<"\n";
    cout <<"\t\tPackets to multicast destinations: "<<packetsMulticast<<"\n";
    cout <<"\t\tPackets to broadcast destinations: "<<packetsBroadcast<<"\n";
    cout <<"\t\tPackets to local address: "<<packetsUnicastToMe<<"\n";
    cout <<"\t\tPackets with unknown protocol: "<<packetsUnknown<<"\n";
    cout <<"\t\tPackets sent: "<<packetsSent<<"\n";
}

void TunTap::doBookkeeping()
{
    struct timeval now;
    gettimeofday(&now, NULL);
    if(now.tv_sec >= lastAction_.tv_sec + 2) {
        lastAction_ = now;
        updateNetinfo();
    }
}

int TunTap::writePacket(uint8_t* buffer, int bufferLen)
{
    packetsSent++;
    return write(tunfd_, buffer, bufferLen);
}


int TunTap::readPacket(uint8_t* buffer, int bufferLen, PacketType& packetTypeOut)
{
    PacketType packetType = UNKNOWN;

    doBookkeeping();

    int len = read(tunfd_, buffer, bufferLen);
    uint16_t* flags = (uint16_t*)buffer;

    if(len <= 0) {
        goto failed;
    }
    if(len <= 4) {
        len = 0;
        goto done;
    }

    buffer+=4;
    len-=4;

    if(tap_) {
        /* This is an Ethernet packet */
        if(len < 12) {
            goto done;
        }
        if(!memcmp(buffer, networkMAC_, 6)) {
            packetType = UNICAST_MYADDR;
        } else if(buffer[0] & 1) {
            if(!memcmp(buffer, "\xFF\xFF\xFF\xFF\xFF\xFF", 6)) {
                packetType = BROADCAST;
            } else {
                packetType = MULTICAST;
            }
        } else {
            packetType = UNICAST_LOCAL;
        }
    } else {
        /* IP Packet: check the protocol */
        uint16_t protocol = ntohs(flags[1]);
        if(protocol == 0x0800) {
            /* IPv4 */
            if(len < 20) goto done;

            struct sockaddr_in addr;
            memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            memcpy(&addr.sin_addr, &buffer[16], 4);

            packetType = ipIsLocal(reinterpret_cast<sockaddr&>(addr));

        } else if(protocol == 0x86DD) {
            /* IPv6 */
            if(len < 40) goto done;

            struct sockaddr_in6 addr;
            memset(&addr, 0, sizeof(addr));
            addr.sin6_family = AF_INET6;
            memcpy(&addr.sin6_addr, &buffer[24], 16);

            packetType = ipIsLocal(reinterpret_cast<sockaddr&>(addr));
        } else {
            /* Unknown */
            packetType = UNKNOWN;
        }
    }

done:

    packetsReceived++;
    switch (packetType) {
        case BROADCAST:
            packetsBroadcast++;
            break;
        case MULTICAST:
            packetsMulticast++;
            break;
        case UNICAST_LOCAL:
            packetsUnicastLinkLocal++;
            break;
        case UNICAST_REMOTE:
            packetsUnicastRemote++;
            break;
        case UNICAST_MYADDR:
            packetsUnicastToMe++;
            break;
        default:
            packetsUnknown++;
            break;
    }
failed:
    packetTypeOut = packetType;

    return len;
}

TunTap::~TunTap()
{
    cleanup();
}
