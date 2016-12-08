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

#include <iostream>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/un.h>
#include <fstream>
#include <getopt.h>
#include <signal.h>
#include <fstream>

#include "TunTap.h"
#include "PacketBuffer.h"
#include "ULEFramer.h"
#include "OutputModule.h"
#include "V4L2Output.h"
#include "UDPOutput.h"
#include "FileOutput.h"
#include "TimerFD.h"

#include "Util.h"

static void stackPrefault(unsigned long maxStack)
{
    unsigned char dummy[maxStack];

    memset(dummy, 0, maxStack);
}

static void setRealtimePriority(unsigned int priority, unsigned long maxStack)
{
    struct sched_param param;
    param.sched_priority = priority;
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cout << "Could not enable FIFO scheduling ("<<strerror(errno)<<"). Performance may be degraded to the point the system will not work!\n";
    }

    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        std::cout << "Could not pin memory to RAM ("<<strerror(errno)<<"). Performance may be degraded!\n";
    }

    stackPrefault(maxStack);
}


static struct option long_options[] = {
    {"interface", required_argument, 0, 'i'},
    {"mtu", required_argument, 0, 'm'},
    {"outputmodule", required_argument, 0, 'o'},
    {"destination", required_argument, 0, 'd'},
    {"buffer", required_argument, 0, 'b'},
    {"pid", required_argument, 0, 'p'},
    {"mpegts", required_argument, 0, 'n'},
    {"tap", no_argument, 0, 't'},
    {"null", no_argument, 0, 'N'},
    {"background", no_argument, 0, 'B'},
    {"logfile", required_argument, 0, 'l'},
    {"timed", required_argument, 0, 'T'},
    {"symlink", required_argument, 0, 'x'},
    {"queue", required_argument, 0, 'q'},
    {"macsfile", required_argument, 0, 'M'},
    {"gateway", required_argument, 0, 'g'},
    {"outputlen", required_argument, 0, 'L'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
};

static void usage()
{
    std::cout << "The following options are supported:\n";
    std::cout<<"\t--help, -h\t\t: Prints this help.\n";
    std::cout<<"\t--interface, -i\t\t: Interface name (default: dvb0)\n";
    std::cout<<"\t--mtu, -m\t\t: Maximum packet size. Sets device MTU! (default: current MTU)\n";
    std::cout<<"\t--outputmodule, -o\t: Output module to use! (default: udp)\n";
    std::cout<<"\t--destination, -d\t: MPEG-TS destination. Depends on output module.\n";
    std::cout<<"\t--symlink, -x\t\t: Create a symbolic link to the device (default: none)\n";
    std::cout<<"\t--buffer, -b\t\t: Size of packet buffer (default: 15000)\n";
    std::cout<<"\t--queue, -q\t\t: Buffer queue length (default: 2)\n";
    std::cout<<"\t--priority, -p\t\t: Set realtime priority (default: 60)\n";
    std::cout<<"\t--pid, -P\t\t: MPEG-TS PID (default: 0x666)\n";
    std::cout<<"\t--mpegts, -n\t\t: MPEG-TS packet size (default: 188)\n";
    std::cout<<"\t--tap, -t\t\t: Use Ethernet bridging (default: no)\n";
    std::cout<<"\t--timed, -T\t\t: Use timed mode. Produce output every n us. (default: backpressure mode)\n";
    std::cout<<"\t--outputlen, -L\t\t: Number of MPEG-TS packets per output (default: optimal number)\n";
    std::cout<<"\t--null, -N\t\t: Inject PID 0x1FFF packets (default: no)\n";
    std::cout<<"\t--macsfile, -M\t\t: File with IP/MAC mappings (default: not used)\n";
    std::cout<<"\t--gateway, -g\t\t: Default gateway in TUN mode (default: not set)\n";
    std::cout<<"\t--background, -B\t: Go to background (default: no)\n";
    std::cout<<"\t--logfile, -l\t\t: Path to the status file in background mode (default: /tmp/ule.status)\n";
    exit(0);
}

static void readMacFile(ULEFramer& uleFramer, std::string& macsFile)
{
    std::ifstream macFile(macsFile);
    if(macFile.fail()) {
        throw std::runtime_error ("Could not open MAC/IP file");
    }

    uleFramer.clearDesintationAddressList();

    std::string type, ip, mac;
    while (macFile >> type >> ip >> mac) {
        struct sockaddr_storage ipStruct;
        uint8_t macArray[6];
        netMacToArray(macArray, mac);
        if(type == "4") {
            struct sockaddr_in* ipStruct4 = reinterpret_cast<struct sockaddr_in*>(&ipStruct);
            ipStruct4->sin_family = AF_INET;
            if(1 != inet_pton(AF_INET, ip.c_str(), &ipStruct4->sin_addr)) {
                throw std::runtime_error ("Failed parse IPv4 address");
            }

        } else if(type == "6") {
            struct sockaddr_in6* ipStruct6 = reinterpret_cast<struct sockaddr_in6*>(&ipStruct);
            ipStruct6->sin6_family = AF_INET6;
            if(1 != inet_pton(AF_INET6, ip.c_str(), &ipStruct6->sin6_addr)) {
                throw std::runtime_error ("Failed parse IPv6 address");
            }

        } else {
            throw std::runtime_error ("MAC/IP file parsing error");
        }

        if(!uleFramer.AddDestinationAddress(ipStruct, macArray)) {
            throw std::runtime_error ("Failed to add MAC/IP mapping");
        }
    }
}

bool doReload = false;
static void handleReload(int signum)
{
    doReload = true;
}

int main(int argc, char *argv[])
{
    std::string deviceName = "dvb0";
    std::string outputModuleName = "udp";
    std::string destination = "";
    std::string logFileName = "/tmp/ule.status";
    std::string macsFile = "";
    std::string symlinkPath = "";
    std::string gatewayMac = "";
    bool useTAP = false;
    bool useTimedMode = false;
    unsigned long bufferLen = 150000;
    unsigned long desiredOutputPacketLength = 0;
    unsigned long mtu = 0;
    unsigned int rtPriority = 60;

    unsigned long failedWrites = 0;
    unsigned long bytesWritten = 0;
    unsigned long mpegWritten = 0;
    unsigned long mpegWrites = 0;
    unsigned int dmaBuffers = 2;
    unsigned int mpegPacketLength = 188;
    unsigned int mpegPID = 0x666;
    unsigned long bytesWrittenDuringThisStatus = 0;
    bool background = false;
    bool sendNULL = false;

    TimerFD rateTimer(CLOCK_MONOTONIC);

    struct timeval now, lastStatus;
    lastStatus.tv_sec = 0;
    lastStatus.tv_usec = 0;

    while(1) {
        int option_index = 0;
        opterr = 0;

        int c = getopt_long (argc, argv, "hi:m:q:d:b:p:P:tBNM:l:x:g:o:n:T:L:", long_options, &option_index);
        if (c == -1) {
            break;
        }
        try {
            switch (c) {
                case 0:
                    break;
                case 'i':
                    deviceName = optarg;
                    break;
                case 'x':
                    symlinkPath = optarg;
                    break;
                case 'm':
                    mtu = std::stoll(optarg);
                    if(mtu <= 0) usage();
                    break;
                case 'q':
                    dmaBuffers = std::stoi(optarg);
                    break;
                case 'o':
                    outputModuleName = optarg;
                    strLowercase(outputModuleName);
                    break;
                case 'd':
                    destination = optarg;
                    break;
                case 'n':
                    mpegPacketLength = std::stoi(optarg);
                    if(mpegPacketLength>255) {
                        mpegPacketLength = 255;
                    }
                    break;
                case 'M':
                    macsFile = optarg;
                    break;
                case 'g':
                    gatewayMac = optarg;
                    break;
                case 'b':
                    bufferLen = std::stoll(optarg);
                    break;
                case 'p':
                    rtPriority = std::stoi(optarg);
                    if(rtPriority > 99) rtPriority = 99;
                    break;
                case 'P':
                    mpegPID = std::stoi(optarg);
                    if(mpegPID >= 0x1FFF) usage();
                    break;
                case 't':
                    useTAP = true;
                    break;
                case 'B':
                    background = true;
                    break;
                case 'l':
                    logFileName = optarg;
                    break;
                case 'N':
                    sendNULL = true;
                    break;
                case 'T':
                    useTimedMode = true;
                    rateTimer.setTimerRepeating(std::stoll(optarg) / 1000000, (std::stoll(optarg) % 1000000) * 1000);
                    break;
                case 'L':
                    desiredOutputPacketLength = std::stoi(optarg);
                    break;
                case 'h':
                case '?':
                    usage();
                    break;
                default:
                    abort();
            }
        } catch (std::exception& e) {
            usage();
        }
    }

    /* Make the output length a multiple of the message length */
    if(desiredOutputPacketLength) {
        desiredOutputPacketLength = desiredOutputPacketLength * mpegPacketLength;
    }


    try {
        OutputModule* outputModule;
        if(outputModuleName == "udp") {
            outputModule = new UDPOutput(destination, desiredOutputPacketLength);
        } else if(outputModuleName == "file") {
            outputModule = new FileOutput(destination, desiredOutputPacketLength);
        } else if(outputModuleName == "v4l2") {
            outputModule = new V4L2Output(destination, dmaBuffers, desiredOutputPacketLength);

        } else {
            throw std::invalid_argument ("Unknown output module. Please choose from: UDP, V4L2 or FILE");
        }
        TunTap virtualInterface(deviceName, useTAP);

        if(mtu) {
            if(!virtualInterface.setMTU(mtu)) {
                throw std::runtime_error("Could not set device MTU");
            }
        }

        mtu = virtualInterface.getMTU();

        if(bufferLen <= 2*mtu) {
            bufferLen = 2*mtu;
        }

        PacketBuffer leakingBuffer(bufferLen);
        ULEFramer uleFramer(leakingBuffer, mtu, mpegPID, sendNULL);

        if(symlinkPath != "") {
            if(symlink(outputModule->getDevicePath().c_str(), symlinkPath.c_str())) {
                throw std::runtime_error ("Could not create symbolic link");
            }
        }

        if(macsFile != "") {
            readMacFile(uleFramer, macsFile);
        }

        if(gatewayMac != "") {
            uint8_t gatewayMacArray[6];
            netMacToArray(gatewayMacArray, gatewayMac);
            uleFramer.setGatewayMac(gatewayMacArray);
        }

        netSetFdBlocking(virtualInterface.getFD(), false);
        netSetFdBlocking(outputModule->getFD(), false);
        netSetFdBlocking(rateTimer.getFD(), false);

        const unsigned int OUTPUT_MODULE_POLLFD = 0;
        const unsigned int VIRTUAL_INTERFACE_POLLFD = 1;
        const unsigned int TIMERFD_POLLFD = 2;

        struct pollfd pollfds[3];
        pollfds[OUTPUT_MODULE_POLLFD].fd = outputModule->getFD();
        pollfds[OUTPUT_MODULE_POLLFD].events = 0;
        pollfds[VIRTUAL_INTERFACE_POLLFD].fd = virtualInterface.getFD();
        pollfds[VIRTUAL_INTERFACE_POLLFD].events = POLLIN;
        pollfds[TIMERFD_POLLFD].fd = rateTimer.getFD();
        pollfds[TIMERFD_POLLFD].events = POLLIN;

        /* If we send NULL packets it is possible to have output without having received any packet first */
        if(sendNULL) {
            pollfds[OUTPUT_MODULE_POLLFD].events = POLLOUT;
        }

        if(useTimedMode) {
            pollfds[OUTPUT_MODULE_POLLFD].events = 0;
        }

        if(rtPriority) {
            std::cout<<"\nEnabling realtime scheduling.\n";
            setRealtimePriority(rtPriority, 32*1024);
            std::cout<<"\n";
        }

        std::cout << "System ready! Transmitting on PID "<<mpegPID<<".\n";

        if(background) {
            pid_t childPid = goBackground();
            if(childPid < 0) {
                throw std::runtime_error("Fork failed");
            } else if(childPid > 0) {
                std::cout << "Forked child with PID: "<<childPid<<".\n";
                exit(0);
            }
        }

        gettimeofday(&lastStatus, NULL);

        signal(SIGPIPE, SIG_IGN);
        signal(SIGUSR1, handleReload);

        while(1) {
            int retVal = poll(pollfds, sizeof(pollfds)/sizeof(struct pollfd), 250);
            if (retVal == -1 && errno != EINTR) {
                throw std::runtime_error("Poll failed");
            } else if (retVal == 0) {
                /* Timeout */
            } else {
                if ((pollfds[OUTPUT_MODULE_POLLFD].revents & POLLOUT) ||
                        (pollfds[TIMERFD_POLLFD].revents & POLLIN)) {
                    unsigned long iterations = 1, i;
                    if(useTimedMode) {
                        iterations = rateTimer.getOverflows();
                    }
                    for(i=0; i<iterations; i++) {
                        long bufferIndex = outputModule->getBuffer();
                        if(bufferIndex >= 0) {
                            unsigned long bufferSize;
                            unsigned long bufferOffset = 0;
                            uint8_t* dataPointer = outputModule->accessBuffer(bufferIndex, bufferSize);

                            while(bufferOffset <= bufferSize-mpegPacketLength) {
                                retVal = uleFramer.getMpegPacket(dataPointer+bufferOffset, mpegPacketLength);
                                if(retVal != (int)mpegPacketLength) {
                                    /* No more things to TX, wait for something to be put in the queue */
                                    pollfds[OUTPUT_MODULE_POLLFD].events = 0;
                                    break;
                                }
                                bufferOffset += retVal;
                                mpegWritten++;
                            }
                            bytesWritten+=bufferOffset;
                            bytesWrittenDuringThisStatus+=bufferOffset;

                            /* Do not do empty sends */
                            if(bufferOffset > 0) {
                                mpegWrites++;
                                if(outputModule->putBuffer(bufferIndex, bufferOffset) < 0) {
                                    failedWrites++;
                                }
                            }
                        } else {
                            std::cerr<<"WARNING: Could not get a buffer! Check driver and requested rate!\n";
                            pollfds[OUTPUT_MODULE_POLLFD].events = 0;
                            mpegWrites++;
                            failedWrites++;
                        }
                    }
                }
                if (pollfds[VIRTUAL_INTERFACE_POLLFD].revents & POLLIN) {
                    uint8_t inputBuffer[mtu+4];
                    for(;;) {
                        TunTap::PacketType packetType;
                        retVal = virtualInterface.readPacket(inputBuffer, mtu+4, packetType);
                        if(retVal<= 0) {
                            break;
                        }
                        if(packetType != TunTap::UNKNOWN && packetType != TunTap::UNICAST_MYADDR) {
                            /* Add some metadata */
                            inputBuffer[0] = useTAP;
                            inputBuffer[1] = packetType;
                            leakingBuffer.putPacket(inputBuffer, retVal + 4);
                            /* Some output will be produced */
                            if(!useTimedMode) {
                                pollfds[OUTPUT_MODULE_POLLFD].events = POLLOUT;
                            }
                        }
                    }
                }
            }

            gettimeofday(&now, NULL);
            float statusTimeDiff = (now.tv_usec - lastStatus.tv_usec)/1000000.0 + (now.tv_sec - lastStatus.tv_sec);
            if(statusTimeDiff >= 0.25) {
                lastStatus = now;
                std::ostream* infoOutput = &std::cout;
                std::ofstream fileOutput;
                if(background) {
                    fileOutput.open(logFileName);
                    infoOutput = &fileOutput;
                } else {
//                    clearConsole();
                }
                virtualInterface.doBookkeeping();
                virtualInterface.printIfStats(*infoOutput);
                *infoOutput<<"\n";
                leakingBuffer.printStatus(*infoOutput);
                *infoOutput<<"\n";
                outputModule->writeStats(*infoOutput);
                *infoOutput<<"\nFramer:\n";
                *infoOutput<<"\tSystem time: "<<now.tv_sec<<"\n";
                *infoOutput<<"\tMPEG packets written: "<<mpegWritten<<"\n";
                *infoOutput<<"\tMPEG bytes written: "<<bytesWritten<<"\n";
                *infoOutput<<"\tMPEG writes failed: "<<failedWrites<<"/"<<mpegWrites<<"\n";
                *infoOutput<<"\nRate control:\n";
                if(useTimedMode) {
                    *infoOutput<<"\tTimed mode\n";
                }
                if(sendNULL) {
                    *infoOutput<<"\tSending NULL packets\n";
                }
                unsigned long rate = (float)bytesWrittenDuringThisStatus/statusTimeDiff*8.0 / 1000.0;
                *infoOutput<<"\tBitrate: "+std::to_string(rate)+"kbps\n";
                if(doReload) {
                    *infoOutput<<"\nReloading data\n";
                    doReload = false;
                    if(macsFile != "") {
                        readMacFile(uleFramer, macsFile);
                    }
                }
                bytesWrittenDuringThisStatus = 0;
                if(background) {
                    fileOutput.close();
                }
            }
        }
    } catch(std::invalid_argument& e) {
        std::cout <<"Invalid argument: "<<e.what()<<".\n";
    } catch(std::runtime_error& e) {
        std::cout <<"Runtime error: "<<e.what()<<".\n";
    }
}

