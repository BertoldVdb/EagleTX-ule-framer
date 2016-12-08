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

#ifndef SRC_V4L2OUTPUT_H_
#define SRC_V4L2OUTPUT_H_

#include <string>
#include "OutputModule.h"


class V4L2Output : public OutputModule
{
public:
    V4L2Output(std::string deviceName, unsigned int bufCount, unsigned long bufSize);
    ~V4L2Output();

    long getBuffer();
    int putBuffer(unsigned long buffer, unsigned long bytesUsed);
    void writeStats(std::ostream& cout);

    uint8_t* accessBuffer(unsigned long buffer, unsigned long& size) {
        size = buffers[buffer].length;
        if(overrideBufSize_ && overrideBufSize_<size) {
            size = overrideBufSize_;
        }
        return (uint8_t*)buffers[buffer].addr;
    }

    int getFD() {
        return outFd_;
    }

    std::string getDevicePath() {
        return devicePath_;
    }

private:
    int manageRecovery();

    void cleanup();
    int openDevice(std::string& deviceName);

    int outFd_ = -1;
    unsigned long overrideBufSize_;
    std::string devicePath_;

    void initMmap(unsigned int bufCount);

    struct buffer {
        bool valid = false;
        bool active = false;
        void*  addr;
        size_t length;
    };
    struct buffer* buffers = NULL;
    unsigned long numBufs = 0;

};

#endif /* SRC_V4L2OUTPUT_H_ */
