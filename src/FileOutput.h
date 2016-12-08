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


#ifndef SRC_FILEOUTPUT_H_
#define SRC_FILEOUTPUT_H_

#include <stdint.h>
#include <string>
#include <ostream>
#include "OutputModule.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdexcept>
#include <fcntl.h>

class FileOutput : public OutputModule
{
public:
    FileOutput(std::string& path, unsigned long bufferSize);
    virtual ~FileOutput();

    long getBuffer() {
        return 0;
    }

    int putBuffer(unsigned long buffer, unsigned long bytesUsed) {
        return write(fd_, buffer_, bytesUsed);
    }

    void writeStats(std::ostream& cout) {
        cout << "File:\n\tPath: "+getDevicePath()+"\n";
    }

    uint8_t* accessBuffer(unsigned long buffer, unsigned long& size) {
        size = bufferSize_;
        return buffer_;
    }

    int getFD() {
        return fd_;
    }

    /* The socket has no path */
    std::string getDevicePath() {
        return path_;
    }

private:
    std::string path_;
    int fd_ = -1;

    uint8_t* buffer_ = NULL;
    unsigned long bufferSize_;

    void cleanup();
};

#endif /* SRC_FILEOUTPUT_H_ */
