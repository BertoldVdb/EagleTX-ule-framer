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

#include "V4L2Output.h"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <exception>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>



int V4L2Output::openDevice(std::string& deviceName)
{
    DIR *d;
    struct dirent *dir;

    if(deviceName[0]=='/') {
        devicePath_ = deviceName[0];
        outFd_ = open(devicePath_.c_str(), O_RDWR);
        if(outFd_ >= 0) {
            return outFd_;
        }
    } else {
        std::string pathV4L = "/sys/devices/" + deviceName + "/video4linux";

        /* Find V4L device name */
        d = opendir(pathV4L.c_str());

        if(!d) throw std::runtime_error("Device not found");

        while ((dir = readdir(d)) != NULL) {
            if(strcmp(dir->d_name, ".") && strcmp(dir->d_name, "..")) {
                std::string devPath = "/dev/";
                devPath += dir->d_name;

                outFd_ = open(devPath.c_str(), O_RDWR);
                if(outFd_ >= 0) {
                    devicePath_ = devPath;
                    return outFd_;
                }
            }
        }
    }

    throw std::runtime_error("Could not find V4L device");
}

V4L2Output::V4L2Output(std::string deviceName, unsigned int bufCount, unsigned long bufSize):
    overrideBufSize_(bufSize)
{
    enum v4l2_buf_type type;

    try {
        if(deviceName == "") {
            deviceName = "/dev/video0";
        }

        outFd_ = openDevice(deviceName);

        initMmap(bufCount);

        type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        if (ioctl(outFd_, VIDIOC_STREAMON, &type) < 0) {
            throw std::runtime_error("Could not start streaming pipe");
        }

    } catch(...) {
        cleanup();
        throw;
    }
}

V4L2Output::~V4L2Output()
{
    cleanup();
}

void V4L2Output::cleanup()
{
    enum v4l2_buf_type type;

    if(outFd_ >=0) {
        type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ioctl(outFd_, VIDIOC_STREAMOFF, &type);
        close(outFd_);
    }

    if(buffers) {
        for (unsigned int i = 0; i < numBufs; i++) {
            if(buffers[i].valid) {
                munmap(buffers[i].addr, buffers[i].length);
            }
        }

        delete buffers;
    }
}

int V4L2Output::manageRecovery()
{
    struct v4l2_buffer buf;

    /* Try to dequeue one */
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_MMAP;
    int retVal;
    if ((retVal = ioctl(outFd_, VIDIOC_DQBUF, &buf)) < 0) {
        return retVal;
    }

    /* Got a buffer back */
    buffers[buf.index].active = false;
    return 0;
}

long V4L2Output::getBuffer()
{
    while(!manageRecovery());

    /* Search for a buffer that is not queued */
    unsigned long i;
    for(i=0; i<numBufs; i++) {
        if(!buffers[i].active) {
            return i;
        }
    }

    return -1;
}

void V4L2Output::writeStats(std::ostream& cout)
{
    cout << "V4L2:\n\tBuffers: ";

    unsigned long i;
    for(i=0; i<numBufs; i++) {
        if(buffers[i].active) {
            cout << "["<<i<<"] ";
        } else {
            cout << " "<<i<<"  ";
        }
    }

    cout <<"\n";
}

int V4L2Output::putBuffer(unsigned long buffer, unsigned long bytesUsed)
{
    struct v4l2_buffer buf;

    if(bytesUsed == 0) {
        return 0;
    }

    if(buffers[buffer].active) {
        return -1;
    }

    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buffer;
    buf.bytesused = bytesUsed;
    buf.length = buffers[buffer].length; /* Not needed, I think */

    /* Fire */
    int retVal;
    if((retVal = ioctl(outFd_, VIDIOC_QBUF, &buf)) >= 0) {
        buffers[buffer].active = true;
    } else {
        std::cerr << "Failed to queue buf: "<<strerror(errno)<<"\n";
    }
    return retVal;
}

void V4L2Output::initMmap(unsigned int bufCount)
{
    struct v4l2_requestbuffers req;
    struct v4l2_format format;
    memset(&req, 0, sizeof(req));
    memset(&format, 0, sizeof(format));

    /* First we should set the format and desired buffersize */
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(outFd_, VIDIOC_G_FMT, &format) < 0) {
        throw std::runtime_error("Failed to query format");
    }

    /*
     * The actual buffer may be bigger (page aligned) but only this much may be written
     * to it.
     */
    unsigned long bufferSize = format.fmt.pix.sizeimage;

    req.count = bufCount;
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(outFd_, VIDIOC_REQBUFS, &req) < 0) {
        throw std::runtime_error("Failed to request buffers");
    }

    if (req.count == 0) {
        throw std::runtime_error("Requested buffers, but got none");
    }

    buffers = new struct buffer[req.count];

    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));

        buf.type        = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        if (ioctl(outFd_, VIDIOC_QUERYBUF, &buf) << 0) {
            throw std::runtime_error("Queried buffer, but it failed");
        }

        buffers[i].addr = mmap(NULL, buf.length,
                               PROT_READ | PROT_WRITE,
                               MAP_SHARED, outFd_,
                               buf.m.offset);
        buffers[i].length = buf.length;

        if (buffers[i].addr == MAP_FAILED) {
            throw std::runtime_error("Couldn't MMAP buffer");
        }

        buffers[i].valid = true;
    }

    std::cout << "Mapped "<<req.count<< " buffers containing "<<bufferSize<<" bytes each\n";
    numBufs = req.count;
}
