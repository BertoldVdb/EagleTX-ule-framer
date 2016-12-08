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

#include "TimerFD.h"

#include <unistd.h>
#include <stdexcept>


TimerFD::TimerFD(int clockid):
    clockid_(clockid)
{

    fd_ = timerfd_create(clockid, 0);

    if(fd_ < 0) {
        throw std::runtime_error("Could not create timer");
    }

    stopTimer();
}

bool TimerFD::setTimer(int flags, time_t tv_sec, long tv_nsec, time_t tv_secInterval, long tv_nsecInterval)
{
    struct itimerspec it_spec;

    it_spec.it_interval.tv_nsec = tv_nsecInterval;
    it_spec.it_interval.tv_sec = tv_secInterval;

    it_spec.it_value.tv_sec = tv_sec;
    it_spec.it_value.tv_nsec = tv_nsec;

    if(timerfd_settime(fd_, flags, &it_spec, NULL)) {
        return false;
    }

    return true;
}

bool TimerFD::setTimerOnce(time_t tv_sec, long tv_nsec)
{
    return setTimer(0, tv_sec, tv_nsec, 0, 0);
}


bool TimerFD::setTimerOnceAbsolute(time_t tv_sec, long tv_nsec)
{
    return setTimer(TFD_TIMER_ABSTIME, tv_sec, tv_nsec, 0, 0);
}

bool TimerFD::setTimerRepeating(time_t tv_sec, long tv_nsec)
{
    return setTimer(0, tv_sec, tv_nsec, tv_sec, tv_nsec);
}

bool TimerFD::stopTimer()
{
    return setTimer(0, 0, 0, 0, 0);
}

uint64_t TimerFD::getOverflows()
{
    uint64_t overflows;
    ssize_t len = read(fd_, &overflows, sizeof(overflows));
    if(len != 8) {
        overflows = 0;
    }
    return overflows;
}

TimerFD::~TimerFD()
{
    close(fd_);
}

