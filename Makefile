#Copyright (c) 2016, Bertold Van den Bergh
#All rights reserved.
#This work has been developed to support research funded by
#"Fund for Scientific Research, Flanders" (F.W.O.-Vlaanderen).
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the author nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR DISTRIBUTOR BE LIABLE FOR ANY
#DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


CCARCH=
CC=$(CCARCH)gcc
CPP=$(CCARCH)g++
STRIP=$(CCARCH)strip

CFLAGS=-std=c++1y -O3 -Wall -c -fmessage-length=0 -Werror
LDFLAGS=

EXECUTABLE=uleframer
INCLUDES=FileOutput.h OutputModule.h PacketBuffer.h TimerFD.h TunTap.h UDPOutput.h ULEFramer.h Util.h V4L2Output.h
SOURCES=CRC32.cpp FileOutput.cpp Main.cpp PacketBuffer.cpp TimerFD.cpp TunTap.cpp UDPOutput.cpp ULEFramer.cpp Util.cpp V4L2Output.cpp


OBJECTS_OBJ=$(addprefix obj/,$(SOURCES:.cpp=.o))
INCLUDES_SRC=$(addprefix src/,$(INCLUDES))
SOURCES_SRC=$(addprefix src/,$(SOURCES))

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS_OBJ)
	$(CPP) $(LDFLAGS) $(OBJECTS_OBJ) -o $@
	$(STRIP) $@
	

obj/%.o: src/%.cpp $(INCLUDES_SRC)
	@mkdir -p $(@D)
	$(CPP) $(CFLAGS) $< -o $@

clean:
	rm -r $(OBJECTS_OBJ) $(EXECUTABLE) obj/

