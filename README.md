 An Unidirectional Lightweight Encapsulation (ULE) protocol framer for satcom systems. — 
# ULEFramer

This program creates a virtual network interface to use together with a satellite modulator. It encapsulates the Ethernet or IP packets into an MPEG Transport Stream using the ULE (RFC 4326) protocol. The Transport Stream can be output via Video4Linux2, UDP and a FIFO file. This should cover most scenarios.

The following options are supported:
```
	--help, -h              : Prints this help.
	--interface, -i         : Interface name (default: dvb0)
	--mtu, -m               : Maximum packet size. Sets device MTU! (default: current MTU)
	--outputmodule, -o      : Output module to use! (default: udp)
	--destination, -d       : MPEG-TS destination. Depends on output module.
	--symlink, -x           : Create a symbolic link to the device (default: none)
	--buffer, -b            : Size of packet buffer (default: 15000)
	--queue, -q             : Buffer queue length (default: 2)
	--priority, -p          : Set realtime priority (default: 60)
	--pid, -P               : MPEG-TS PID (default: 0x666)
	--mpegts, -n            : MPEG-TS packet size (default: 188)
	--tap, -t               : Use Ethernet bridging (default: no)
	--timed, -T             : Use timed mode. Produce output every n us. (default: backpressure mode)
	--outputlen, -L         : Number of MPEG-TS packets per output (default: optimal number)
	--null, -N              : Inject PID 0x1FFF packets (default: no)
	--macsfile, -M          : File with IP/MAC mappings (default: not used)
	--gateway, -g           : Default gateway in TUN mode (default: not set)
	--background, -B        : Go to background (default: no)
	--logfile, -l           : Path to the status file in background mode (default: /tmp/ule.status)
```
