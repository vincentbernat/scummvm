/* ScummVM - Graphic Adventure Engine
 *
 * ScummVM is the legal property of its developers, whose names
 * are too numerous to list here. Please refer to the COPYRIGHT
 * file distributed with this source distribution.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */

/* OPL implementation for OPL2 Audio Board for Arduino, through the serial port.
 */

#define FORBIDDEN_SYMBOL_ALLOW_ALL
#include "common/scummsys.h"

#include "common/config-manager.h"
#include "common/debug.h"
#include "common/str.h"
#include "common/textconsole.h"
#include "audio/fmopl.h"
#include "SDL_thread.h"
#include "SDL_mutex.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

namespace OPL {
namespace OPL2Arduino {

class OPL : public ::OPL::RealOPL {
private:
	int port;
	int index;		// current register
	SDL_sem *sem;
	SDL_Thread *thread;

	bool configureSerial(Common::String serialPortName);
	void writeReg(int a, int v);

public:
	OPL();
	~OPL();

	bool init();
	void reset();

	void write(int a, int v);
	byte read(int a);

	int ackThread();
};
}
}

static void opl2arduino_shutdown(int port) {
	if (port != -1) {
		close(port);
	}
}

static void opl2arduino_write(int port, byte d, byte c) {
	if (port != -1) {
		/* The command format is:
		 *   - byte 1: register
		 *   - byte 2: value
		 *   - remaining: delay.
		 *
		 * No delay is needed because ScummVM takes care of them and
		 * the code running on the Arduino already sleeps for 3.3 µs
		 * and 24 µs. */
		byte cmd[5] = {d, c, 0, 0, 0};
		write(port, cmd, sizeof(cmd));
	}
}

static bool opl2arduino_ack(int port) {
	if (port != -1) {
		byte ack;
		ssize_t n;
		n = read(port, &ack, 1);
		if (n == 1) {
			if (ack != 'k') {
				warning("OPL2Arduino: expected ack, got 0x%02x", ack & 0xff);
				return false;
			}
			return true;
		}

		/* Maybe the serial port was closed? */
		if (n == 0 || (n == -1 && errno == EBADF)) {
			warning("OPL2Arduino: terminating ack thread");
			return false;
		}

		/* Other errors */
		warning("OPL2Arduino: cannot get ack: %s",
			strerror(errno));
		return false;
	}
	return true;
}

static int opl2arduino_ack_thread(void *ptr) {
	OPL::OPL2Arduino::OPL *self = static_cast<OPL::OPL2Arduino::OPL *>(ptr);
	return self->ackThread();
}

static void opl2arduino_reset(int port) {
	warning("OPL2Arduino: reset OPL2 chip");
	if (port != -1) {
		byte cmd[5] = {};
		write(port, cmd, sizeof(cmd));
	}
}

static bool opl2arduino_configure_serial(const char *name) {
	struct termios tty = {};
	int fd = open(name, O_RDWR);
	if (fd == -1) {
		warning("OPL2Arduino: cannot open serial port: %s", strerror(errno));
		return false;
	}
	if (tcgetattr(fd, &tty) != 0) {
		warning("OPL2Arduino: cannot get serial port attributes: %s", strerror(errno));
		close(fd);
		return false;
	}
	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		warning("OPL2Arduino: cannot set serial port attributes: %s", strerror(errno));
		close(fd);
		return false;
	}
	close(fd);
	return true;
}

static int opl2arduino_init(const char *name, int *max_write) {
	int fd;
	char cmd[5];
	char digit;

	opl2arduino_configure_serial(name);
	fd = open(name, O_RDWR);
	if (fd == -1) {
		warning("OPL2Arduino: cannot open serial port \"%s\": %s",
			name, strerror(errno));
		return -1;
	}

	// Wait for HLO!
	if (read(fd, cmd, sizeof(cmd)) != sizeof(cmd)) {
		warning("OPL2Arduino: short read (1): %s", strerror(errno));
		goto error;
	}
	if (strncmp("HLO!\n", cmd, sizeof(cmd))) {
		warning("OPL2Arduino: expected `HLO!', got `%s'", cmd);
		goto error;
	}

	// Send BUF? and wait for buffer size
	*max_write = 0;
	write(fd, "BUF?\n", 5);
	while (true) {
		if (read(fd, &digit, 1) != 1) {
			warning("OPL2Arduino: short read (2): %s",
				strerror(errno));
			goto error;
		}
		if (digit >= '0' && digit <= '9') {
			*max_write *= 10;
			*max_write += digit - '0';
		} else if (digit == '\n') {
			break;
		} else {
			warning("OPL2Arduino: cannot read buffer size (got 0x%02x)",
				digit & 0xff);
			goto error;
		}
	}
	*max_write /= 5; // maximum number of commands we can pipeline
	warning("OPL2Arduino: buffer is %d commands", *max_write);
	return fd;
error:
	close(fd);
	return -1;
}

namespace OPL {
namespace OPL2Arduino {

OPL::OPL() : port(-1) {
}

OPL::~OPL() {
	opl2arduino_reset(port);
	opl2arduino_shutdown(port);
	SDL_WaitThread(thread, nullptr);

	// Display last semaphore value. If different of
	// buffer size, we may have lost some acks.
	warning("OPL2Arduino: last sem value is %d", SDL_SemValue(sem));
	SDL_DestroySemaphore(sem);
}

bool OPL::init() {
	const Common::String portName = ConfMan.get("opl2arduino_device");

	int max_write = 0;
	index = 0;
	port = opl2arduino_init(portName.c_str(), &max_write);
	if (port != -1) {
		sem = SDL_CreateSemaphore(max_write);
		if (!sem) {
			warning("OPL2Arduino: unable to create semaphore: %s",
				SDL_GetError());
			opl2arduino_shutdown(port);
			port = -1;
		}
		thread = SDL_CreateThread(opl2arduino_ack_thread, "OPL2 ack thread", this);
		if (!thread) {
			warning("OPL2Arduino: unable to create ack thread: %s",
				SDL_GetError());
			opl2arduino_shutdown(port);
			port = -1;
		}
	}
	return (port != -1);
}

void OPL::reset() {
	opl2arduino_reset(port);
}

void OPL::write(int ioport, int val) {
	if (ioport & 1) {
		/* 0x389: write the received value to the current register */
		writeReg(index, val);
	} else {
		/* 0x388: change the current register */
		index = val;
	}
}

byte OPL::read(int ioport) {
	// No read support for the OPL2 audio board
	return 0;
}

void OPL::writeReg(int r, int v) {
	/* Write the provided value to the provided register */
	r &= 0xff;
	v &= 0xff;
	if (SDL_SemTryWait(sem) == SDL_MUTEX_TIMEDOUT) {
		warning("OPL2Arduino: too slow, consider increasing buffer");
		SDL_SemWait(sem);
	}
	opl2arduino_write(port, r, v);
}

int OPL::ackThread() {
	while (true) {
		if (!opl2arduino_ack(port)) {
			warning("OPL2Arduino: end of ack thread");
			break;
		}
		SDL_SemPost(sem);
	}
	return 0;
}

OPL *create() {
	return new OPL();
}

} // End of namespace OPL2Arduino
} // End of namespace OPL
