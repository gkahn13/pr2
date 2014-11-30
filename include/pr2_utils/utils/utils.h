#ifndef __PR2_UTILS_H__
#define __PR2_UTILS_H__

#include <unistd.h>
#include <termios.h>

namespace pr2_utils {

inline char getch() {
	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0) {
		perror("tcsetattr()");
	}
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0) {
		perror("tcsetattr ICANON");
	}
	if (read(0, &buf, 1) < 0) {
		perror ("read()");
	}
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0) {
		perror ("tcsetattr ~ICANON");
	}
	return (buf);
}

inline double smaller_angle(double angle) {
	return fmod(angle + M_PI, 2*M_PI) - M_PI;
}

inline double closer_angle(double x, double a) {
	return a + smaller_angle(x-a);
}

}

#endif
