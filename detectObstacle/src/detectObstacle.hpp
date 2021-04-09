#ifndef DETECT_OBSTACLE_HPP
#define DETECT_OBSTACLE_HPP

#define OBP_SNAME "obp_fs"

#include <ssmtype/spur-odometry.h>

enum DETECT_STATUS {
	TRAVELING,
	DECELERATION,
	STOP
};

typedef struct {
	enum DETECT_STATUS status;
	double pos[ 3 ];
} obp_fs;	// OBstacle Point


#endif
