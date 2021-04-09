#ifndef WP_HPP
#define WP_HPP

#define WP_SNAME	"wp_gl"

enum ROBOT_STATUS {
	START,
	NAVI,
//	AVOID,
	GOAL,
	POSITIONING,
//	CMD_WAITING,
	DUMMY
};

typedef struct {
	unsigned int id;
	double x, y, theta;
	double v;
	unsigned int flag_cut;
	int area_type;
	int gain_id;
} wp_gl;


#endif
