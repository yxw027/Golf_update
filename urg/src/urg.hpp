#ifndef URG_HPP
#define URG_HPP

#include <scip2awd.h>

#define URG_SNAME "urg_fs"

#define URG_DATA_MAX 1080	// = 270[deg] / 0.25[deg]

typedef struct
{
	unsigned int size;		// ポイント数 (=URG_DATA_MAX)
	double length[ URG_DATA_MAX ];
	double angle[ URG_DATA_MAX ];
    unsigned intensity[ URG_DATA_MAX ];	// 受信強度
} urg_fs;

typedef struct {
	char vender[ SCIP2_MAX_LENGTH ];
	char product[ SCIP2_MAX_LENGTH ];
	char firmware[ SCIP2_MAX_LENGTH ];
	char protocol[ SCIP2_MAX_LENGTH ];
	char serialno[ SCIP2_MAX_LENGTH ];
	char model[ SCIP2_MAX_LENGTH ];
	int dist_min;
	int dist_max;
	int step_resolution;
	int step_min;
	int step_max;
	int step_front;
	int revolution;
} urg_property;

#endif // URG_HPP
