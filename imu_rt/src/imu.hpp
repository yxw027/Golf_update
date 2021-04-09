#ifndef IMU_HPP
#define IMU_HPP

#define IMU_SNAME "imu_fs"

typedef struct {
	bool status;	// 1:true. 2:false
	// Raw data
	double angvel[ 3 ];
	double accel[ 3 ];
	double mag[ 3 ];
	double temperature;
	// 推定した角度
	double estAng[ 3 ];
} imu_fs;

typedef struct {
	double gravity;
	double initAng[ 2 ];	// 設置角度 (roll, pitch)
	double angvel_offset[ 3 ];	// 角速度のオフセット
	double K_Coef;	// 相補フィルターの係数
	
} imu_property;

#endif // IMU_HPP
