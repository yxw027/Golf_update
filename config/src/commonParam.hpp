#ifndef COMMON_PARAM_HPP
#define COMMON_PARAM_HPP

#include "config.hpp"
#include "utility.hpp"

class commonParam
{
private:
	config_property property;
	bool flag_all_print;	// 障害物検出範囲を表示するかどうか
	char flag_mode[ 5 ];	// "r" or "w"
	FILE *fp_config;
	char WPfile[ STRLEN ];

	int readProperty( FILE *fp, char *path );

public:
	commonParam( void ) : flag_all_print( false ) { }
	~commonParam( void ) { }

	int initialize( char *path, bool flag, const char* mode );
	config_property getConfProperty( void ){ return property; }
	void setConfProperty( config_property c ){ property = c; }
	int printProperty( void );
	int saveProperty( void );
};

#endif
