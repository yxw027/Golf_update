 /*
  * Data : 2021.04.06
  * Author : T.Hasegawa
  */
#ifndef _CREATE_WP_HPP_
#define _CREATE_WP_HPP_

#include <stdio.h>
#include <stdlib.h>
#include "utility.hpp"
#include "localizer.hpp"
#include "wp.hpp"

class createWP_Base
{
protected:
	FILE *savefp;
	FILE *temp_fp;
	char path[ STRLEN ];
	wp_gl wp;

	double distThre;
	double angThre;
	double initPose[ 3 ];
	unsigned int wp_size;
	
	void openWPfile( const char *file, char *p, const char* mode );
	void writeInitPose( void );
	void writeWPsize( void );
	
public:
	createWP_Base( void ) : distThre( 0.3 ), angThre( M_PI/4 ), wp_size( 0 ) { }
	~createWP_Base( void ) { fclose( temp_fp ); fclose( savefp ); }

	bool setPose( localizer *data );
	wp_gl getWP( void ){ return wp; }
	void initialize( char *path );
	
	virtual void saveWPFile( void );
};

#endif


