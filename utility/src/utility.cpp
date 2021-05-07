#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdexcept>
//#include <cmath>
//#include <Eigen/Core>
//#include <Eigen/Eigen>
//#include <Eigen/SVD>
#include "utility.hpp"

using namespace std;

void Gprint( const char *str )
{
	printf( "\x1b[32m\x1b[1m%s\x1b[39m\x1b[0m", str );
}
void Rprint( const char *str )
{
	printf( "\033[1m\033[31m%s\033[30m\033[0m", str );
}

//*******************************
// ファイル名がcsvファイルになっているか確認
// 引数 char* filename : ファイル名
// 戻値 int : EXIT_FAILUREの場合、csvファイルを指定していない
//            EXIT_SUCCESSの場合、csvファイル
//*******************************
static int isValidFileName( const char* filename, const char* ext )
{
	char str[ 10 ], tmp[ 50 ];
	int len = strlen( filename );
//std::cout << len << std::endl;
	for( int i = 0 ; i < len ; i++ ){
		tmp[ i ] = filename[ i ];
	}
	len = strcspn( tmp, "." );
	if( len < 3 ){
		tmp[ 0 ] = 'a';
		tmp[ 1 ] = 'a';
		len = strcspn( tmp, "." );
	}
	if( len < 6 ){
		tmp[ 0 ] = 'a';
		tmp[ 1 ] = 'a';
		tmp[ 2 ] = 'a';
		tmp[ 3 ] = 'a';
		tmp[ 4 ] = 'a';
		len = strcspn( tmp, "." );
	}
//	std::cout << len << std::endl;
	for( int i = 0 ; i < 5 ; i++ ){
		str[ i ] = filename[ len+i+1 ];
	}
	if( strncmp( ext, str, 5 ) ){
		fprintf( stderr, "Cannot make .%s file\n", ext );
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
//*******************************
// ファイル名をオープンする関数
// 引数 char* filename : ファイル名
// 戻値 FILEポインタ
//*******************************
FILE* isValidFile( const char* filename, const char* mode, const char* ext )
{
  FILE *fp1;

  if( isValidFileName( filename, ext ) == EXIT_FAILURE ) exit( EXIT_FAILURE );
  if( ( fp1 = fopen( filename, mode ) ) == NULL ){
    fprintf( stderr, "Cannot open %s\n", filename );
    exit( EXIT_FAILURE );
  }

  return fp1;
}
//*******************************
// ファイルからデータをchar型で返す関数
// 引数 FILEポインタ
// 戻値 char型のデータ
//*******************************
#define MaxLine 3000
#define CmtChar '#'
static char linebuf[MaxLine] = {'\0'};
static char *cur_ptr = linebuf;
static int skipComment( FILE* fp, char* buf, int size )
{
  do {
    if( !fgets( buf, size, fp ) ) return EOF;
  } while( buf[ 0 ] == CmtChar );
  return EXIT_SUCCESS ;
}
char* getWord( FILE *fp )
{
  char *ptr;
  while( isspace( *cur_ptr ) || *cur_ptr == '\0' ){
    if( *cur_ptr == '\0' ){
      if( skipComment( fp, linebuf, MaxLine ) == EOF ) return NULL;
      cur_ptr = linebuf;
    } else cur_ptr++;
  }
  ptr = cur_ptr;
  while( !isspace( *cur_ptr ) && *cur_ptr != '\0' ){
    if( *cur_ptr == '\0' ){
      if( skipComment( fp, linebuf, MaxLine ) == EOF) break;
      cur_ptr = linebuf;
    } else cur_ptr++;
  }
  if( *cur_ptr != '\0' ) *cur_ptr++ = '\0';
  return ptr;
}

/*-PI < theta < PIに調整する*/
double trans_q( double theta )
{
    while( theta > M_PI )
        theta -= 2.0 * M_PI;
    while( theta < -M_PI )
        theta += 2.0 * M_PI;
    return theta;
}

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
bool kbhit( void )
{
	struct termios oldt, newt;
//	int ch;
	int oldf;

	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	oldf = fcntl( STDIN_FILENO, F_GETFL, 0 );
	fcntl( STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK );

	int ch = getchar( );

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	fcntl( STDIN_FILENO, F_SETFL, oldf );

	if( ch != EOF ){
		ungetc( ch, stdin );
		return true;
	}

	return false;
}
