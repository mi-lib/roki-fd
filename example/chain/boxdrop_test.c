#include <roki_fd/roki_fd.h>

#define T   10.0
#define DT  0.001

#define NMAX  9
#define N     4

int main(int argc, char *argv[])
{
  rkFD fd;
  int i, n;
  rkFDCell *cell[NMAX];
  zVec dis[NMAX];
  FILE *fp[NMAX];
  char name[BUFSIZ];

  /* zRandInit(); */
  rkFDCreate( &fd );
  rkFDContactInfoScanFile( &fd, "../model/contactinfo.ztk" );

  n = argc > 1 ? atoi( argv[1] ) : N;
  if( n > NMAX ) n = NMAX;
  for( i=0; i<n; i++ ){
    sprintf( name, "%d.zvs", i+1 );
    fp[i] = fopen( name, "w" );
    cell[i] = rkFDChainRegFile( &fd, "../model/box.ztk" );
    dis[i] = zVecAlloc( rkChainJointSize(rkFDCellChain(cell[i])) );
    zVecElemNC(dis[i],0) = zRandF( -0.1, 0.1 );
    zVecElemNC(dis[i],1) = zRandF( -0.1, 0.1 );
    zVecElemNC(dis[i],2) = 0.1 + i*0.15;
    zVecElemNC(dis[i],3) = zDeg2Rad(zRandF(-90.0, 90.0));
    zVecElemNC(dis[i],4) = zDeg2Rad(zRandF(-90.0, 90.0));
    zVecElemNC(dis[i],5) = zDeg2Rad(zRandF(-90.0, 90.0));

    rkFDChainSetDis( cell[i], dis[i] );
    rkCDPairChainUnreg( rkFDCDBase(&fd.cd), rkFDCellChain(cell[i]) );
  }
  rkFDChainRegFile( &fd, "../model/floor.ztk" );

  /* ode */
  rkFDODE2Assign( &fd, Regular );
  rkFDODE2AssignRegular( &fd, RKG );
  rkFDPrpSetDT( &fd, DT );

  /* solver */
  rkFDSetSolver( &fd, Volume );
/*
  rkFDSetSolver( &fd, MLCP );
  rkFDSetSolver( &fd, Vert );
*/
  rkFDUpdateInit( &fd );
  while( rkFDTime(&fd) < T ){
    eprintf( "t = %f\n", rkFDTime(&fd) );
    rkFDUpdate( &fd );
    for( i=0; i<n; i++ ){
      rkChainGetJointDisAll( rkFDCellChain(cell[i]), dis[i] );
      fprintf( fp[i], "%f ", rkFDDT(&fd) );
      zVecFPrint( fp[i], dis[i] );
    }
  }
  rkFDUpdateDestroy( &fd );

  for( i=0; i<n; i++ ){
    zVecFree( dis[i] );
    fclose( fp[i] );
  }
  rkFDDestroy( &fd );
  return 0;
}
