#include <roki-fd/rokifd.h>

#define T 5
#define DT 0.001
#define DTC 0.002

#define K 12000
#define C 120

void control(rkFDCell *cell)
{
  register int i;
  double dis, vel, e;
  rkJoint *joint;

  for( i=0; i<rkChainNum(rkFDCellChain(cell)); i++ ){
    joint = rkChainLinkJoint(rkFDCellChain(cell),i);
    rkJointGetDis( joint, &dis );
    rkJointGetVel( joint, &vel );
    e = -K*(dis-zDeg2Rad(90)) - C*vel;
    rkJointMotorSetInput( joint, &e );
  }
}

int main(int argc, char *argv[])
{
  rkFD fd;
  rkFDCell *cell[2];
  zVec dis[2];
  FILE *fp[2];
  double t_cnt;

  fp[0] = fopen( "arm.zvs", "w" );
  fp[1] = fopen( "box.zvs", "w" );

  rkFDCreate( &fd );
	rkFDContactInfoReadFile( &fd, "../model/contactinfo.ztk" );

  cell[0] = rkFDChainRegFile( &fd, "../model/arm_2DoF_trq.ztk" );
  cell[1] = rkFDChainRegFile( &fd, "../model/box.ztk" );
  rkFDChainRegFile( &fd, "../model/floor.ztk" );

  /* init */
  /* chain 0 */
  dis[0] = zVecAlloc(rkChainJointSize(rkFDCellChain(cell[0])));
  zVecElemNC(dis[0],0) = zDeg2Rad(90);
  zVecElemNC(dis[0],1) =-zDeg2Rad(90);
  rkFDChainSetDis( cell[0], dis[0] );
  rkCDPairChainUnreg( rkFDCDBase(&fd.cd), rkFDCellChain(cell[0]));
  /* chain 1 */
  dis[1] = zVecAlloc(rkChainJointSize(rkFDCellChain(cell[1])));
  zVecElemNC(dis[1],0) = 0.02;
  zVecElemNC(dis[1],1) = 0.6;
  zVecElemNC(dis[1],2) = 0.2;
  rkFDChainSetDis( cell[1], dis[1] );

  /* ode */
  rkFDODE2Assign( &fd, Regular );
  rkFDODE2AssignRegular( &fd, RKG );
  rkFDPrpSetDT( &fd, DT );

  /* solver */
  /* rkFDSetSolver( &fd, MLCP ); */
  rkFDSetSolver( &fd, Volume );

  rkFDUpdateInit( &fd );

  t_cnt = rkFDTime(&fd);
  while( rkFDTime(&fd) < T ){
    if( t_cnt <= rkFDTime(&fd) ){
      control( cell[0] );
      t_cnt += DTC;
    }
    eprintf( "t = %f\n", rkFDTime(&fd) );
    rkFDUpdate( &fd );
    rkChainGetJointDisAll( rkFDCellChain(cell[0]), dis[0] );
    fprintf( fp[0], "%f ", rkFDDT(&fd) );
    zVecFWrite( fp[0], dis[0] );
    rkChainGetJointDisAll( rkFDCellChain(cell[1]), dis[1] );
    fprintf( fp[1], "%f ", rkFDDT(&fd) );
    zVecFWrite( fp[1], dis[1] );
  }
  rkFDUpdateDestroy( &fd );

  zVecFreeAO( 2, dis[0], dis[1] );
  rkFDDestroy(&fd);
  fclose( fp[0] );
  fclose( fp[1] );
  return 0;
}
