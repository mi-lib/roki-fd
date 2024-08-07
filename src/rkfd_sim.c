/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_sim - forward dynamics simulation
 * contributer: 2014- Naoki Wakisaka
 */

/* ********************************************************** */
/* CLASS: rkFD
   forward dynamics class
 * ********************************************************** */
#include <roki_fd/roki_fd.h>
#include <roki_fd/rkfd_util.h>
#include <roki/rk_abi.h>

#define rkChainUpdateRateGrav(c) \
  rkLinkUpdateRate( rkChainRoot(c), ZVEC6DZERO, ZVEC6DZERO )

void _rkFDCellDatFree(rkFDCellDat *ld)
{
  rkChainDestroy( rkFDCellDatChain(ld) );
}

void _rkFDSolverCreate(rkFD *fd)
{
  rkFDSolverInit( &fd->solver );
  fd->solver.t     = 0;
  fd->solver.fdprp = &fd->prp;
  fd->solver.cd    = &fd->cd;
}

rkFD *rkFDCreate(rkFD *fd)
{
  fd->t = 0.0;
  rkFDPrpInit( &fd->prp );
  zListInit( &fd->list );
  zArrayInit( &fd->ci );
  rkFDCDInit( &fd->cd );

  fd->size = 0;
  fd->dis = NULL;
  fd->vel = NULL;
  fd->acc = NULL;

  /* ODE solver */
  rkFDODE2Assign( fd, Regular );
  rkFDODE2AssignRegular( fd, RKG );
  fd->ode_step = 0;

  /* default collision solver */
  _rkFDSolverCreate( fd );
  rkFDSetSolver( fd, Vert );
  return fd;
}

void rkFDDestroy(rkFD *fd)
{
  rkFDCell *lc;

  rkFDSolverDestroy( &fd->solver );
  zVecFreeAtOnce( 3, fd->dis, fd->vel, fd->acc );
  rkFDCDDestroy( &fd->cd );
  rkContactInfoArrayDestroy( &fd->ci );
  while( !zListIsEmpty( &fd->list ) ){
    zListDeleteHead( &fd->list, &lc );
    _rkFDCellDatFree( &lc->data );
    zFree( lc );
  }
  rkFDPrpDestroy( &fd->prp );
}

void _rkFDCellDatSetOffset(rkFD *fd, rkFDCellDat *ld, int offset)
{
  ld->_offset = offset;
  zVecBufNC(&ld->_dis) = &zVecElemNC(fd->dis,offset);
  zVecBufNC(&ld->_vel) = &zVecElemNC(fd->vel,offset);
}

bool _rkFDAllocJointStatePush(rkFD *fd, rkFDCell *rlc)
{
  rkFDCell *lc;
  int size, offset = 0;
  zVec pdis, pvel;

  if( (size = rkChainJointSize( rkFDCellChain(rlc))) == 0 ) return true;
  pdis = fd->dis;
  pvel = fd->vel;
  zVecFree( fd->acc );
  if( !( fd->dis = zVecAlloc( fd->size+size ) ) ||
      !( fd->vel = zVecAlloc( fd->size+size ) ) ||
      !( fd->acc = zVecAlloc( fd->size+size ) ) ){
    fd->size = 0;
    ZALLOCERROR();
    zVecFreeAtOnce( 5, pdis, pvel, fd->dis, fd->vel, fd->acc );
    return false;
  }

  if( fd->size ){
    zRawVecCopy( zVecBufNC(pdis), zVecBufNC(fd->dis), fd->size );
    zRawVecCopy( zVecBufNC(pvel), zVecBufNC(fd->vel), fd->size );
    zVecFreeAtOnce( 2, pdis, pvel );
  }

  zListForEach( &fd->list, lc ){
    _rkFDCellDatSetOffset( fd, &lc->data, offset );
    offset += rkChainJointSize( rkFDCellChain(lc) );
  }
  fd->size += size;
  return true;
}

bool _rkFDAllocJointStatePop(rkFD *fd, rkFDCell *rlc)
{
  rkFDCell *lc;
  int size, offset = 0, poffset = 0;
  zVec pdis, pvel;

  if( fd->size == 0 || (size = rkChainJointSize( rkFDCellChain(rlc) )) == 0 ) return true;
  zVecFree( fd->acc );
  if( fd->size - size <= 0 ){
    fd->size = 0;
    zVecFreeAtOnce( 2, fd->dis, fd->vel );
    fd->dis = NULL;
    fd->vel = NULL;
    fd->acc = NULL;
    return true;
  }
  pdis = fd->dis;
  pvel = fd->vel;
  if( !( fd->dis = zVecAlloc( fd->size-size ) ) ||
      !( fd->vel = zVecAlloc( fd->size-size ) ) ||
      !( fd->acc = zVecAlloc( fd->size-size ) ) ){
    ZALLOCERROR();
    fd->size = 0;
    zVecFreeAtOnce( 5, pdis, pvel, fd->dis, fd->vel, fd->acc );
    return false;
  }

  zListForEach( &fd->list, lc ){
    size = rkChainJointSize( rkFDCellChain(lc) );
    if( lc == rlc ){
      poffset += size;
      continue;
    }
    zRawVecCopy( zVecBuf(pdis)+poffset, zVecBuf(fd->dis)+offset, size );
    zRawVecCopy( zVecBuf(pvel)+poffset, zVecBuf(fd->vel)+offset, size );
    _rkFDCellDatSetOffset( fd, &lc->data, offset );
    offset += size;
    poffset += size;
  }
  fd->size -= size;

  zVecFreeAtOnce( 2, pdis, pvel );
  return true;
}

rkFDCellDat *_rkFDCellDatJointFrictionPivotInit(rkFDCellDat *ld)
{
  int i, j;
  rkJoint *joint;
  double val[6];
  rkJointFrictionPivot jfp[6];

  for( i=0; i<rkChainLinkNum(rkFDCellDatChain(ld)); i++ ){
    joint = rkChainLinkJoint(&ld->fc.chain,i);
    rkJointGetDis( joint, val );
    for( j=0; j<rkJointDOF(joint); j++ ){
      jfp[j].ref_dis = val[j];
      jfp[j].prev_trq = 0.0;
      jfp[j].type = RK_CONTACT_SF;
    }
    rkJointSetFrictionPivot( joint, jfp );
  }
  return ld;
}

rkFDCellDat *_rkFDCellDatInit(rkFDCellDat *ld)
{
  rkChainAllocABI( rkFDCellDatChain(ld) );
  ld->_dis.size = ld->_vel.size = ld->_acc.size = rkChainJointSize( rkFDCellDatChain(ld) );
  ld->_dis.buf = NULL;
  ld->_vel.buf = NULL;
  ld->_acc.buf = NULL;
  _rkFDCellDatJointFrictionPivotInit( ld );
  return ld;
}

rkFDCell *_rkFDCellPush(rkFD *fd, rkFDCell *lc)
{
  rkCDPair *cdp;
  _rkFDCellDatInit( &lc->data );
  zListInsertHead( &fd->list, lc );
  if( !_rkFDAllocJointStatePush( fd, lc ) ){
    rkFDDestroy( fd );
    return NULL;
  }
  /* cd reg */
  rkCDChainReg( rkFDCDBase(&fd->cd), rkFDCellChain(lc), RK_CD_CELL_MOVE );
  /* set contact info */
  zListForEach( &rkFDCDBase(&fd->cd)->plist, cdp ){
    if( cdp->data.ci == NULL ){
      cdp->data.ci = rkContactInfoArrayAssoc( &fd->ci, rkLinkStuff(cdp->data.cell[0]->data.link),
                                                       rkLinkStuff(cdp->data.cell[1]->data.link) );
      if( cdp->data.ci == NULL )
        cdp->data.ci = &fd->cidef;
    }
  }
  return lc;
}

rkFDCell *rkFDChainReg(rkFD *fd, rkChain *chain)
{
  rkFDCell *lc;

  if( !chain ) return NULL;
  lc = zAlloc( rkFDCell, 1 );
  if( !rkChainClone( chain, rkFDCellChain(lc) ) ){
    zFree( lc );
    return NULL;
  }
  return _rkFDCellPush( fd, lc ) ? lc : NULL;
}

rkFDCell *rkFDChainRegFile(rkFD *fd, char filename[])
{
  rkFDCell *lc;

  lc = zAlloc( rkFDCell, 1 );
  if( !rkChainReadZTK( rkFDCellChain(lc), filename ) ){
    _rkFDCellDatFree( &lc->data );
    zFree( lc );
    return NULL;
  }
  return _rkFDCellPush( fd, lc ) ? lc : NULL;
}

bool rkFDChainUnreg(rkFD *fd, rkFDCell *cell)
{
  rkFDCell *lc;

  zListForEach( &fd->list, lc ){
    if( lc != cell ) continue;
    if( !_rkFDAllocJointStatePop( fd, lc ) ){
      rkFDDestroy( fd );
      return false;
    }
    zListPurge( &fd->list, lc );
    /* cd reg */
    rkCDChainUnreg( rkFDCDBase(&fd->cd), rkFDCellChain(lc) );
    _rkFDCellDatFree( &lc->data );
    zFree( lc );
    return true;
  }
  return false;
}

/******************************************************************************/
/* read contact information */
bool rkFDContactInfoScanFile(rkFD *fd, char filename[]){
  rkCDPair *cdp;

  if( zArraySize(&fd->ci) != 0)
    rkContactInfoArrayDestroy( &fd->ci );
  if( !rkContactInfoArrayReadZTK( &fd->ci, filename ) ) return false;
  /* set contact info */
  zListForEach( &rkFDCDBase(&fd->cd)->plist, cdp ){
    cdp->data.ci = rkContactInfoArrayAssoc( &fd->ci, rkLinkStuff(cdp->data.cell[0]->data.link),
                                                     rkLinkStuff(cdp->data.cell[1]->data.link) );
    if( cdp->data.ci == NULL )
      cdp->data.ci = &fd->cidef;
  }
  return true;
}

/******************************************************************************/
/* set state function */
void rkFDChainSetDis(rkFDCell *lc, zVec dis)
{
  zVecCopy( dis, &lc->data._dis );
  rkChainSetJointDisAll( rkFDCellChain(lc), dis );
}

void rkFDChainSetVel(rkFDCell *lc, zVec vel)
{
  zVecCopy( vel, &lc->data._vel );
  rkChainSetJointVelAll( rkFDCellChain(lc), vel );
}

/* connect each cell state to the total state */
void _rkFDConnectJointState(rkFD *fd, zVec dis, zVec vel, zVec acc)
{
  rkFDCell *lc;

  zListForEach( &fd->list, lc ){
    lc->data._dis.buf = &zVecElemNC(dis,lc->data._offset);
    lc->data._vel.buf = &zVecElemNC(vel,lc->data._offset);
    lc->data._acc.buf = &zVecElemNC(acc,lc->data._offset);
    rkChainFK( rkFDCellChain(lc), &lc->data._dis );
    rkChainSetJointVelAll( rkFDCellChain(lc), &lc->data._vel );
    rkChainUpdateVel( rkFDCellChain(lc) );
  }
}

/******************************************************************************/
/* ODE solver default function */
zVec rkFDODECatDefault(zVec x, double k, zVec v, zVec xnew, void *util)
{
  rkFDCell *lc;
  zVecStruct lv, lxn;

  zVecCopyNC( x, xnew );
  zListForEach( &((rkFD *)util)->list, lc ){
    lv.size  = rkChainJointSize( rkFDCellChain(lc) );
    lxn.size = rkChainJointSize( rkFDCellChain(lc) );
    lv.buf   = &zVecElemNC(v,   lc->data._offset);
    lxn.buf  = &zVecElemNC(xnew,lc->data._offset);
    rkChainCatJointDisAll( rkFDCellChain(lc), &lxn, k, &lv );
  }
  return xnew;
}

zVec rkFDODESubDefault(zVec x1, zVec x2, zVec dx, void *util)
{
  rkFDCell *lc;
  zVecStruct lx2, ldx;

  zVecCopyNC( x1, dx );
  zListForEach( &((rkFD *)util)->list, lc ){
    lx2.size = rkChainJointSize( rkFDCellChain(lc) );
    ldx.size = rkChainJointSize( rkFDCellChain(lc) );
    lx2.buf  = &zVecElemNC(x2,lc->data._offset);
    ldx.buf  = &zVecElemNC(dx,lc->data._offset);
    rkChainSubJointDisAll( rkFDCellChain(lc), &ldx, &lx2 );
  }
  return dx;
}

/******************************************************************************/
/* forward kinematics */
/* NOTE:
 * this function doesn't update the total state in rkFD
 * it may be better to change the specification
 */
void rkFDFK(rkFD *fd, zVec dis)
{
  rkFDCell *lc;
  zVecStruct ldis;

  zListForEach( &fd->list, lc ){
    ldis.size = rkChainJointSize( rkFDCellChain(lc) );
    ldis.buf  = &zVecElemNC(dis,lc->data._offset);
    rkChainFK( rkFDCellChain(lc), &ldis );
  }
}

void rkFDUpdateRate(rkFD *fd, zVec vel, zVec acc)
{
  rkFDCell *lc;
  zVecStruct lvel, lacc;

  zListForEach( &fd->list, lc ){
    lvel.size = lacc.size = rkChainJointSize( rkFDCellChain(lc) );
    lvel.buf  = &zVecElemNC(vel,lc->data._offset);
    lacc.buf  = &zVecElemNC(acc,lc->data._offset);
    rkChainSetJointRateAll( rkFDCellChain(lc), &lvel, &lacc );
    rkChainUpdateRateGrav( rkFDCellChain(lc) );
  }
}

void _rkFDChainUpdateFKRate(rkFDCell *lc)
{
  rkChainFK( rkFDCellChain(lc), &lc->data._dis );
  rkChainSetJointRateAll( rkFDCellChain(lc), &lc->data._vel, &lc->data._acc );
  rkChainUpdateRateGrav( rkFDCellChain(lc) );
}

void rkFDUpdateFKRate(rkFD *fd)
{
  rkFDCell *lc;

  zListForEach( &fd->list, lc ){
    _rkFDChainUpdateFKRate( lc );
  }
}

/******************************************************************************/
/* for a fake-crawler */
void rkFDCDCellSetSlideMode(rkCDCell *cell, bool mode)
{
  cell->data.slide_mode = mode;
}

void rkFDCDCellSetSlideVel(rkCDCell *cell, double vel)
{
  cell->data.slide_vel = vel;
}

void rkFDCDCellSetSlideAxis(rkCDCell *cell, zVec3D *axis)
{
  zVec3DCopy( axis, &cell->data.slide_axis );
}

rkCDCell *rkFDShape3DGetCDCell(rkFD *fd, zShape3D *shape)
{
  rkCDCell *cell;

  zListForEach( &rkFDCDBase(&fd->cd)->clist, cell )
    if( cell->data.shape == shape ) return cell;
  return NULL;
}

rkCDCell *rkFDShape3DSetSlideMode(rkFD *fd, zShape3D *shape, bool mode)
{
  rkCDCell *cell;

  if( ( cell = rkFDShape3DGetCDCell( fd, shape ) ) == NULL )
    return NULL;
  rkFDCDCellSetSlideMode( cell, mode );
  return cell;
}

rkCDCell *rkFDShape3DSetSlideVel(rkFD *fd, zShape3D *shape, double vel)
{
  rkCDCell *cell;

  if( ( cell = rkFDShape3DGetCDCell( fd, shape ) ) == NULL )
    return NULL;
  rkFDCDCellSetSlideVel( cell, vel );
  return cell;
}

rkCDCell *rkFDShape3DSetSlideAxis(rkFD *fd, zShape3D *shape, zVec3D *axis)
{
  rkCDCell *cell;

  if( ( cell = rkFDShape3DGetCDCell( fd, shape ) ) == NULL )
    return NULL;
  rkFDCDCellSetSlideAxis( cell, axis );
  return cell;
}

/******************************************************************************/
/* update function */
/* reset */
void _rkFDUpdateReset(rkFD *fd)
{
  rkFDCell *lc;

  zListForEach( &fd->list, lc ){
    rkFDChainExtWrenchDestroy( rkFDCellChain(lc) );
    rkFDChainReset( &lc->data.fc );
  }
}

/* colchk */
void _rkFDUpdateChainRigidCol(rkFD *fd)
{
  rkCDPairDat **pd;

  rkFDCDForEachRigidPair( &fd->cd, pd ){
    rkFDChainDerived((*pd)->cell[0]->data.chain)->has_rigid_col = true;
    rkFDChainDerived((*pd)->cell[1]->data.chain)->has_rigid_col = true;
  }
}

void _rkFDUpdateCD(rkFD *fd, bool doUpRef)
{
  rkFDSolverColChk( &fd->solver, doUpRef );
  rkFDCDUpdate( &fd->cd );
  _rkFDUpdateChainRigidCol( fd );
}

/* solver */
bool _rkFDUpdateInitSolver(rkFD *fd)
{
  rkFDCell *lc;
  int i;

  if( zListSize(&fd->list) != 0 ){
    zArrayAlloc( rkFDSolverChains(&fd->solver), rkFDChain*, zListSize(&fd->list) );
    if( zArraySize(rkFDSolverChains(&fd->solver)) == 0 ){
      ZALLOCERROR();
      return false;
    }
    i = 0;
    zListForEach( &fd->list, lc ){
      rkFDSolverChain(&fd->solver,i) = &lc->data.fc;
      i++;
    }
  }
  rkFDSolverUpdateInit( &fd->solver );
  return true;
}

void _rkFDUpdateDestroySolver(rkFD *fd)
{
  rkFDSolverUpdateDestroy( &fd->solver );
  zArrayFree( rkFDSolverChains(&fd->solver) );
}

/******************************************************************************/
void _rkFDUpdateAcc(rkFD *fd, bool doUpRef)
{
  rkFDCell *lc;
  void (*update_abi)(rkChain*);
  void (*update)(rkChain*);

  if( !doUpRef ){
    update_abi = &rkChainUpdateCachedABI;
    update     = &rkChainUpdateABI;
  } else {
    update_abi = &rkChainUpdateCachedABIWrench;
    update     = &rkChainUpdateABIWrench;
  }

  zListForEach( &fd->list, lc ){
    if( lc->data.fc.done_abi_init )
      update_abi( rkFDCellChain(lc) );
    else
      update( rkFDCellChain(lc) );
    rkChainGetJointAccAll( rkFDCellChain(lc), &lc->data._acc );
  }
}

void _rkFDUpdateAll(rkFD *fd, double t, bool doUpRef)
{
  _rkFDUpdateReset( fd );
  _rkFDUpdateCD( fd, doUpRef );
  rkFDSolverT(&fd->solver) = t;
  rkFDSolverUpdate( &fd->solver, doUpRef );
}

zVec _rkFDUpdate(double t, zVec dis, zVec vel, void *fd, zVec acc)
{
  zVecZero( acc );
  _rkFDConnectJointState( (rkFD *)fd, dis, vel, acc );
  _rkFDUpdateAll( (rkFD *)fd, t, false );
  _rkFDUpdateAcc( (rkFD *)fd, false );
  return acc;
}

void _rkFDUpdateRef(rkFD *fd)
{
  zVecZero( fd->acc );
  _rkFDConnectJointState( fd, fd->dis, fd->vel, fd->acc );
  _rkFDUpdateAll( fd, rkFDTime(fd), true );
  _rkFDUpdateAcc( fd, true );
  rkFDSolverUpdatePrevDrivingTrq( &fd->solver );
}

/* public update function */
void rkFDUpdateInit(rkFD *fd)
{
  rkFDCDUpdateInit( &fd->cd );
  _rkFDUpdateInitSolver( fd );
  _rkFDUpdateRef( fd );
  zODE2Init( &fd->ode, zVecSize( fd->dis ), fd->ode_step, _rkFDUpdate );
}

rkFD *rkFDUpdate(rkFD *fd)
{
  zODE2Update( &fd->ode, fd->t, fd->dis, fd->vel, rkFDDT(fd), fd );
  fd->t += rkFDDT(fd);
  _rkFDUpdateRef( fd );
  return fd;
}

void rkFDUpdateDestroy(rkFD *fd)
{
  zODE2Destroy( &fd->ode );
  _rkFDUpdateDestroySolver( fd );
  rkFDCDUpdateDestroy( &fd->cd );
}

/* usually not used since allocation cost is high */
rkFD *rkFDSolve(rkFD *fd)
{
  rkFDUpdateInit( fd );
  rkFDUpdate( fd );
  rkFDUpdateDestroy( fd );
  return fd;
}


/******************************************************************************/
/* for debug */
void rkFDPrint(rkFD *fd)
{
  rkFDCell *lc;

  zListForEach( &fd->list, lc )
    rkChainFPrintZTK( stdout, rkFDCellChain(lc) );
}
