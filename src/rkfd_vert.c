/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_vert - vertex / plane contact model
 * contributer: 2014- Naoki Wakisaka
 */

#include <roki_fd/rkfd_util.h>
#include <roki_fd/rkfd_vert.h>
#include <roki_fd/rkfd_penalty.h>
#include <roki_fd/rkfd_opt_qp.h>

/* the friction states are determined based on the computed friction forces one step before.
 * the direction of the kinetic friction is not fixed.
 */
/* ************************************************************************** */
/* the forward dynamics based on vertex contact
 * ************************************************************************** */
#define _prp(s) ( (rkFDSolverPrpVert* )((s)->prp) )

/* count rigid contact vertics */
static void _rkFDSolverCountContacts(rkFDSolver *s)
{
  rkCDPairDat **pd;

  _prp(s)->colnum = 0;
  rkFDCDForEachRigidPair( s->cd, pd )
    _prp(s)->colnum += zListSize(&(*pd)->vlist);
}

/* allocate workspace */
static bool _rkFDSolverPrpReAlloc(rkFDSolver *s)
{
  int fnum = 3*_prp(s)->colnum;
  int cnum = rkFDPrpPyramid(s->fdprp) * _prp(s)->colnum;

  if( _prp(s)->qp_size < _prp(s)->colnum ){
    zMatFreeAtOnce( 3, _prp(s)->a, _prp(s)->q, _prp(s)->nf );
    zVecFreeAtOnce( 5, _prp(s)->b, _prp(s)->t, _prp(s)->c, _prp(s)->d, _prp(s)->f );
    zIndexFree( _prp(s)->idx );

    _prp(s)->qp_size = _prp(s)->colnum;
    _prp(s)->a       = zMatAllocSqr( fnum );
    _prp(s)->b       = zVecAlloc( fnum );
    _prp(s)->t       = zVecAlloc( fnum );
    _prp(s)->q       = zMatAllocSqr( fnum );
    _prp(s)->c       = zVecAlloc( fnum );
    _prp(s)->f       = zVecAlloc( fnum );
    _prp(s)->nf      = zMatAlloc( cnum, fnum );
    _prp(s)->d       = zVecAlloc( cnum );
    _prp(s)->idx     = zIndexCreate( cnum );
    if( !_prp(s)->a || !_prp(s)->q || !_prp(s)->nf || !_prp(s)->b || !_prp(s)->t || !_prp(s)->c || !_prp(s)->d || !_prp(s)->f || !_prp(s)->idx ){
      zMatFreeAtOnce( 3, _prp(s)->a, _prp(s)->q, _prp(s)->nf );
      zVecFreeAtOnce( 5, _prp(s)->b, _prp(s)->t, _prp(s)->c, _prp(s)->d, _prp(s)->f );
      zIndexFree( _prp(s)->idx );
      return false;
    }
    zVecZero( _prp(s)->d );
  } else {
    zMatSetSize( _prp(s)->a, fnum, fnum );
    zVecSetSize( _prp(s)->b, fnum );
    zVecSetSize( _prp(s)->t, fnum );
    zMatSetSize( _prp(s)->q, fnum, fnum );
    zVecSetSize( _prp(s)->c, fnum );
    zVecSetSize( _prp(s)->f, fnum );
    zMatSetSize( _prp(s)->nf, cnum, fnum );
    zVecSetSize( _prp(s)->d, cnum );
  }
  return true;
}

/* friction constraint */
static void _rkFDSolverFrictionConstraint(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int i;
  int cnt = 0;
  int cnum;
  int offset, ioffset;
  double fric;

  cnum = rkFDPrpPyramid(s->fdprp);
  zMatZero( _prp(s)->nf );
  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      offset  = 3 * cnt;
      ioffset = cnum * cnt;
      if( cdv->data.type == RK_CONTACT_KF )
        fric = rkContactInfoKF((*pd)->ci);
      else
        fric = rkContactInfoSF((*pd)->ci);
      fric *= zVecElemNC(_prp(s)->sc_table[1],0);

      for( i=0; i<cnum; i++ ){
        zMatElemNC(_prp(s)->nf,ioffset+i,offset  ) = fric;
        zMatElemNC(_prp(s)->nf,ioffset+i,offset+1) = zVecElemNC(_prp(s)->sc_table[0],i);
        zMatElemNC(_prp(s)->nf,ioffset+i,offset+2) = zVecElemNC(_prp(s)->sc_table[1],i);
      }
      cnt++;
    }
  }
}

/* relation between acc and force */

static void _rkFDSolverBiasAcc(rkFDSolver *s, zVec acc)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int i;
  int offset = 0;
  zVec3D av;

  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      rkFDChainPointRelativeAcc( *pd, cdv->data.vert, cdv->data.cell, &av );
      for( i=0; i<3; i++ )
        zVecElemNC(acc,offset+i) = zVec3DInnerProd( &cdv->data.axis[i], &av );
      offset += 3;
    }
  }
}

static void _rkFDSolverRelativeAcc(rkFDSolver *s, rkCDPairDat *cpd, zVec b, zVec a)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int i;
  int offset = 0;
  zVec3D av;

  rkFDCDForEachRigidPair( s->cd, pd ){
    if( (*pd)->cell[0]->data.chain != cpd->cell[0]->data.chain &&
        (*pd)->cell[0]->data.chain != cpd->cell[1]->data.chain &&
        (*pd)->cell[1]->data.chain != cpd->cell[0]->data.chain &&
        (*pd)->cell[1]->data.chain != cpd->cell[1]->data.chain ){
      for( i=0; i<zListSize(&(*pd)->vlist); i++ ){
        zVec3DZero( (zVec3D *)&zVecElemNC(a,offset) );
        offset += 3;
      }
    } else {
      zListForEach( &(*pd)->vlist, cdv ){
        rkFDChainPointRelativeAcc( *pd, cdv->data.vert, cdv->data.cell, &av );
        for( i=0; i<3; i++ )
          zVecElemNC(a,offset+i) = zVec3DInnerProd( &cdv->data.axis[i], &av ) - zVecElemNC(b,offset+i);
        offset += 3;
      }
    }
  }
}

static void _rkFDSolverRelationAccForce(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int i, j;
  int offset = 0;

  /* b */
  rkFDUpdateAccBias( &s->chains );
  _rkFDSolverBiasAcc( s, _prp(s)->b );
  /* a */
  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      for( i=0; i<3; i++ ){
        for( j=0; j<2; j++ ){
          rkWrenchInit( _prp(s)->w[j] );
          if( (*pd)->cell[j]->data.type == RK_CD_CELL_STAT ) continue;
          zXform3DInv( rkLinkWldFrame((*pd)->cell[j]->data.link), cdv->data.vert, rkWrenchPos(_prp(s)->w[j]) );
          zMulMat3DTVec3D( rkLinkWldAtt((*pd)->cell[j]->data.link), &cdv->data.axis[i], rkWrenchForce(_prp(s)->w[j]) );
          if( (*pd)->cell[j] != cdv->data.cell )
            zVec3DRevDRC( rkWrenchForce(_prp(s)->w[j]) );
        }
        rkFDChainUpdateCachedABIPair( *pd, _prp(s)->w );
        _rkFDSolverRelativeAcc( s, *pd, _prp(s)->b, _prp(s)->t );
        zMatPutCol( _prp(s)->a, offset+i, _prp(s)->t );

        /* restore ABIPrp */
        rkFDChainRestoreABIAccBiasPair( *pd );
      }
      offset += 3;
    }
  }
}

/* solve QP */

static void _rkFDSolverBiasVel(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int offset = 0;

  zVecMulDRC( _prp(s)->b, rkFDPrpDT(s->fdprp) );
  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      rkFDChainPointRelativeVel( *pd, cdv->data.vert, &cdv->data.norm, cdv->data.cell, &cdv->data.vel );

      zVecElemNC(_prp(s)->b,offset  ) += zVec3DInnerProd( &cdv->data.vel, &cdv->data.axis[0] );
      zVecElemNC(_prp(s)->b,offset+1) += zVec3DInnerProd( &cdv->data.vel, &cdv->data.axis[1] );
      zVecElemNC(_prp(s)->b,offset+2) += zVec3DInnerProd( &cdv->data.vel, &cdv->data.axis[2] );
      offset += 3;
    }
  }
}

static void _rkFDSolverCompensateDepth(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int offset = 0;
  zVec3D d;
  double fric, k;

  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      zVec3DSub( cdv->data.vert, &cdv->data.ref, &d );
      if( cdv->data.type == RK_CONTACT_KF )
        fric = rkContactInfoKF((*pd)->ci);
      else
        fric = rkContactInfoSF((*pd)->ci);
      k = rkContactInfoK((*pd)->ci);
      /* k = rkContactInfoK((*pd)->ci) / ( 1 + rkFDDT(fd)*rkContactInfoK((*pd)->ci) ); */

      zVecElemNC(_prp(s)->c,offset  ) = zVecElemNC(_prp(s)->b,offset  ) + k        * zVec3DInnerProd( &d, &cdv->data.axis[0] );
      zVecElemNC(_prp(s)->c,offset+1) = zVecElemNC(_prp(s)->b,offset+1) + k * fric * zVec3DInnerProd( &d, &cdv->data.axis[1] );
      zVecElemNC(_prp(s)->c,offset+2) = zVecElemNC(_prp(s)->b,offset+2) + k * fric * zVec3DInnerProd( &d, &cdv->data.axis[2] );
      offset += 3;
    }
  }
}

static zVec _rkFDSolverQPASMInit(zMat a, zVec b, zVec ans, void *util)
{
  rkFDSolver *s;
  int i;

  s = (rkFDSolver *)util;
  zVecZero( ans );
  for( i=0; i<_prp(s)->colnum; i++ )
    zVecElemNC(ans,3*i) = 1.0;
  return ans;
}

static double _rkFDSolverQPASMCond(zMat a, zVec ans, int i, void *util)
{
  int n = i / ( rkFDPrpPyramid(((rkFDSolver *)util)->fdprp) );
  return zVec3DInnerProd( (zVec3D *)&zMatElemNC(a,i,3*n), (zVec3D *)&zVecElemNC(ans,3*n) );
}

static void _rkFDSolverQPASM(rkFDSolver *s)
{
  rkFDQPSolveASM( _prp(s)->q, _prp(s)->c, _prp(s)->nf, _prp(s)->d, _prp(s)->f, _prp(s)->idx, _rkFDSolverQPASMInit, _rkFDSolverQPASMCond, s);
}

/* Quadratic Problem */
static void _rkFDSolverQP(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int offset = 0;

  _rkFDSolverBiasVel( s );
  _rkFDSolverCompensateDepth( s );
  /* estimation function (sum of error norm) */
  zMulMatTMat( _prp(s)->a, _prp(s)->a, _prp(s)->q );
  zMulMatTVecDRC( _prp(s)->a, _prp(s)->c );
  /* other version (low computation cost) */
  /* zMatCopy( _prp(s)->a, _prp(s)->q ); */

  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      zMatElemNC(_prp(s)->q,offset  ,offset  ) += rkContactInfoL((*pd)->ci);
      zMatElemNC(_prp(s)->q,offset+1,offset+1) += rkContactInfoL((*pd)->ci);
      zMatElemNC(_prp(s)->q,offset+2,offset+2) += rkContactInfoL((*pd)->ci);
      offset += 3;
    }
  }
  /* solve */
  _rkFDSolverQPASM( s );
  zVecDivDRC( _prp(s)->f, rkFDPrpDT(s->fdprp) );
}

/* set force and update friction type */
static void _rkFDSolverSetForce(rkFDSolver *s, bool doUpRef)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int i;
  int cnt = 0, offset, ioffset, cnum;
  bool flag;

  cnum = rkFDPrpPyramid(s->fdprp);
  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      zVec3DZero( &cdv->data.f );
      offset = 3 * cnt;
      ioffset = cnum * cnt;
      for( i=0; i<3; i++ )
        zVec3DCatDRC( &cdv->data.f, zVecElemNC(_prp(s)->f,offset+i), &cdv->data.axis[i] );
      rkFDContactForcePushWrench( *pd, cdv );

      /* update friction type */
      if( doUpRef ){
        flag = false;
        for( i=0; i<cnum; i++ ){
          if( zIndexElemNC(_prp(s)->idx,ioffset+i) ){
            flag = true;
            break;
          }
        }
        if( flag ){
          cdv->data.type = RK_CONTACT_KF;
          zVec3DCopy( &cdv->data._pro, &cdv->data._ref );
        } else {
          cdv->data.type = RK_CONTACT_SF;
          rkFDUpdateRefSlide( *pd, cdv, rkFDPrpDT(s->fdprp) );
        }
      }
      cnt++;
    }
  }
}

/*****************************************************************************/
static bool _rkFDSolverConstraint(rkFDSolver *s, bool doUpRef)
{
  _rkFDSolverCountContacts( s );
  if( !_rkFDSolverPrpReAlloc( s ) ) return false;
  _rkFDSolverFrictionConstraint( s );
  _rkFDSolverRelationAccForce( s );
  _rkFDSolverQP( s );
  _rkFDSolverSetForce( s, doUpRef );
  return true;
}

/*****************************************************************************/
/* abstruct functions */
void rkFDSolverGetDefaultContactInfo_Vert(rkFDSolver *s, rkContactInfo *ci)
{
  rkContactInfoInit( ci );
  rkContactInfoSetType( ci, RK_CONTACT_RIGID );
  rkContactInfoSetK( ci, 1000.0 );
  rkContactInfoSetL( ci, 1.0 );
  rkContactInfoSetSF( ci, 0.5 );
  rkContactInfoSetKF( ci, 0.3 );
}

bool rkFDSolverUpdateInit_Vert(rkFDSolver *s)
{
  _prp(s)->colnum = 0;
  _prp(s)->a = NULL;
  _prp(s)->b = NULL;
  _prp(s)->t = NULL;

  _prp(s)->qp_size = 0;
  _prp(s)->q   = NULL;
  _prp(s)->nf  = NULL;
  _prp(s)->c   = NULL;
  _prp(s)->d   = NULL;
  _prp(s)->f   = NULL;
  _prp(s)->idx = NULL;

  _prp(s)->w[0] = zAlloc( rkWrench, 1 );
  _prp(s)->w[1] = zAlloc( rkWrench, 1 );
  if( !_prp(s)->w[0] || !_prp(s)->w[1] ) return false;

  return rkFDCrateSinCosTable( _prp(s)->sc_table, rkFDPrpPyramid(s->fdprp), -zDeg2Rad( 180.0 ) / rkFDPrpPyramid(s->fdprp) );
}

void rkFDSolverColChk_Vert(rkFDSolver *s, bool doUpRef)
{
  zEchoOff();
  /* if( doUpRef ) */
  rkCDColChkVert( rkFDCDBase(rkFDSolverCD(s)) );
  zEchoOn();
}

bool rkFDSolverUpdate_Vert(rkFDSolver *s, bool doUpRef)
{
  rkFDJointFriction( rkFDSolverChains(s), rkFDPrpDT(s->fdprp), rkFDPrpFrictionWeight(s->fdprp), doUpRef );
  if( rkFDCDElastNum(s->cd) != 0 )
    rkFDSolverPenalty( s, doUpRef );
  if( rkFDCDRigidNum(s->cd) != 0 )
    if( !_rkFDSolverConstraint( s, doUpRef ) ) return false;
  return true;
}

void rkFDSolverUpdatePrevDrivingTrq_Vert(rkFDSolver *s)
{
  rkFDUpdateJointPrevDrivingTrq( rkFDSolverChains(s) );
}

void rkFDSolverUpdateDestroy_Vert(rkFDSolver *s)
{
  zMatFreeAtOnce( 3, _prp(s)->a, _prp(s)->q, _prp(s)->nf );
  zVecFreeAtOnce( 7, _prp(s)->b, _prp(s)->t, _prp(s)->c, _prp(s)->d, _prp(s)->f, _prp(s)->sc_table[0], _prp(s)->sc_table[1] );
  zFree( _prp(s)->w[0] );
  zFree( _prp(s)->w[1] );
  zIndexFree( _prp(s)->idx );
}

RKFD_SOLVER_DEFAULT_GENERATOR( Vert )

#undef _prp
