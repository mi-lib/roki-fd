/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_mlcp - LCP based formulation with Projected Gauss Seidel method.
 * contributer: 2014- Naoki Wakisaka
 */

#include <roki-fd/rkfd_util.h>
#include <roki-fd/rkfd_mlcp.h>
#include <roki-fd/rkfd_penalty.h>

/* ************************************************************************** */
/* the forward dynamics based on vertex contact
 * ************************************************************************** */
#define _prp(s) ( (rkFDSolverPrpMLCP* )((s)->prp) )

/**************************************/
/* rigid contact */
static void _rkFDSolverContactState(rkFDSolver *s)
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

  if( _prp(s)->size < _prp(s)->colnum ){
    zMatFree( _prp(s)->a );
    zVecFreeAO( 3, _prp(s)->b, _prp(s)->t, _prp(s)->f );

    _prp(s)->size = _prp(s)->colnum;
    _prp(s)->a    = zMatAllocSqr( fnum );
    _prp(s)->b    = zVecAlloc( fnum );
    _prp(s)->t    = zVecAlloc( fnum );
    _prp(s)->f    = zVecAlloc( fnum );
    if( !_prp(s)->a || !_prp(s)->b || !_prp(s)->t || !_prp(s)->f ){
      zMatFree( _prp(s)->a );
      zVecFreeAO( 3, _prp(s)->b, _prp(s)->t, _prp(s)->f );
      return false;
    }
  } else{
    zMatSetSize( _prp(s)->a, fnum, fnum );
    zVecSetSize( _prp(s)->b, fnum );
    zVecSetSize( _prp(s)->t, fnum );
    zVecSetSize( _prp(s)->f, fnum );
  }
  return true;
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
  int offset = 0;
  zVec3D av;
  int i;

  rkFDCDForEachRigidPair( s->cd, pd ){
    if( (*pd)->cell[0]->data.chain != cpd->cell[0]->data.chain &&
        (*pd)->cell[0]->data.chain != cpd->cell[1]->data.chain &&
        (*pd)->cell[1]->data.chain != cpd->cell[0]->data.chain &&
        (*pd)->cell[1]->data.chain != cpd->cell[1]->data.chain ){
      zListForEach( &(*pd)->vlist, cdv ){
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

static void _rkFDSolverRelationAccForceOne(rkFDSolver *s, rkCDPairDat *cpd, rkCDVert *cdv, zVec3D *axis, int offset)
{
  int j;

  for( j=0; j<2; j++ ){
    rkWrenchInit( _prp(s)->w[j] );
    if( cpd->cell[j]->data.type == RK_CD_CELL_STAT ) continue;
    zXform3DInv( rkLinkWldFrame(cpd->cell[j]->data.link), cdv->data.vert, rkWrenchPos(_prp(s)->w[j]) );
    zMulMat3DTVec3D( rkLinkWldAtt(cpd->cell[j]->data.link), axis, rkWrenchForce(_prp(s)->w[j]) );
    if( cpd->cell[j] != cdv->data.cell )
      zVec3DRevDRC( rkWrenchForce(_prp(s)->w[j]) );
  }
  rkFDChainUpdateCachedABIPair( cpd, _prp(s)->w );
  _rkFDSolverRelativeAcc( s, cpd, _prp(s)->b, _prp(s)->t );
  zMatPutCol( _prp(s)->a, offset, _prp(s)->t );

  /* restore ABIPrp */
  rkFDChainRestoreABIAccBiasPair( cpd );
}

static void _rkFDSolverRelationAccForce(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int i;
  int offset = 0;

  /* b */
  rkFDUpdateAccBias( &s->chains );
  _rkFDSolverBiasAcc( s, _prp(s)->b );
  /* a */
  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      for( i=0; i<3; i++ )
        _rkFDSolverRelationAccForceOne( s, *pd, cdv, &cdv->data.axis[i], offset+i );
      offset += 3;
    }
  }
}

/* solve MLCP */

static void _rkFDSolverBiasVel(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int offset = 0;
  int i;

  zVecMulDRC( _prp(s)->b, rkFDPrpDT(s->fdprp) );
  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      rkFDChainPointRelativeVel( *pd, cdv->data.vert, &cdv->data.norm, cdv->data.cell, &cdv->data.vel );
      for( i=0; i<3; i++ )
        zVecElemNC(_prp(s)->b,offset+i) += zVec3DInnerProd( &cdv->data.vel, &cdv->data.axis[i] );
      offset += 3;
    }
  }
}

static void _rkFDSolverRelaxationCompensation(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int i;
  int offset = 0;
  zVec3D d;
  double k;

  rkFDCDForEachRigidPair( s->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      zVec3DSub( cdv->data.vert, &cdv->data.ref, &d );
      /* relaxation */
      for( i=0; i<3; i++ )
        zMatElemNC(_prp(s)->a,offset+i,offset+i) += rkContactInfoL((*pd)->ci);
      /* compensation */
      k = cdv->data.type == RK_CONTACT_SF ?
        rkContactInfoSF((*pd)->ci) : rkContactInfoKF((*pd)->ci);
      zVecElemNC(_prp(s)->b,offset  ) += rkContactInfoK((*pd)->ci)     * zVec3DInnerProd( &d, &cdv->data.axis[0] );
      zVecElemNC(_prp(s)->b,offset+1) += rkContactInfoK((*pd)->ci) * k * zVec3DInnerProd( &d, &cdv->data.axis[1] );
      zVecElemNC(_prp(s)->b,offset+2) += rkContactInfoK((*pd)->ci) * k * zVec3DInnerProd( &d, &cdv->data.axis[2] );
      offset += 3;
    }
  }
}

static void _rkFDSolverMLCP(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int offset = 0;
  int cnt = 0;
  double ff[2], fs, fnorm;
  int i;

  /* MLCP */
  zVecZero( _prp(s)->f );
  while( cnt < rkFDPrpMaxIter(s->fdprp) ){
    /* norm */
    offset = 0;
    rkFDCDForEachRigidPair( s->cd, pd ){
      zListForEach( &(*pd)->vlist, cdv ){
        ff[0] = - ( zVecElemNC(_prp(s)->b,offset) + zRawVecInnerProd( zMatRowBuf(_prp(s)->a,offset), zVecBuf(_prp(s)->f), zVecSizeNC(_prp(s)->f) )
                - zMatElemNC(_prp(s)->a,offset,offset) * zVecElemNC(_prp(s)->f,offset) ) / zMatElemNC(_prp(s)->a,offset,offset);
        if( ff[0] < zTOL )
          zVecElemNC(_prp(s)->f,offset) = 0.0;
        else
          zVecElemNC(_prp(s)->f,offset) = ff[0];
        offset += 3;
      }
    }
    /* fric */
    offset = 0;
    rkFDCDForEachRigidPair( s->cd, pd ){
      zListForEach( &(*pd)->vlist, cdv ){
        for( i=0; i<2; i++ ){
          if( fabs( zMatElemNC(_prp(s)->a,offset+i,offset+i) ) < zTOL )
            ff[i] = 0;
          else
            ff[i] = - ( zVecElemNC(_prp(s)->b,offset+i) + zRawVecInnerProd( zMatRowBuf(_prp(s)->a,offset+i), zVecBuf(_prp(s)->f), zVecSizeNC(_prp(s)->f) )
                        - zMatElemNC(_prp(s)->a,offset+i,offset+i) * zVecElemNC(_prp(s)->f,offset+i) ) / zMatElemNC(_prp(s)->a,offset+i,offset+i);
        }

        fnorm = zSqr( ff[0] ) + zSqr( ff[1] );
        fs = cdv->data.type == RK_CONTACT_SF ?
          zSqr( rkContactInfoSF((*pd)->ci)*zVecElemNC(_prp(s)->f,offset) ) :
          zSqr( rkContactInfoKF((*pd)->ci)*zVecElemNC(_prp(s)->f,offset) );

        if( fnorm < zTOL || fs < zTOL ){
          zVecElemNC(_prp(s)->f,offset+1) = 0.0;
          zVecElemNC(_prp(s)->f,offset+2) = 0.0;
        } else if( fnorm > fs ){
          fs /= fnorm;
          zVecElemNC(_prp(s)->f,offset+1) = ff[0] * fs;
          zVecElemNC(_prp(s)->f,offset+2) = ff[1] * fs;
        } else {
          zVecElemNC(_prp(s)->f,offset+1) = ff[0];
          zVecElemNC(_prp(s)->f,offset+2) = ff[1];
        }
        offset += 3;
      }
    }
    cnt++;
  }
  zVecDivDRC( _prp(s)->f, rkFDPrpDT(s->fdprp) );
}

/* set force */
static void _rkFDSolverSetForce(rkFDSolver *s, bool doUpRef)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  int offset = 0;
  int i;
  double fn ,fs, mu;

  rkFDCDForEachRigidPair( s->cd, pd ){
    if( (*pd)->cell[0]->data.type == RK_CD_CELL_STAT && (*pd)->cell[1]->data.type == RK_CD_CELL_STAT ) continue;
    zListForEach( &(*pd)->vlist, cdv ){
      zVec3DZero( &cdv->data.f );
      for( i=0; i<3; i++ )
        zVec3DCatDRC( &cdv->data.f, zVecElemNC(_prp(s)->f,offset+i), &cdv->data.axis[i] );
      rkFDContactForcePushWrench( *pd, cdv );

      /* update friction type */
      fn = cdv->data.f.e[0];
      fs = sqrt( zSqr( cdv->data.f.e[1] ) + zSqr( cdv->data.f.e[2] ) );
      mu = cdv->data.type == RK_CONTACT_SF ?
        rkContactInfoSF((*pd)->ci) : rkContactInfoKF((*pd)->ci);

      if( fs > mu * fn - zTOL ){
        cdv->data.type = RK_CONTACT_KF;
        zVec3DCopy( &cdv->data._pro, &cdv->data._ref );
      } else {
        cdv->data.type = RK_CONTACT_SF;
        rkFDUpdateRefSlide( *pd, cdv, rkFDPrpDT(s->fdprp) );
      }
      offset += 3;
    }
  }
}

/*****************************************************************************/
static bool _rkFDSolverConstraint(rkFDSolver *s, bool doUpRef)
{
  _rkFDSolverContactState( s );
  if( !_rkFDSolverPrpReAlloc( s ) ) return false;
  _rkFDSolverRelationAccForce( s );
  _rkFDSolverBiasVel( s );
  _rkFDSolverRelaxationCompensation( s );
  _rkFDSolverMLCP( s );
  _rkFDSolverSetForce( s, doUpRef );
  return true;
}

/*****************************************************************************/
/* abstruct functions */
void rkFDSolverGetDefaultContactInfo_MLCP(rkFDSolver *s, rkContactInfo *ci)
{
  rkContactInfoInit( ci );
  rkContactInfoSetType( ci, RK_CONTACT_RIGID );
  rkContactInfoSetSF( ci, 0.5 );
  rkContactInfoSetKF( ci, 0.3 );

  rkContactInfoSetK( ci, 1000.0 );
  rkContactInfoSetL( ci, 1.0 );
}

bool rkFDSolverUpdateInit_MLCP(rkFDSolver *s)
{
  _prp(s)->colnum = 0;
  _prp(s)->size = 0;
  _prp(s)->a = NULL;
  _prp(s)->b = NULL;
  _prp(s)->t = NULL;
  _prp(s)->f   = NULL;

  _prp(s)->w[0] = zAlloc( rkWrench, 1 );
  _prp(s)->w[1] = zAlloc( rkWrench, 1 );
  if( !_prp(s)->w[0] || !_prp(s)->w[1] ) return false;
  return true;
}

void rkFDSolverColChk_MLCP(rkFDSolver *s, bool doUpRef)
{
  zEchoOff();
  /* if( doUpRef ) */
  rkCDColChkVert( rkFDCDBase(rkFDSolverCD(s)) );
  zEchoOn();
}

bool rkFDSolverUpdate_MLCP(rkFDSolver *s, bool doUpRef)
{
  rkFDJointFriction( rkFDSolverChains(s), rkFDPrpDT(s->fdprp), rkFDPrpFrictionWeight(s->fdprp), doUpRef );
  if( rkFDCDElastNum(s->cd) != 0 )
    rkFDSolverPenalty( s, doUpRef );
  if( rkFDCDRigidNum(s->cd) != 0 )
    if( !_rkFDSolverConstraint( s, doUpRef ) ) return false;
  return true;
}

void rkFDSolverUpdatePrevDrivingTrq_MLCP(rkFDSolver *s)
{
  rkFDUpdateJointPrevDrivingTrq( rkFDSolverChains(s) );
}

void rkFDSolverUpdateDestroy_MLCP(rkFDSolver *s)
{
  zMatFree( _prp(s)->a );
  zVecFreeAO( 3, _prp(s)->b, _prp(s)->t, _prp(s)->f );
  zFree( _prp(s)->w[0] );
  zFree( _prp(s)->w[1] );
}

RKFD_SOLVER_DEFAULT_GENERATOR( MLCP )

#undef _prp
