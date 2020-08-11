/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_penalty - contact force computation based on penalty method
 * contributer: 2014- Naoki Wakisaka
 */

#include <roki-fd/rkfd_penalty.h>
#include <roki-fd/rkfd_util.h>

void rkFDSolverPenalty(rkFDSolver *solver, bool doUpRef)
{
  rkCDPairDat **pd;
  rkCDVert *cdv;
  zVec3D d, vr;

  rkFDCDForEachElastPair( solver->cd, pd ){
    zListForEach( &(*pd)->vlist, cdv ){
      zVec3DSub( cdv->data.vert, &cdv->data.ref, &d );
      rkFDChainPointRelativeVel( *pd, cdv->data.vert, &cdv->data.norm, cdv->data.cell, &vr );

      /* penalty force */
      zVec3DMul( &d, - rkContactInfoE((*pd)->ci), &cdv->data.f );
      zVec3DCatDRC( &cdv->data.f, -1.0*( rkContactInfoV((*pd)->ci) + rkContactInfoE((*pd)->ci) * rkFDSolverDT(solver) ), &vr );
      if( zVec3DInnerProd( &cdv->data.f, &cdv->data.axis[0] ) < 0.0 ) continue;

      rkFDContactForceModifyFriction( rkFDSolverFDPrp(solver), *pd, cdv, vr, doUpRef );
      rkFDContactForcePushWrench( *pd, cdv );
    }
  }
}

void rkFDSolverPenaltyVolumeBased(rkFDSolver *solver, bool doUpRef)
{
  /* under construction */
}
