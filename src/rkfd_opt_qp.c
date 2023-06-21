/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_opt_qp - optimization tools: quadratic programming.
 * contributer: 2014- Naoki Wakisaka
 */

#include <roki_fd/rkfd_opt_qp.h>
#include <zm/zm_opt.h>

typedef struct{ /* list of active set indices */
  zIndex idx;
  double min;
} _rkFDQPASMIndexData;
zListClass( _rkFDQPASMIndexList, _rkFDQPASMIndex, _rkFDQPASMIndexData );

static zVec _rkFDQPSolveASMInitDefault(zMat a, zVec b, zVec ans, void *util)
{
  return zVecZero( ans );
}

static double _rkFDQPSolveASMConditionDefault(zMat a, zVec ans, int i, void *util)
{
  return zRawVecInnerProd(zMatRowBuf(a,i),zVecBuf(ans),zVecSizeNC(ans));
}

static uint _rkFDQPSolveASMInitIndex(zIndex idx, zMat a, zVec b, zVec ans, void *util, double cond(zMat,zVec,int,void*))
{
  int m, i;

  /* initialize the active set of constraints */
  for( m=0, i=0; i<zVecSizeNC(b); i++ ){
    if( zIsEqual( cond( a, ans, i, util ), zVecElemNC(b,i), zTOL ) ){
      zIndexElemNC(idx,i) = 1;
      m++;
    } else
      zIndexElemNC(idx,i) = 0;
  }
  return m;
}

#define RKFD_OPT_QP_ASM_TOL ( 1.0e-8 )
bool rkFDQPSolveASM(zMat q, zVec c, zMat a, zVec b, zVec ans, zIndex idx, zVec init(zMat,zVec,zVec,void*), double cond(zMat,zVec,int,void*), void *util)
{
  zMat qa;
  zVec xy, cb, d;
  double tempd, tempd2, objv;
  _rkFDQPASMIndexList ilist;
  _rkFDQPASMIndex *idata;
  bool endflag, ret = true;
  int tempi;
  int n, nm, m;
  int i, j, k;

  /* function settings */
  if( !init ) init = _rkFDQPSolveASMInitDefault;
  if( !cond ) cond = _rkFDQPSolveASMConditionDefault;

  /* initalize */
  init( a, b, ans, util );

  /* allocate workspace */
  n = zVecSizeNC(ans);
  zListInit( &ilist );
  d = zVecAlloc(n);
  qa = zMatAllocSqr( n + zVecSizeNC(b) );
  xy = zVecAlloc( n + zVecSizeNC(b) );
  cb = zVecAlloc( n + zVecSizeNC(b) );
  if( !d || !qa || !xy || !cb ){
    ret = false;
    goto RET;
  }

  /* solve */
  m = _rkFDQPSolveASMInitIndex( idx, a, b, ans, util, cond );
  do{
    nm = n + m;
    zMatSetSize( qa, nm, nm );
    zVecSetSize( xy, nm );
    zVecSetSize( cb, nm );

    for( i=0; i<n; i++ )
      for( j=0; j<n; j++ )
        zMatElemNC(qa,i,j) = -zMatElemNC(q,i,j);
    for( k=0, j=n; j<nm; j++ ){
      while( !zIndexElemNC(idx,k) && k < zVecSizeNC(b) ) k++;
      for( i=0; i<n; i++ )
        zMatElemNC(qa,i,j) = zMatElemNC(a,k,i);
      k++;
    }
    for( k=0, i=n; i<nm; i++ ){
      while( !zIndexElemNC(idx,k) && k < zVecSizeNC(b) ) k++;
      for( j=0; j<n; j++ )
        zMatElemNC(qa,i,j) = zMatElemNC(a,k,j);
      k++;
    }
    for( i=n; i<nm; i++ )
      for( j=n; j<nm; j++ )
        zMatElemNC(qa,i,j) = 0.0;
    for( i=0; i<n; i++ )
      zVecElemNC(cb,i) = zVecElemNC(c,i);
    for( k=0; i<nm; i++ ){
      while( !zIndexElemNC(idx,k) && k < zVecSizeNC(b) ) k++;
      zVecElemNC(cb,i) = zVecElemNC(b,k);
      k++;
    }
    zLESolveMP( qa, cb, NULL, NULL, xy );

    for( i=0; i<n; i++ )
      if( !zIsEqual( zVecElemNC(xy,i), zVecElemNC(ans,i), zTOL ) ) goto STEP2;
    for( i=0; i<n; i++ )
      zVecElemNC(ans,i) = zVecElemNC(xy,i);
    for( i=0; i<m; i++ )
      if( zVecElemNC(xy,n+i) < 0 ) goto NEXT;
    goto RET; /* found the optimal solution */

   NEXT:
    tempd = zVecElemNC(xy,n);
    for( i=1; i<m; i++ )
      if( zVecElemNC(xy,n+i) < tempd )
        tempd = zVecElemNC(xy,n+i);
    tempi = 0;
    for( i=0; i<zVecSizeNC(b); i++ )
      if( zIndexElemNC(idx,i) != 0 ){
        if( fabs( zVecElemNC(xy,tempi+n) - tempd ) < RKFD_OPT_QP_ASM_TOL ){
          zIndexElemNC(idx,i) = 0;
          m--;
        }
        tempi++;
      }
    continue;

   STEP2:
    /* find a new feasible direction */
    for( i=0; i<n; i++ )
      zVecElemNC(d,i) = zVecElemNC(xy,i) - zVecElemNC(ans,i);
    tempd = 1.0;
    for( i=0; i<zVecSizeNC(b); i++ ){
      tempd2 = zRawVecInnerProd( zMatRowBuf(a,i), zVecBuf(d), n );
      if( zIndexElemNC(idx,i) == 0 && tempd2 < 0 ){
        tempd2 = ( zVecElemNC(b,i) - cond( a, ans, i, util ) ) / tempd2;
        if( tempd2 < tempd ) tempd = tempd2;
      }
    }
    for( i=0; i<n; i++ )
      zVecElemNC(ans,i) += tempd * zVecElemNC(d,i);
    for( i=0; i<zVecSizeNC(b); i++ )
      if( zIndexElemNC(idx,i) == 0 && zIsEqual( cond( a, ans, i, util ), zVecElemNC(b,i), zTOL ) ){
        zIndexElemNC(idx,i) = 1;
        m++;
      }
    /* check if circulation happens due to degeneracy */
    endflag = false;
    objv = zQuadraticValue( q, c, ans );
    zListForEach( &ilist, idata ){
      for( i=0; i<zVecSizeNC(b);i++ )
        if( zIndexElemNC(idx,i) != zIndexElemNC(idata->data.idx,i) ) goto CONTINUE;
      if( fabs( idata->data.min/objv - 1.0 ) > RKFD_OPT_QP_ASM_TOL ) goto CONTINUE;
      endflag = true;
      break;
     CONTINUE: ;
    }
    if( endflag == true ) goto RET;

    /* register to the list of bases */
    idata = zAlloc( _rkFDQPASMIndex, 1 );
    idata->data.idx = zIndexCreate( zVecSizeNC(b) );
    for( i=0; i<zVecSizeNC(b); i++ )
      zIndexElemNC(idata->data.idx,i) = zIndexElemNC(idx,i);
    idata->data.min = objv;
    zListInsertTail( &ilist ,idata );
  } while( 1 );

 RET:
  zListForEach( &ilist, idata )
    zIndexFree( idata->data.idx );
  zListDestroy( _rkFDQPASMIndex, &ilist );
  zMatFree( qa );
  zVecFreeAO( 3, xy, cb, d );
  return ret;
}
