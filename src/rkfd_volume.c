#include <roki-fd/rkfd_util.h>
#include <roki-fd/rkfd_volume.h>
#include <roki-fd/rkfd_penalty.h>

/* ************************************************************************** */
/* the forward dynamics based on contact volume
 * ************************************************************************** */
#define _prp(s) ( (rkFDSolverPrpVolume* )((s)->prp) )
#define _zMat6DElem(m,i,j)  (m)->e[(i)/3][(j)/3].e[(j)%3][(i)%3]
/* #define _zMat6DElem(m,i,j)  (m)->e[(j)/3][(i)/3].e[(j)%3][(i)%3] */

/* count rigid contact vertics */
static void _rkFDSolverCountContacts(rkFDSolver *s);
void _rkFDSolverCountContacts(rkFDSolver *s)
{
  rkCDPairDat **pd;

  _prp(s)->colnum = 0;
  rkFDCDForEachRigidPair( s->cd, pd )
    _prp(s)->colnum += zListNum(&(*pd)->cplane);
}

/* reallocate workspace */
static bool _rkFDSolverPrpReAllocQP(rkFDSolver *s);
static bool _rkFDSolverPrpReAllocQPCondition(rkFDSolver *s);
static bool _rkFDSolverPrpReAllocModification(rkFDSolver *s);

bool _rkFDSolverPrpReAllocQP(rkFDSolver *s){
  int fnum = 6*rkFDCDRigidNum(s->cd);

  if( _prp(s)->fsize < fnum ){
    zMatFreeAO( 2, _prp(s)->a, _prp(s)->q );
    zVecFreeAO( 4, _prp(s)->b, _prp(s)->t, _prp(s)->c, _prp(s)->f );

    _prp(s)->fsize = fnum;
    _prp(s)->a = zMatAllocSqr( fnum );
    _prp(s)->b = zVecAlloc( fnum );
    _prp(s)->t = zVecAlloc( fnum );
    _prp(s)->q = zMatAllocSqr( fnum );
    _prp(s)->c = zVecAlloc( fnum );
    _prp(s)->f = zVecAlloc( fnum );

    if( !_prp(s)->a || !_prp(s)->q || !_prp(s)->b || !_prp(s)->t || !_prp(s)->c || !_prp(s)->f ){
      zMatFreeAO( 2, _prp(s)->a, _prp(s)->q );
      zVecFreeAO( 4, _prp(s)->b, _prp(s)->t, _prp(s)->c, _prp(s)->f );
      return false;
    }
  } else {
    zMatSetSize( _prp(s)->a, fnum, fnum );
    zVecSetSize( _prp(s)->b, fnum );
    zVecSetSize( _prp(s)->t, fnum );
    zMatSetSize( _prp(s)->q, fnum, fnum );
    zVecSetSize( _prp(s)->c, fnum );
    zVecSetSize( _prp(s)->f, fnum );
  }
  return true;
}

bool _rkFDSolverPrpReAllocQPCondition(rkFDSolver *s)
{
  int fnum = 6*rkFDCDRigidNum(s->cd);
  int cnum = rkFDCDRigidNum(s->cd) + _prp(s)->colnum;

  if( _prp(s)->fsize * _prp(s)->csize < fnum * cnum ){
    zMatFree( _prp(s)->nf );
    _prp(s)->nf = zMatAlloc( cnum, fnum );
    if( !_prp(s)->nf )
      return false;
  } else
    zMatSetSize( _prp(s)->nf, cnum, fnum );

  if( _prp(s)->csize < cnum ){
    zVecFree( _prp(s)->d );
    zIndexFree( _prp(s)->idx );

    _prp(s)->csize = cnum;
    _prp(s)->d   = zVecAlloc( cnum );
    _prp(s)->idx = zIndexCreate( cnum );

    if( !_prp(s)->d || !_prp(s)->idx ){
      zVecFree( _prp(s)->d );
      zIndexFree( _prp(s)->idx );
      return false;
    }
    zVecZero( _prp(s)->d );
  } else
    zVecSetSize( _prp(s)->d, cnum );
  return true;
}

bool _rkFDSolverPrpReAllocModification(rkFDSolver *s)
{
  rkCDPairDat **pd;
  int pvnum = 0;

  rkFDCDForEachRigidPair( s->cd, pd )
    pvnum = zMax( pvnum, zListNum(&(*pd)->cplane) );

  if( _prp(s)->pvsize < pvnum ){
    zMatFree( _prp(s)->ma );
    zVecFreeAO( 2, _prp(s)->mc, _prp(s)->mf );

    _prp(s)->ma = zMatAlloc( 6, rkFDPrpPyramid(s->fdprp) * pvnum );
    _prp(s)->mc = zVecAlloc( pvnum );
    _prp(s)->mf = zVecAlloc( rkFDPrpPyramid(s->fdprp) * pvnum );

    if( !_prp(s)->ma || !_prp(s)->mc || !_prp(s)->mf ){
      zMatFree( _prp(s)->ma );
      zVecFreeAO( 2, _prp(s)->mc, _prp(s)->mf );
      return false;
    }
  }
  return true;
}

/* friction constraint */
static void _rkFDSolverFrictionConstraint(rkFDSolver *s);
void _rkFDSolverFrictionConstraint(rkFDSolver *s){
  rkCDPairDat **pd;
  rkCDPlane *cdpl;
  int ioffset = 0, joffset = 0;

  zMatZero( _prp(s)->nf );
  rkFDCDForEachRigidPair( s->cd, pd ){
    zVec3DCopy( &(*pd)->norm, (zVec3D *)&zMatElemNC(_prp(s)->nf,ioffset++,joffset) );
    zListForEach( &(*pd)->cplane, cdpl ){
      zVec3DMul( &(*pd)->norm, -zVec3DInnerProd( &cdpl->data.norm, &cdpl->data.v ), (zVec3D *)&zMatElemNC(_prp(s)->nf,ioffset,joffset) );
      zVec3DMul( &(*pd)->axis[1],  zVec3DInnerProd( &cdpl->data.norm, &(*pd)->axis[2] ), (zVec3D *)&zMatElemNC(_prp(s)->nf,ioffset,joffset+3) );
      zVec3DCatDRC( (zVec3D *)&zMatElemNC(_prp(s)->nf,ioffset,joffset+3), -zVec3DInnerProd( &cdpl->data.norm, &(*pd)->axis[1] ), &(*pd)->axis[2] );
      ioffset++;
    }
    joffset += 6;
  }
}

/* ************************************************************************** */
/* relation between acc and force */
static void _rkFDSolverBiasAcc(rkFDSolver *s, zVec acc);
static void _rkFDSolverRelativeAcc(rkFDSolver *s, rkCDPairDat *cpd, zVec b, zVec a);
static void _rkFDSolverRelationAccForce(rkFDSolver *s);
void _rkFDSolverBiasAcc(rkFDSolver *s, zVec acc)
{
  rkCDPairDat **pd;
  int offset = 0;
  zVec6D av;

  rkFDCDForEachRigidPair( s->cd, pd ){
    rkFDChainPointRelativeAcc6D( *pd, &(*pd)->center, &av );
    zVec6DCopy( &av, (zVec6D *)&zVecElemNC(acc,offset) );
    offset += 6;
  }
}

void _rkFDSolverRelativeAcc(rkFDSolver *s, rkCDPairDat *cpd, zVec b, zVec a)
{
  rkCDPairDat **pd;
  int offset = 0;
  zVec6D av;

  rkFDCDForEachRigidPair( s->cd, pd ){
    if( (*pd)->cell[0]->data.chain != cpd->cell[0]->data.chain &&
        (*pd)->cell[0]->data.chain != cpd->cell[1]->data.chain &&
        (*pd)->cell[1]->data.chain != cpd->cell[0]->data.chain &&
        (*pd)->cell[1]->data.chain != cpd->cell[1]->data.chain ){
      zVec6DZero( (zVec6D *)&zVecElemNC(a,offset) );
    } else {
      rkFDChainPointRelativeAcc6D( *pd, &(*pd)->center, &av );
      zVec6DSub( &av, (zVec6D *)&zVecElemNC(b,offset), (zVec6D *)&zVecElemNC(a,offset) );
    }
    offset += 6;
  }
}

void _rkFDSolverRelationAccForce(rkFDSolver *s)
{
  rkCDPairDat **pd;
  register int i, j;
  int offset = 0;
  zVec3D pos[2];

  /* b */
  rkFDUpdateAccBias( &s->chains );
  _rkFDSolverBiasAcc( s, _prp(s)->b );
  /* a */
  rkFDCDForEachRigidPair( s->cd, pd ){
    for( j=0; j<2; j++ )
      if( (*pd)->cell[j]->data.type != RK_CD_CELL_STAT )
        zXform3DInv( rkLinkWldFrame((*pd)->cell[j]->data.link), &(*pd)->center, &pos[j] );
    for( i=0; i<6; i++ ){
      for( j=0; j<2; j++ ){
        rkWrenchInit( _prp(s)->w[j] );
        if( (*pd)->cell[j]->data.type == RK_CD_CELL_STAT ) continue;
        rkWrenchSetPos( _prp(s)->w[j], &pos[j] );
        if( i<3 ) /* force */
          zMat3DRow( rkLinkWldAtt((*pd)->cell[j]->data.link), i, rkWrenchForce(_prp(s)->w[j]) );
        else /* torque */
          zMat3DRow( rkLinkWldAtt((*pd)->cell[j]->data.link), i-3, rkWrenchTorque(_prp(s)->w[j]) );
        if( j != 0 )
          zVec6DRevDRC( rkWrenchW(_prp(s)->w[j]) );
      }

      rkFDChainUpdateAccAddExForceTwo( *pd, _prp(s)->w );
      _rkFDSolverRelativeAcc( s, *pd, _prp(s)->b, _prp(s)->t );
      zMatPutCol( _prp(s)->a, offset+i, _prp(s)->t );

      /* restore ABIPrp */
      rkFDChainABIPopPrpExForceTwo( *pd );
    }
    offset += 6;
  }
}

/* bias velocity */
static void _rkFDSolverBiasVel(rkFDSolver *s);
void _rkFDSolverBiasVel(rkFDSolver *s)
{
  rkCDPairDat **pd;
  int offset = 0;
  zVec6D vr;

  zVecMulDRC( _prp(s)->b, rkFDPrpDT(s->fdprp) );
  rkFDCDForEachRigidPair( s->cd, pd ){
    rkFDChainPointRelativeVel6D( *pd, &(*pd)->center, &(*pd)->norm, &vr );
    zVec6DAddDRC( (zVec6D *)&zVecElemNC(_prp(s)->b,offset), &vr );
    offset += 6;
  }
}

/* ************************************************************************** */
/* constraint with each contact volume */
/* TODO: some functions must be converted to macro functions */
static void _rkFDSolverConstraintMidDepth(double h[], rkContactInfo *ci, double s, double hm[], double *hc);
static void _rkFDSolverConstraintMidPoint(zVec3D p[], zVec3D pm[]);
static void _rkFDSolverConstraintAvePoint(zVec3D p[], double s, zVec3D *pc);
static void _rkFDSolverConstraintAveMat(zMat3D m[], double s, zMat3D *mc);
static double _rkFDSolverConstrainArea(zVec3D p[]);
static void _rkFDSolverConstraintAddQ(zVec3D p[], zVec3D pm[], double s, zMat6D *q);
static void _rkFDSolverConstraintDepth(zVec3D pm[], double h[], rkContactInfo *ci, double s, zVec3D norm, zVec6D *c);
static void _rkFDSolverConstraintSignDepth(double h[], int *st, int stp[]);
static void _rkFDSolverConstraintInnerPoint(zVec3D *p1, zVec3D *p2, double h1, double h2, zVec3D *pp);
static void _rkFDSolverSetContactPlane(rkCDPairDat *cpd, zVec3D *p, zVec3D *norm);
static int __rk_fd_plane_cmp(void *p1, void *p2, void *priv);
static void _rkFDSolverConstraint(rkCDPairDat *cpd, zMat6D *q, zVec6D *c);

void _rkFDSolverConstraintMidDepth(double h[], rkContactInfo *ci, double s, double hm[], double *hc)
{
  double k;

  k = rkContactInfoK(ci) * s / 6.0;
  hm[0] = k * ( h[0] + h[1]        );
  hm[1] = k * (        h[1] + h[2] );
  hm[2] = k * ( h[0]        + h[2] );
  *hc   = k * ( h[0] + h[1] + h[2] ) * 2;
}

void _rkFDSolverConstraintMidPoint(zVec3D p[], zVec3D pm[])
{
  zVec3DMid( &p[0], &p[1], &pm[0] );
  zVec3DMid( &p[1], &p[2], &pm[1] );
  zVec3DMid( &p[2], &p[0], &pm[2] );
}

void _rkFDSolverConstraintAvePoint(zVec3D p[], double s, zVec3D *pc)
{
  double k;

  k = s / 3.0;
  _zVec3DAdd( &p[0], &p[1], pc );
  _zVec3DAddDRC( pc, &p[2] );
  _zVec3DMulDRC( pc, k );
}

void _rkFDSolverConstraintAveMat(zMat3D m[], double s, zMat3D *mc)
{
  double k;

  k = s / 3.0;
  _zMat3DAdd( &m[0], &m[1], mc );
  _zMat3DAddDRC( mc, &m[2] );
  _zMat3DMulDRC( mc, k );

}

double _rkFDSolverConstrainArea(zVec3D p[])
{
  zVec3D e1, e2;

  zVec3DSub( &p[1], &p[0], &e1 );
  zVec3DSub( &p[2], &p[0], &e2 );
  return 0.5 * zVec3DOuterProdNorm( &e1, &e2 );
}

void _rkFDSolverConstraintAddQ(zVec3D p[], zVec3D pm[], double s, zMat6D *q)
{
  register int i;
  zVec3D pc;
  zMat3D mm[3], tmpm;

  _rkFDSolverConstraintAvePoint( p, s, &pc );
  zMat3DMul( ZMAT3DIDENT, s, &tmpm );
  zMat3DAddDRC( &q->e[0][0], &tmpm );
  _zVec3DOuterProd2Mat3D( &pc, &tmpm );
  zMat3DAddDRC( &q->e[1][0], &tmpm );
  zMat3DSubDRC( &q->e[0][1], &tmpm );
  for( i=0; i<3; i++ ) _zVec3DTripleProd2Mat3D( &pm[i], &pm[i], &mm[i] );
  _rkFDSolverConstraintAveMat( mm, s, &tmpm );
  zMat3DSubDRC( &q->e[1][1], &tmpm );
}

void _rkFDSolverConstraintDepth(zVec3D pm[], double h[], rkContactInfo *ci, double s, zVec3D norm, zVec6D *c)
{
  register int i;
  double hm[3], hc;
  zVec3D tmpv;

  _rkFDSolverConstraintMidDepth( h, ci, s, hm, &hc );
  zVec3DMul( &norm, -hc, zVec6DLin(c) );
  zVec3DZero( zVec6DAng(c) );
  for( i=0; i<3; i++ ){
    zVec3DMulDRC( &pm[i], hm[i] );
    zVec3DOuterProd( &norm, &pm[i], &tmpv );
    zVec3DAddDRC( zVec6DAng(c), &tmpv );
  }
}

void _rkFDSolverConstraintSignDepth(double h[], int *st, int stp[])
{
  register int i;

  *st = 0;
  stp[0] = stp[1] = stp[2] = 0;
  for( i=0; i<3; i++ ){
    if( h[i] > zTOL ){
      *st += 1<<(i*2);
      stp[1] = i;
    } else if( h[i] < -zTOL ){
      *st += 1<<(i*2+1);
      stp[2] = i;
    } else{
      stp[0] = i;
    }
  }
}

void _rkFDSolverConstraintInnerPoint(zVec3D *p1, zVec3D *p2, double h1, double h2, zVec3D *pp)
{
  /* for safety */
  if( zIsTiny( h1 ) ){
    zVec3DCopy( p1, pp );
    return;
  }
  if( zIsTiny( h2 ) ){
    zVec3DCopy( p2, pp );
    return;
  }
  if( zIsTiny( h2 - h1 ) ){
    zVec3DMid( p1, p2, pp );
    return;
  }
  zVec3DMul( p1, h2/(h2 - h1), pp );
  zVec3DCatDRC( pp, h1/(h1 - h2), p2 );
}

void _rkFDSolverSetContactPlane(rkCDPairDat *cpd, zVec3D *p, zVec3D *norm)
{
  rkCDPlane *cdpl, *cdpl2;
  zVec3D tmpv;

  zVec3DCat( norm, -zVec3DInnerProd( norm, &cpd->norm ), &cpd->norm, &tmpv );
  if( zVec3DIsTiny( &tmpv ) ) return;
  cdpl = zAlloc( rkCDPlane, 1 );
  zVec3DCopy( p, &cdpl->data.v );
  zVec3DDiv( &tmpv, -zVec3DNorm( &tmpv ) , &cdpl->data.norm );

  /* to eliminate identical conditions */
  zListForEach( &cpd->cplane, cdpl2 ){
    if( zVec3DIsTol( zVec3DSub( &cdpl->data.norm, &cdpl2->data.norm, &tmpv ), 1e-8 ) &&
        zIsTol( zVec3DInnerProd( &cdpl->data.norm, zVec3DSub( &cdpl2->data.v, &cdpl->data.v, &tmpv ) ), 1e-8 ) ){
      zVec3DOuterProd( &cdpl->data.norm, &tmpv, &tmpv );
      if( zVec3DInnerProd( &cpd->norm, &tmpv ) > 0.0 ){
        zVec3DCopy( &cdpl->data.v, &cdpl2->data.v );
      }
      zFree( cdpl );
      return;
    }
  }
  zListInsertHead( &cpd->cplane, cdpl );
}

int __rk_fd_plane_cmp(void *p1, void *p2, void *priv)
{
  zVec3D *a, *n, tmp;
  double th1, th2;

  n = (zVec3D *)priv;
  a = (zVec3D *)priv + 1;
  zVec3DOuterProd( a, &((rkCDPlane *)p1)->data.norm, &tmp );
  if( zVec3DInnerProd( &tmp, n ) > 0 )
    th1 = atan2( -zVec3DNorm( &tmp ), zVec3DInnerProd( a, &((rkCDPlane *)p1)->data.norm ) );
  else
    th1 = atan2(  zVec3DNorm( &tmp ), zVec3DInnerProd( a, &((rkCDPlane *)p1)->data.norm ) );
  zVec3DOuterProd( a, &((rkCDPlane *)p2)->data.norm, &tmp );
  if( zVec3DInnerProd( &tmp, n ) > 0 )
    th2 = atan2( -zVec3DNorm( &tmp ), zVec3DInnerProd( a, &((rkCDPlane *)p2)->data.norm ) );
  else
    th2 = atan2(  zVec3DNorm( &tmp ), zVec3DInnerProd( a, &((rkCDPlane *)p2)->data.norm ) );
  if( zIsTiny( th1 - th2 ) ) return 0;
  return th1 > th2 ? 1: -1;
}

void _rkFDSolverConstraint(rkCDPairDat *cpd, zMat6D *q, zVec6D *c)
{
  register int i, j;
  zTri3D *face;
  double h[3], s;
  zVec3D pf[3], p[3], pm[3], pp[2];
  zVec6D cc;
  int stp[3], st;

  zMat6DZero( q );
  zVec6DZero( c );
  for( i=0; i<zPH3DFaceNum(&cpd->colvol); i++ ){
    face = zPH3DFace(&cpd->colvol,i);
    for( j=0; j<3; j++ ){
      zVec3DSub( zTri3DVert(face,j), &cpd->center, &pf[j] );
      h[j] = zVec3DInnerProd( &cpd->norm, &pf[j] );
      zVec3DCat( &pf[j], -h[j], &cpd->norm, &p[j] );
    }
    _rkFDSolverConstraintMidPoint( p, pm );
    s = _rkFDSolverConstrainArea( p );
    _rkFDSolverConstraintAddQ( p, pm, s, q ); /* q */
    _rkFDSolverConstraintDepth( pm, h, cpd->ci, s, cpd->norm, &cc ); /* c */

    /* a process dependent on signum of h */
    _rkFDSolverConstraintSignDepth( h, &st, stp );
    switch( st ){
    case 0x01: case 0x04: case 0x10: /* two points are on the plane and one point is above it */
    case 0x05: case 0x11: case 0x14: /* one point is on the plane and two points are above it */
      _rkFDSolverSetContactPlane( cpd, &pf[stp[0]], zTri3DNorm(face) );
    case 0x15: /* all points are above the plane */
      zVec6DAddDRC( c, &cc );
      continue;
    case 0x02: case 0x08: case 0x20: /* two points are on the line and one point is below it */
    case 0x0a: case 0x22: case 0x28: /* one point is on the line and two points are below it */
      _rkFDSolverSetContactPlane( cpd, &pf[stp[0]], zTri3DNorm(face) );
    case 0x2a: /* all points are below the plane */
      zVec6DSubDRC( c, &cc );
      continue;
    case 0x24: /* 0:on    1:above 2:below */
    case 0x12: /* 0:below 1:on    2:above */
    case 0x09: /* 0:above 1:below 2:on    */
      zVec6DAddDRC( c, &cc );
      _rkFDSolverConstraintInnerPoint( &pf[stp[1]], &pf[stp[2]], h[stp[1]], h[stp[2]], &p[stp[1]] );
      h[stp[1]] = 0.0;
      zVec3DCopy( &p[stp[0]], &pp[0] );
      zVec3DCopy( &p[stp[1]], &pp[1] );
      break;
    case 0x06: /* 0:below 1:above 2:on    */
    case 0x21: /* 0:above 1:on    2:below */
    case 0x18: /* 0:on    1:below 2:above */
      zVec6DAddDRC( c, &cc );
      _rkFDSolverConstraintInnerPoint( &pf[stp[1]], &pf[stp[2]], h[stp[1]], h[stp[2]], &p[stp[2]] );
      h[stp[2]] = 0.0;
      zVec3DCopy( &p[stp[2]], &pp[0] );
      zVec3DCopy( &p[stp[0]], &pp[1] );
      break;
    case 0x16:case 0x19:case 0x25: /* two points are above the plane and one point is below it */
      stp[0] = (stp[2]+1) % 3;
      stp[1] = (stp[0]+1) % 3;
      zVec6DAddDRC( c, &cc );
      _rkFDSolverConstraintInnerPoint( &pf[stp[2]], &pf[stp[0]], h[stp[2]], h[stp[0]], &p[stp[0]] );
      _rkFDSolverConstraintInnerPoint( &pf[stp[2]], &pf[stp[1]], h[stp[2]], h[stp[1]], &p[stp[1]] );
      h[stp[0]] = h[stp[1]] = 0.0;
      zVec3DCopy( &p[stp[0]], &pp[0] );
      zVec3DCopy( &p[stp[1]], &pp[1] );
      break;
    case 0x1a:case 0x26:case 0x29: /* two points are below the plane and one point is above it */
      stp[0] = (stp[1]+1) % 3;
      stp[2] = (stp[0]+1) % 3;
      zVec6DSubDRC( c, &cc );
      _rkFDSolverConstraintInnerPoint( &pf[stp[1]], &pf[stp[0]], h[stp[1]], h[stp[0]], &p[stp[0]] );
      _rkFDSolverConstraintInnerPoint( &pf[stp[1]], &pf[stp[2]], h[stp[1]], h[stp[2]], &p[stp[2]] );
      h[stp[0]] = h[stp[2]] = 0.0;
      zVec3DCopy( &p[stp[2]], &pp[0] );
      zVec3DCopy( &p[stp[0]], &pp[1] );
      break;
    default: /* all points are on the plane */
      continue;
    }
    s = _rkFDSolverConstrainArea( p );
    _rkFDSolverConstraintMidPoint( p, pm );
    _rkFDSolverConstraintDepth( pm, h, cpd->ci, s, cpd->norm, &cc );
    switch( st ){
    case 0x1a: case 0x26: case 0x29:
      zVec6DCatDRC( c, 2.0, &cc );
      break;
    default:
      zVec6DCatDRC( c, -2.0, &cc );
    }
    /* register pp */
    _rkFDSolverSetContactPlane( cpd, &pp[0], zTri3DNorm(face) );
  }
  /* sort plane list */
  rkCDPlaneListQuickSort( &cpd->cplane, __rk_fd_plane_cmp, cpd->axis );
}

/* ************************************************************************** */
/* solve QP */
static void _rkFDSolverQPCreate(rkFDSolver *s);
static zVec _rkFDSolverQPInit(zMat a, zVec b, zVec ans, void *util);
static void _rkFDSolverQP(rkFDSolver *s);

void _rkFDSolverQPCreate(rkFDSolver *s)
{
  zMat6D qv;
  zVec6D cv, tmpv;
  rkCDPairDat **pd;
  register int i, j;
  int offset = 0;

  zMatZero( _prp(s)->q );
  zVecZero( _prp(s)->c );
  /* evaluation function */
  rkFDCDForEachRigidPair( s->cd, pd ){
    _rkFDSolverConstraint( *pd, &qv, &cv );
    /* q */
    for( i=0; i<6; i++ )
      for( j=0; j<6; j++ )
        zRawMatCatDyad( zMatBuf(_prp(s)->q), _zMat6DElem(&qv,i,j), zMatRowBuf(_prp(s)->a,offset+i), zMatColSizeNC(_prp(s)->a), zMatRowBuf(_prp(s)->a,offset+j), zMatColSizeNC(_prp(s)->a) );
    /* c */
    zMulMat6DVec6D( &qv, (zVec6D *)&zVecElemNC(_prp(s)->b,offset), &tmpv );
    zVec6DAddDRC( &cv, &tmpv );
    for( i=0; i<6; i++ )
      zRawVecCatDRC( zVecBuf(_prp(s)->c), cv.e[i], zMatRowBuf(_prp(s)->a,offset+i), zMatColSizeNC(_prp(s)->a) );
    offset += 6;

  }
  /* relaxation */
  offset = 0;
  rkFDCDForEachRigidPair( s->cd, pd ){
    for( i=0; i<6; i++ )
      zMatElemNC(_prp(s)->q,offset+i,offset+i) += rkContactInfoL((*pd)->ci);
    offset += 6;
  }
}

zVec _rkFDSolverQPInit(zMat a, zVec b, zVec ans, void *util){
  rkCDPairDat **pd;
  int offset = 0;

  zVecZero( ans );
  rkFDCDForEachRigidPair( ((rkFDSolver*)util)->cd, pd ){
    /* normal forces initial offset */
    zVec3DMul( &(*pd)->norm, 1.0, (zVec3D *)&zVecElemNC(ans,offset) );
    offset += 6;
  }
  return ans;
}

void _rkFDSolverQP(rkFDSolver *s)
{
  zQPSolveASM( _prp(s)->q, _prp(s)->c, _prp(s)->nf, _prp(s)->d, _prp(s)->f, NULL, _rkFDSolverQPInit, s );
  zVecDivDRC( _prp(s)->f, rkFDPrpDT(s->fdprp) );
}

/* ************************************************************************** */
/* set force to rkCDPair */
static void _rkFDSolverSetForce(rkFDSolver *s);
void _rkFDSolverSetForce(rkFDSolver *s)
{
  rkCDPairDat **pd;
  int offset =  0;

  rkFDCDForEachRigidPair( s->cd, pd ){
    if( zListNum(&(*pd)->cplane) == 0 ){
      zVec6DZero( &(*pd)->f );
      continue;
    }
    zVec6DCopy( (zVec6D *)&zVecElemNC(_prp(s)->f,offset), &(*pd)->f );
    if( zVec3DIsTiny( zVec6DLin(&(*pd)->f) ) || zVec3DInnerProd( zVec6DLin(&(*pd)->f), &(*pd)->norm ) < zTOL ){
      zVec6DZero( &(*pd)->f );
    }
    offset += 6;
  }
}

/* ************************************************************************** */
/* normal conditions */
static void _rkFDSolverModifyNormForceCenterTrq(rkCDPairDat *cpd, zVec3D *r, double fn);
static void _rkFDSolverModifyNormalForceCenter(rkFDSolver *solver);

void _rkFDSolverModifyNormForceCenterTrq(rkCDPairDat *cpd, zVec3D *r, double fn)
{
  zVec3DMul( &cpd->norm, zVec3DInnerProd( &cpd->norm, zVec6DAng(&cpd->f) ), zVec6DAng(&cpd->f) );
  zVec3DCatDRC( zVec6DAng(&cpd->f),  fn * zVec3DInnerProd( &cpd->axis[2], r ), &cpd->axis[1] );
  zVec3DCatDRC( zVec6DAng(&cpd->f), -fn * zVec3DInnerProd( &cpd->axis[1], r ), &cpd->axis[2] );
}

void _rkFDSolverModifyNormalForceCenter(rkFDSolver *solver)
{
  rkCDPairDat **pd;
  rkCDPlane *cdpl[4];
  zVec3D r0, r, dir, tmp;
  double fn, d, s;
  bool flag = false;

  rkFDCDForEachRigidPair( solver->cd, pd ){
    /* unilateral constraint */
    fn = zVec3DInnerProd( &(*pd)->norm, zVec6DLin(&(*pd)->f) );
    if( fn < zTOL ) continue;

    zVec3DMul( &(*pd)->axis[1], -zVec3DInnerProd( &(*pd)->axis[2], zVec6DAng(&(*pd)->f) ) / fn, &r0 );
    zVec3DCatDRC( &r0, zVec3DInnerProd( &(*pd)->axis[1], zVec6DAng(&(*pd)->f) ) / fn, &(*pd)->axis[2] );
    flag = false;
    cdpl[2] = zListHead( &(*pd)->cplane );
    cdpl[1] = zListCellPrev( cdpl[2] );
    cdpl[0] = zListCellPrev( cdpl[1] );
    for( cdpl[3]=zListTail(&(*pd)->cplane); cdpl[3]!=zListRoot(&(*pd)->cplane);
         cdpl[0]=cdpl[1],cdpl[1]=cdpl[2],cdpl[2]=cdpl[3],cdpl[3]=zListCellNext(cdpl[3]) ){
      zVec3DSub( &cdpl[2]->data.v, &cdpl[1]->data.v, &dir );
      d = zVec3DSqrNorm( &dir );
      if( zIsTiny( d ) ) continue;
      zVec3DSub( &r0, &cdpl[1]->data.v, &tmp );
      if( zVec3DInnerProd( &tmp, &cdpl[1]->data.norm ) > zTOL ) continue;
      s = zVec3DInnerProd( &dir, &tmp ) / d;
      if( s < zTOL ){
        if( flag ) break;
        /* r: cdpl[1]->data.v */
        zVec3DSub( &cdpl[0]->data.v, &cdpl[1]->data.v, &tmp );
        zVec3DAddDRC( &tmp, &dir );
        zVec3DCat( &cdpl[1]->data.v, zTOL / zVec3DNorm( &tmp ), &tmp, &r );
        _rkFDSolverModifyNormForceCenterTrq( *pd, &r, fn );
        break;
      } else if ( s < 1.0-zTOL ){
        /* r: projected position */
        zVec3DCat( &cdpl[1]->data.v, s, &dir, &r );
        zVec3DCatDRC( &r, zTOL, &cdpl[1]->data.norm );
        _rkFDSolverModifyNormForceCenterTrq( *pd, &r, fn );
        break;
      } else {
        /* r: cdpl[2]->data.v */
        zVec3DSub( &cdpl[3]->data.v, &cdpl[2]->data.v, &tmp );
        zVec3DSubDRC( &tmp, &dir );
        zVec3DCat( &cdpl[2]->data.v, zTOL / zVec3DNorm( &tmp ), &tmp, &r );
        _rkFDSolverModifyNormForceCenterTrq( *pd, &r, fn );
        flag = true;
      }
    }
  }
}

/* ************************************************************************** */
/* friction conditions */
/* NOTE:
 * the ASM solver does not work well occasionally.
 * in that case, the solution does not meet the constraints with respect to normal force.
 * before the friction modification, the solution is projected inside the constraints with a small margin.
 */
/* static friction */
static void _rkFDSolverModifyWrenchStaticSetSize(rkFDSolver *s, rkCDPairDat *cpd);
static void _rkFDSolverModifyWrenchStaticConstraint(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w);
static bool _rkFDSolverModifyWrenchStatic(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w);
static void _rkFDSolverModifyWrenchSetStatic(rkCDPairDat *cpd, bool doUpRef);

void _rkFDSolverModifyWrenchStaticSetSize(rkFDSolver *s, rkCDPairDat *cpd)
{
  int fnum = rkFDPrpPyramid(s->fdprp) * zListNum(&cpd->cplane);

  zMatSetSize( _prp(s)->ma, 6, fnum );
  zVecSetSize( _prp(s)->mb, 6 );
  zVecSetSize( _prp(s)->mf, fnum );
}

void _rkFDSolverModifyWrenchStaticConstraint(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w)
{
  rkCDPlane *cdpl;
  register int i;
  int offset = 0;

  zListForEach( &cpd->cplane, cdpl ){
    for( i=0; i<rkFDPrpPyramid(s->fdprp); i++ ){
      zMatElemNC(_prp(s)->ma,0,offset+i) = 1.0;
      zMatElemNC(_prp(s)->ma,1,offset+i) = cdpl->data.r[1];
      zMatElemNC(_prp(s)->ma,2,offset+i) = -cdpl->data.r[0];
      zMatElemNC(_prp(s)->ma,3,offset+i) = rkContactInfoSF(cpd->ci)*zVecElemNC(_prp(s)->sc_table[1],i);
      zMatElemNC(_prp(s)->ma,4,offset+i) = rkContactInfoSF(cpd->ci)*zVecElemNC(_prp(s)->sc_table[0],i);
      zMatElemNC(_prp(s)->ma,5,offset+i) = -(zMatElemNC(_prp(s)->ma,2,offset+i)*zMatElemNC(_prp(s)->ma,4,offset+i) + zMatElemNC(_prp(s)->ma,1,offset+i)*zMatElemNC(_prp(s)->ma,3,offset+i));
    }
    offset += rkFDPrpPyramid(s->fdprp);
  }
  zVecElemNC(_prp(s)->mb,0) = w->e[0];
  zVecElemNC(_prp(s)->mb,1) = w->e[4];
  zVecElemNC(_prp(s)->mb,2) = w->e[5];
  zVecElemNC(_prp(s)->mb,3) = w->e[1];
  zVecElemNC(_prp(s)->mb,4) = w->e[2];
  zVecElemNC(_prp(s)->mb,5) = w->e[3];
}

bool _rkFDSolverModifyWrenchStatic(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w)
{
  bool ret;

  _rkFDSolverModifyWrenchStaticSetSize( s, cpd );
  _rkFDSolverModifyWrenchStaticConstraint( s, cpd, w );

  zEchoOff();
  ret = zLPFeasibleBase( _prp(s)->ma, _prp(s)->mb, _prp(s)->mf );
  zEchoOn();

  return ret;
}

/* if this function's overhead is large, this function should be expanded */
void _rkFDSolverModifyWrenchSetStatic(rkCDPairDat *cpd, bool doUpRef)
{
  if( doUpRef ){
    cpd->type = RK_CONTACT_SF;
  }
}

/* kinetic friction */
static void _rkFDSolverPlaneVertPos(rkCDPairDat *cpd, double *tl);
static void _rkFDSolverModifyWrenchKineticCenter(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w);
static void _rkFDSolverModifyWrenchKineticSetSize(rkFDSolver *s, rkCDPairDat *cpd);
static void _rkFDSolverModifyWrenchKineticConstraint(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w);
static void _rkFDSolverPlaneVertSlideDir(rkFDSolver *s, rkCDPairDat *cpd, rkCDPlane *cdpl);
static void _rkFDSolverModifyWrenchKineticEvalFunc(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w);
static void _rkFDSolverModifyWrenchKineticEvalFuncSafaty(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w);
static void _rkFDSolverModifyWrenchKineticTotalWrench(rkCDPairDat *cpd, zVec f, zVec6D *w);
static void _rkFDSolverModifyWrenchKinetic(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w);
static void _rkFDSolverModifyWrenchSetKinetic(rkCDPairDat *cpd, bool doUpRef);

void _rkFDSolverPlaneVertPos(rkCDPairDat *cpd, double *tl)
{
  rkCDPlane *cdpl;
  double rl;

  *tl = 0.0;
  zListForEach( &cpd->cplane, cdpl ){
    cdpl->data.r[0] = zVec3DInnerProd( &cdpl->data.v, &cpd->axis[1] );
    cdpl->data.r[1] = zVec3DInnerProd( &cdpl->data.v, &cpd->axis[2] );
    if( *tl < (rl = zVec2DNorm( cdpl->data.r )) )
      *tl = rl;
  }
}

void _rkFDSolverModifyWrenchKineticCenter(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w)
{
  zVec3D v;
  double nv, tmpd;

  rkFDChainPointRelativeVel( cpd, &cpd->center, &cpd->norm, cpd->cell[0], &v );
  zVec3DCatDRC( &v, -zVec3DInnerProd( &cpd->norm, &v ), &cpd->norm );
  nv = zVec3DNorm( &v );
  if( zIsTiny( nv ) ){
    w->e[1] = 0.0;
    w->e[2] = 0.0;
  } else {
    tmpd = rkFDKineticFrictionWeight( rkFDPrpFricWeight(s->fdprp), nv ) * rkContactInfoKF(cpd->ci) * w->e[0] / nv;
    w->e[1] = -tmpd * zVec3DInnerProd( &v, &cpd->axis[1] );
    w->e[2] = -tmpd * zVec3DInnerProd( &v, &cpd->axis[2] );
  }
}

void _rkFDSolverModifyWrenchKineticSetSize(rkFDSolver *s, rkCDPairDat *cpd)
{
  int fnum = zListNum(&cpd->cplane);

  zMatSetSize( _prp(s)->ma, 3, fnum );
  zVecSetSize( _prp(s)->mb, 3 );
  zVecSetSize( _prp(s)->mc, fnum );
  zVecSetSize( _prp(s)->mf, fnum );
}

void _rkFDSolverModifyWrenchKineticConstraint(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w)
{
  rkCDPlane *cdpl;
  int offset = 0;

  zListForEach( &cpd->cplane, cdpl ){
    zMatElemNC(_prp(s)->ma,0,offset) = 1.0;
    zMatElemNC(_prp(s)->ma,1,offset) = cdpl->data.r[1];
    zMatElemNC(_prp(s)->ma,2,offset) = -cdpl->data.r[0];
    offset++;
  }
  zVecElemNC(_prp(s)->mb,0) = w->e[0];
  zVecElemNC(_prp(s)->mb,1) = w->e[4];
  zVecElemNC(_prp(s)->mb,2) = w->e[5];
}

void _rkFDSolverPlaneVertSlideDir(rkFDSolver *s, rkCDPairDat *cpd, rkCDPlane *cdpl)
{
  zVec3D p, v;
  double nv, w;

  zVec3DAdd( &cpd->center, &cdpl->data.v, &p );
  rkFDChainPointRelativeVel( cpd, &p, &cpd->norm, cpd->cell[0], &v );
  zVec3DCatDRC( &v, -zVec3DInnerProd( &cpd->norm, &v ), &cpd->norm );
  nv = zVec3DNorm( &v );
  if( zIsTiny( nv ) ){
    cdpl->data.s[0] = 0.0;
    cdpl->data.s[1] = 0.0;
  } else {
    w = rkFDKineticFrictionWeight( rkFDPrpFricWeight(s->fdprp), nv ) * rkContactInfoKF(cpd->ci) / nv;
    cdpl->data.s[0] = -w * zVec3DInnerProd( &v, &cpd->axis[1] );
    cdpl->data.s[1] = -w * zVec3DInnerProd( &v, &cpd->axis[2] );
  }
}

void _rkFDSolverModifyWrenchKineticEvalFunc(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w)
{
  rkCDPlane *cdpl;
  register int i;
  int offset = 0;
  double wn[3];

  for( i=0; i<3; i++ )
    if( !zIsTiny( w->e[i+1] ) )
      wn[i] = 1.0 / w->e[i+1];
    else
      wn[i] = 0.0;
  zListForEach( &cpd->cplane, cdpl ){
    _rkFDSolverPlaneVertSlideDir( s, cpd, cdpl );
    zVecElemNC(_prp(s)->mc,offset) = -wn[0] * cdpl->data.s[0] - wn[1] * cdpl->data.s[1] -
      wn[2] * ( cdpl->data.r[0] * cdpl->data.s[1] - cdpl->data.r[1] * cdpl->data.s[0] );
    offset++;
  }
}

void _rkFDSolverModifyWrenchKineticEvalFuncSafaty(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w)
{
  rkCDPlane *cdpl;
  register int i;
  int offset = 0;
  double wn[2];

  zMatSetSize( _prp(s)->ma, 1, zListNum(&cpd->cplane) );
  zVecSetSize( _prp(s)->mb, 1 );
  for( i=0; i<2; i++ )
    if( !zIsTiny( w->e[i+3] ) )
      wn[i] = 1.0 / w->e[i+3];
    else
      wn[i] = 0.0;
  offset = 0;
  zListForEach( &cpd->cplane, cdpl ){
    _rkFDSolverPlaneVertSlideDir( s, cpd, cdpl );
    zVecElemNC(_prp(s)->mc,offset) += wn[0] * cdpl->data.r[0] - wn[1] * cdpl->data.r[1];
    offset++;
  }
}

void _rkFDSolverModifyWrenchKineticTotalWrench(rkCDPairDat *cpd, zVec f, zVec6D *w)
{
  rkCDPlane *cdpl;
  int offset = 0;
  zVec2D fs;

  zVec3DZero( (zVec3D *)&w->e[1] );
  zListForEach( &cpd->cplane, cdpl ){
    zVec2DMul( cdpl->data.s, zVecElemNC(f,offset), fs );
    w->e[1] += fs[0];
    w->e[2] += fs[1];
    w->e[3] += cdpl->data.r[0] * fs[1] - cdpl->data.r[1] * fs[0];
    offset++;
  }
}

void _rkFDSolverModifyWrenchKinetic(rkFDSolver *s, rkCDPairDat *cpd, zVec6D *w)
{
  _rkFDSolverModifyWrenchKineticSetSize( s, cpd );
  _rkFDSolverModifyWrenchKineticConstraint( s, cpd, w );
  _rkFDSolverModifyWrenchKineticEvalFunc( s, cpd, w );

  zEchoOff();
  if( !zLPSolveSimplex( _prp(s)->ma, _prp(s)->mb, _prp(s)->mc, _prp(s)->mf, NULL ) ){
    _rkFDSolverModifyWrenchKineticEvalFuncSafaty( s, cpd, w );
    zLPSolveSimplex( _prp(s)->ma, _prp(s)->mb, _prp(s)->mc, _prp(s)->mf, NULL );
  }
  zEchoOn();

  _rkFDSolverModifyWrenchKineticTotalWrench( cpd, _prp(s)->mf, w );
}

/* if this function's overhead is large, this function should be expanded */
void _rkFDSolverModifyWrenchSetKinetic(rkCDPairDat *cpd, bool doUpRef)
{
  if( doUpRef ){
    cpd->type = RK_CONTACT_KF;
    /* this version doesn't use the reference frame */
    /* zFrame3DCopy( rkLinkWldFrame(cpd->cell[0]->data.link), &cpd->ref[0] ); */
    /* zFrame3DCopy( rkLinkWldFrame(cpd->cell[1]->data.link), &cpd->ref[1] ); */
  }
}

/* modify wrench */
static void _rkFDSolverModifyWrenchSetForce(rkCDPairDat *cpd, zVec6D *w);
static void _rkFDSolverModifyWrench(rkFDSolver *s, bool doUpRef);

void _rkFDSolverModifyWrenchSetForce(rkCDPairDat *cpd, zVec6D *w)
{
  register int i;

  zVec6DZero( &cpd->f );
  for( i=0; i<3; i++ ){
    zVec3DCatDRC( zVec6DLin(&cpd->f), w->e[i  ], &cpd->axis[i] );
    zVec3DCatDRC( zVec6DAng(&cpd->f), w->e[i+3], &cpd->axis[i] );
  }
}

void _rkFDSolverModifyWrench(rkFDSolver *s, bool doUpRef)
{
  rkCDPairDat **pd;
  zVec6D w;
  double fn, fs, tl;
  register int i;

  rkFDCDForEachRigidPair( s->cd, pd ){
    if( zListIsEmpty(&(*pd)->cplane) ) continue;
    if( zIsTiny( zVec3DInnerProd( zVec6DLin(&(*pd)->f), &(*pd)->axis[0] ) ) ) continue;
    for( i=0; i<3; i++ ){
      w.e[i  ] = zVec3DInnerProd( zVec6DLin(&(*pd)->f), &(*pd)->axis[i] );
      w.e[i+3] = zVec3DInnerProd( zVec6DAng(&(*pd)->f), &(*pd)->axis[i] );
    }

    /* friction */
    fn = w.e[0];
    fs = sqrt( zSqr( w.e[1] ) + zSqr( w.e[2] ) );
    _rkFDSolverPlaneVertPos( *pd, &tl );
    if( zIsTiny( tl ) ){
      /* the plane is too small */
      zVec3DZero( zVec6DAng(&w) );
      if( !zIsTiny( fs ) && fs > rkContactInfoSF((*pd)->ci) * fn ){
        _rkFDSolverModifyWrenchKineticCenter( s, *pd, &w );
        _rkFDSolverModifyWrenchSetKinetic( *pd, doUpRef );
      } else
        _rkFDSolverModifyWrenchSetStatic( *pd, doUpRef );
      _rkFDSolverModifyWrenchSetForce( *pd, &w );
      continue;
    }

    if( ( !zIsTiny( fs ) && fs > rkContactInfoSF((*pd)->ci) * fn ) || fabs(w.e[3]) > tl * w.e[0] ){
      /* kinetic friction */
      _rkFDSolverModifyWrenchKinetic( s, *pd, &w );
      _rkFDSolverModifyWrenchSetKinetic( *pd, doUpRef );
      _rkFDSolverModifyWrenchSetForce( *pd, &w );
    } else {
      if( _rkFDSolverModifyWrenchStatic( s, *pd, &w ) )
        /* static friction */
        _rkFDSolverModifyWrenchSetStatic( *pd, doUpRef );
      else {
        /* kinetic friction */
        _rkFDSolverModifyWrenchKinetic( s, *pd, &w );
        _rkFDSolverModifyWrenchSetKinetic( *pd, doUpRef );
        _rkFDSolverModifyWrenchSetForce( *pd, &w );
      }
    }
  }
}

/* ************************************************************************** */
static void _rkFDSolverPushWrench(rkFDSolver *s);
void _rkFDSolverPushWrench(rkFDSolver *s)
{
  rkCDPairDat **pd;
  rkWrench *w;
  register int i;

  rkFDCDForEachRigidPair( s->cd, pd ){
    for( i=0; i<2; i++ ){
      w = zAlloc( rkWrench, 1 );
      rkWrenchInit( w );
      zXform3DInv( rkLinkWldFrame((*pd)->cell[i]->data.link), &(*pd)->center, rkWrenchPos(w) );
      zMulMat3DTVec6D( rkLinkWldAtt((*pd)->cell[i]->data.link), &(*pd)->f, rkWrenchW(w) );
      if( i == 1 )
        zVec6DRevDRC( rkWrenchW(w) );
      rkWrenchListPush( rkLinkExtWrenchBuf((*pd)->cell[i]->data.link), w );
    }
  }
}

/* ************************************************************************** */
static bool _rkFDSolverVolume(rkFDSolver *s, bool doUpRef);
bool _rkFDSolverVolume(rkFDSolver *s, bool doUpRef)
{
  if( !_rkFDSolverPrpReAllocQP( s ) ) return false;
  _rkFDSolverRelationAccForce( s );
  _rkFDSolverBiasVel( s );

  _rkFDSolverQPCreate( s );
  _rkFDSolverCountContacts( s );
  if( !_rkFDSolverPrpReAllocQPCondition( s ) ) return false;
  _rkFDSolverFrictionConstraint( s );
  _rkFDSolverQP( s );
  _rkFDSolverSetForce( s );

  if( !_rkFDSolverPrpReAllocModification( s ) ) return false;
  _rkFDSolverModifyNormalForceCenter( s );
  _rkFDSolverModifyWrench( s, doUpRef );
  _rkFDSolverPushWrench( s );
  return true;
}

/*****************************************************************************/
/* abstruct functions */
void rkFDSolverGetDefaultContactInfo_Volume(rkFDSolver *s, rkContactInfo *ci)
{
  rkContactInfoInit( ci );
  rkContactInfoSetType( ci, RK_CONTACT_RIGID );
  rkContactInfoSetK( ci, 1000.0 );
  rkContactInfoSetL( ci, 0.001 );
  rkContactInfoSetSF( ci, 0.5 );
  rkContactInfoSetKF( ci, 0.3 );
}

bool rkFDSolverUpdateInit_Volume(rkFDSolver *s){
  _prp(s)->colnum = 0;
  _prp(s)->a = NULL;
  _prp(s)->b = NULL;
  _prp(s)->t = NULL;

  _prp(s)->fsize = 0;
  _prp(s)->csize = 0;
  _prp(s)->q   = NULL;
  _prp(s)->nf  = NULL;
  _prp(s)->c   = NULL;
  _prp(s)->d   = NULL;
  _prp(s)->f   = NULL;
  _prp(s)->idx = NULL;

  _prp(s)->w[0] = zAlloc( rkWrench, 1 );
  _prp(s)->w[1] = zAlloc( rkWrench, 1 );
  if( !_prp(s)->w[0] || !_prp(s)->w[1] ) return false;

  _prp(s)->pvsize = 0;
  _prp(s)->ma = NULL;
  _prp(s)->mb = zVecAlloc( 6 );
  _prp(s)->mc = NULL;
  _prp(s)->mf = NULL;

  if( !_prp(s)->mb ) return false;

  return rkFDCrateSinCosTable( _prp(s)->sc_table, rkFDPrpPyramid(s->fdprp), 0.0 );
}

void rkFDSolverColChk_Volume(rkFDSolver *s, bool doUpRef){
  zEchoOff();
  /* rkCDColVolBREP( rkFDCDBase(rkFDSolverCD(s)) ); */
  rkCDColVolBREPVert( rkFDCDBase(rkFDSolverCD(s)) );
  zEchoOn();
}

bool rkFDSolverUpdate_Volume(rkFDSolver *s, bool doUpRef)
{
  rkFDJointFriction( rkFDSolverChains(s), rkFDPrpDT(s->fdprp), rkFDPrpFricWeight(s->fdprp), doUpRef );
  if( rkFDCDElastNum(s->cd) != 0 )
    rkFDSolverPenalty( s, doUpRef );
  if( rkFDCDRigidNum(s->cd) != 0 )
    if( !_rkFDSolverVolume( s, doUpRef ) ) return false;
  return true;
}

void rkFDSolverUpdateRefWithAcc_Volume(rkFDSolver *s){
  rkFDUpdateJointRefDrivingTrq( rkFDSolverChains(s) );
}

void rkFDSolverUpdateDestroy_Volume(rkFDSolver *s){
  zMatFreeAO( 3, _prp(s)->a, _prp(s)->q, _prp(s)->nf );
  zVecFreeAO( 7, _prp(s)->b, _prp(s)->t, _prp(s)->c, _prp(s)->d, _prp(s)->f, _prp(s)->sc_table[0], _prp(s)->sc_table[1] );
  zFree( _prp(s)->w[0] );
  zFree( _prp(s)->w[1] );
  zIndexFree( _prp(s)->idx );

  zMatFree( _prp(s)->ma );
  zVecFreeAO( 3, _prp(s)->mb, _prp(s)->mc, _prp(s)->mf );
}

RKFD_SOLVER_DEFAULT_GENERATOR( Volume )

#undef _zMat6DElem
#undef _prp

