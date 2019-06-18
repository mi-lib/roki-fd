#include <roki-fd/rkfd_util.h>
#include <roki-fd/rkfd_array.h>
#include <roki/rk_abi.h>

/******************************************************************************/
/* relative velocity */
zVec3D *rkFDLinkPointWldVel(rkLink *link, zVec3D *p, zVec3D *v)
{
  zVec6D v6;
  zVec3D tempv;

  zMulMat3DVec6D( rkLinkWldAtt(link), rkLinkVel(link), &v6 );
  zVec3DSub( p, rkLinkWldPos(link) ,&tempv );
  zVec3DOuterProd( zVec6DAng(&v6), &tempv, &tempv );
  zVec3DAdd( zVec6DLin(&v6), &tempv, v );
  return v;
}

void rkFDLinkAddSlideVel(rkCDCell *cell, zVec3D *p, zVec3D *n, zVec3D *v)
{
  zVec3D sv, tmpv;

  zVec3DSub( p, rkLinkWldPos(cell->data.link), &tmpv );
  zMulMat3DVec3D( rkLinkWldAtt(cell->data.link), &cell->data.slide_axis, &sv );
  zVec3DOuterProd( &sv, &tmpv, &sv );
  zVec3DCatDRC( &sv, -zVec3DInnerProd( &sv, n ), n );
  if( zIsTiny( zVec3DNorm( &sv ) ) ){
    zVec3DZero( &sv );
  }else{
    zVec3DMulDRC( &sv, cell->data.slide_vel/zVec3DNorm( &sv ) );
  }
  zVec3DAddDRC( v, &sv );
}

zVec3D *rkFDChainPointRelativeVel(rkCDPairDat *pd, zVec3D *p, zVec3D *n, rkCDCell *cell, zVec3D *v)
{
  zVec3D vv[2];
  register int i;

  for( i=0; i<2; i++ ){
    if( pd->cell[i]->data.type == RK_CD_CELL_STAT )
      zVec3DZero( &vv[i] );
    else
      rkFDLinkPointWldVel( pd->cell[i]->data.link, p, &vv[i] );
    if( pd->cell[i]->data.slide_mode )
      rkFDLinkAddSlideVel( pd->cell[i], p, n, &vv[i] );
  }
  if( pd->cell[0] == cell )
    zVec3DSub( &vv[0], &vv[1], v );
  else
    zVec3DSub( &vv[1], &vv[0], v );
  return v;
}

zVec6D *rkFDLinkPointWldVel6D(rkLink *link, zVec3D *p, zVec6D *v)
{
  zVec3D tempv;

  zMulMat3DVec6D( rkLinkWldAtt(link), rkLinkVel(link), v );
  zVec3DSub( p, rkLinkWldPos(link) ,&tempv );
  zVec3DOuterProd( zVec6DAng(v), &tempv, &tempv );
  zVec3DAddDRC( zVec6DLin(v), &tempv );
  return v;
}

zVec6D *rkFDChainPointRelativeVel6D(rkCDPairDat *pd, zVec3D *p, zVec3D *n, zVec6D *v)
{
  zVec6D vv[2];
  register int i;

  for( i=0; i<2; i++ ){
    if( pd->cell[i]->data.type == RK_CD_CELL_STAT )
      zVec6DZero( &vv[i] );
    else
      rkFDLinkPointWldVel6D( pd->cell[i]->data.link, p, &vv[i] );
    /* NOTE: this version of the volume-based method does not support slide-mode */
    /* if( pd->cell[i]->data.slide_mode ) */
    /*  rkFDLinkAddSlideVel( pd->cell[i], p, n, &vv[i] ); */
  }
  zVec6DSub( &vv[0], &vv[1], v );
  return v;
}

/* relative acceleration */
zVec3D *rkFDLinkPointWldAcc(rkLink *link, zVec3D *p, zVec3D *a)
{
  zVec3D vp;

  zVec3DSub( p, rkLinkWldPos(link), &vp );
  zMulMat3DTVec3DDRC( rkLinkWldAtt(link), &vp );
  rkLinkPointAcc( link, &vp, a );
  zMulMat3DVec3DDRC( rkLinkWldAtt(link), a );
  return a;
}

zVec3D *rkFDChainPointRelativeAcc(rkCDPairDat *pd, zVec3D *p, rkCDCell *cell, zVec3D *a)
{
  zVec3D av[2];
  register int i;

  for( i=0; i<2; i++ )
    if( pd->cell[i]->data.type == RK_CD_CELL_STAT )
      zVec3DZero( &av[i] );
    else
      rkFDLinkPointWldAcc( pd->cell[i]->data.link, p, &av[i] );
  if( pd->cell[0] == cell )
    zVec3DSub( &av[0], &av[1], a );
  else
    zVec3DSub( &av[1], &av[0], a );
  return a;
}

zVec6D *rkFDLinkPointWldAcc6D(rkLink *link, zVec3D *p, zVec6D *a)
{
  zVec3D vp;

  zVec3DSub( p, rkLinkWldPos(link), &vp );
  zMulMat3DTVec3DDRC( rkLinkWldAtt(link), &vp );
  rkLinkPointAcc( link, &vp, zVec6DLin(a) );
  zMulMat3DVec3DDRC( rkLinkWldAtt(link), zVec6DLin(a) );
  zMulMat3DVec3D( rkLinkWldAtt(link), rkLinkAngAcc(link), zVec6DAng(a) );
  return a;
}

zVec6D *rkFDChainPointRelativeAcc6D(rkCDPairDat *pd, zVec3D *p, zVec6D *a)
{
  zVec6D av[2];
  register int i;

  for( i=0; i<2; i++ )
    if( pd->cell[i]->data.type == RK_CD_CELL_STAT )
      zVec6DZero( &av[i] );
    else{
      rkFDLinkPointWldAcc6D( pd->cell[i]->data.link, p, &av[i] );
    }
  zVec6DSub( &av[0], &av[1], a );
  return a;
}

/******************************************************************************/
/* update and set ABIPrp */
void rkFDUpdateAccBias(rkFDChainArray *chains)
{
  rkFDChain **c;

  rkFDArrayForEach( chains, c ){
    if( (*c)->has_rigid_col ){
      (*c)->done_abi_init = true;
      rkChainABIUpdate( &(*c)->chain );
      rkChainABIPushPrpAccBias( &(*c)->chain );
      rkFDChainExtWrenchDestroy( &(*c)->chain );
    }
  }
}

void rkFDChainUpdateAccAddExForceTwo(rkCDPairDat *pd, rkWrench *w[])
{
  if( pd->cell[0]->data.chain == pd->cell[1]->data.chain ) /* self collision */
    rkChainABIUpdateAddExForceTwo( pd->cell[0]->data.chain, pd->cell[0]->data.link, w[0], pd->cell[1]->data.link, w[1] );
  else {
    rkChainABIUpdateAddExForceTwo( pd->cell[0]->data.chain, pd->cell[0]->data.link, w[0], NULL, NULL );
    rkChainABIUpdateAddExForceTwo( pd->cell[1]->data.chain, pd->cell[1]->data.link, w[1], NULL, NULL );
  }
}

void rkFDChainABIPopPrpExForceTwo(rkCDPairDat *pd)
{
  if( pd->cell[0]->data.chain == pd->cell[1]->data.chain )
    rkChainABIPopPrpAccBiasAddExForceTwo( pd->cell[0]->data.chain, pd->cell[0]->data.link, pd->cell[1]->data.link );
  else {
    rkChainABIPopPrpAccBiasAddExForceTwo( pd->cell[0]->data.chain, pd->cell[0]->data.link, NULL );
    rkChainABIPopPrpAccBiasAddExForceTwo( pd->cell[1]->data.chain, pd->cell[1]->data.link, NULL );
  }
}

/******************************************************************************/
/* destroy temporary wrench list */
void rkFDChainExtWrenchDestroy(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
    rkWrenchListDestroy( rkLinkExtWrenchBuf(rkChainLink(chain,i)) );
}

double rkFDKineticFrictionWeight(double w, double fs){
  return 1.0 - exp( -1.0 * w * fs );
}

/* sin cos table */
bool rkFDCrateSinCosTable(zVec table[2], int num, double offset){
	register int i;
	double th, dth;

	table[0] = zVecAlloc( num );
	table[1] = zVecAlloc( num );
  if( table[0] == NULL || table[1] == NULL ){
    zVecFreeAO( 2, table[0], table[1] );
    return false;
  }
	dth = zDeg2Rad( 360 ) / num;
	for( i=0,th=0.0; i<num; i++,th+=dth )
		zSinCos( th+offset, &zVecElemNC(table[0],i), &zVecElemNC(table[1],i) );
  return true;
}

/******************************************************************************/
/* contact force */
void rkFDUpdateRefSlide(rkCDPairDat *pd, rkCDVert *cdv, double dt)
{
  register int i;
  zVec3D tmpv, sv;

  /* for slide mode */
  for( i=0; i<2; i++ ){
    if( pd->cell[i]->data.slide_mode ){
      zVec3DSub( cdv->data.vert, rkLinkWldPos(pd->cell[i]->data.link), &tmpv );
      zMulMat3DVec3D( rkLinkWldAtt(pd->cell[i]->data.link), &pd->cell[i]->data.slide_axis, &sv );
      zVec3DOuterProd( &sv, &tmpv, &sv );
      zVec3DCatDRC( &sv, -zVec3DInnerProd( &sv, &cdv->data.norm ), &cdv->data.norm );
      if( !zIsTiny( zVec3DNorm( &sv ) ) ){
        zVec3DMulDRC( &sv, (pd->cell[i]==cdv->data.cell?-1.0:1.0) * dt * pd->cell[i]->data.slide_vel/zVec3DNorm( &sv ) );
        zMulMat3DTVec3DDRC( rkLinkWldAtt(pd->cell[(pd->cell[i]==cdv->data.cell?1:0)]->data.link), &sv );
        zVec3DAddDRC( &cdv->data._ref, &sv );
      }
    }
  }
}

void rkFDContactForceModifyFriction(rkFDPrp *prp, rkCDPairDat *pd, rkCDVert *cdv, zVec3D v, bool doUpRef)
{
  double fn, fs, vs;

  fn = zVec3DInnerProd( &cdv->data.f, &cdv->data.axis[0] );
  fs = sqrt( zSqr( zVec3DInnerProd( &cdv->data.f, &cdv->data.axis[1] ) ) + zSqr( zVec3DInnerProd( &cdv->data.f, &cdv->data.axis[2] ) ) );
  if( !zIsTiny( fs ) &&
      fs > ( cdv->data.type == RK_CONTACT_SF ? rkContactInfoSF(pd->ci) : rkContactInfoKF(pd->ci) )*fn ){
    /* kinetic friction */
    zVec3DCatDRC( &v, -zVec3DInnerProd( &v, &cdv->data.axis[0] ), &cdv->data.axis[0] );
    vs = zVec3DNorm( &v );
    zVec3DMul( &cdv->data.axis[0], fn, &cdv->data.f );
    if( !zIsTiny( vs ) ){
      zVec3DNormalizeNCDRC( &v );
      zVec3DCatDRC( &cdv->data.f, - rkFDKineticFrictionWeight( rkFDPrpFricWeight(prp), vs ) * rkContactInfoKF(pd->ci) * fn, &v );
    }
    if( doUpRef ){
      cdv->data.type = RK_CONTACT_KF;
      zVec3DCopy( &cdv->data._pro, &cdv->data._ref );
    }
  } else {
    /* static friction */
    if( doUpRef ){
      cdv->data.type = RK_CONTACT_SF;
      rkFDUpdateRefSlide( pd, cdv, rkFDPrpDT(prp) );
    }
  }
}

void rkFDContactForcePushWrench(rkCDPairDat *pd, rkCDVert *cdv)
{
  rkWrench *w;
  register int i;

  for( i=0; i<2; i++){
    w = zAlloc( rkWrench, 1 );
    rkWrenchInit( w );
    zXform3DInv( rkLinkWldFrame(pd->cell[i]->data.link), cdv->data.vert, rkWrenchPos(w) );
    zMulMat3DTVec3D( rkLinkWldAtt(pd->cell[i]->data.link), &cdv->data.f, rkWrenchForce(w) );
    if( pd->cell[i] != cdv->data.cell )
      zVec3DRevDRC( rkWrenchForce(w) );
    rkWrenchListPush( rkLinkExtWrenchBuf(pd->cell[i]->data.link), w );
  }
}

/******************************************************************************/
/* computation of the joint references */
/* NOTE:
 * this function must be called after the computation of joint accelerations
 */
void rkFDUpdateJointRefDrivingTrq(rkFDChainArray *chains)
{
  rkFDChain **fdc;
  rkChain *chain;
  rkJoint *joint;
  register int i, j;
  double val[6], tf[6];
  rkJointRef jref[6];

  rkFDArrayForEach( chains, fdc ){
    chain = rkFDChainBase( *fdc );
    for( i=0; i<rkChainNum(chain); i++ ){
      joint = rkChainLinkJoint(chain,i);
      rkJointGetRef( joint, jref );
      rkJointGetFric( joint, tf );
      rkJointMotorDrivingTrq( joint, val );
      for( j=0; j<rkJointSize(joint); j++ ){
        jref[j].ref_trq = val[j] + tf[j];
      }
      rkJointSetRef( joint, jref );
    }
  }
}

/* default joint friction computation*/
/* NOTE:
 * before this function is called,
 * the reference value at the one step before must be set
 */
void rkFDJointFrictionAll(rkJoint *joint, double weight)
{
  double kf[6], v[6];
  register int i;

  rkJointGetKFric( joint, kf );
  rkJointGetVel( joint, v );
  for( i=0; i<rkJointSize(joint); i++ )
    kf[i] *= rkFDKineticFrictionWeight( weight, fabs(v[i]) );
  rkJointSetFric( joint, kf );
}

void rkFDJointFrictionRevolDC(rkJoint *joint, double dt, bool doUpRef)
{
  double v, val, fmax, tf;
  rkJointRef jref;

  rkJointMotorInertia( joint, &tf );
  rkJointGetVel( joint, &v );
  tf *= -v / dt;
  rkJointMotorInputTrq( joint, &val );
  tf -= val;
  rkJointMotorRegistance( joint, &val );
  tf += val;
  rkJointGetRef( joint, &jref );
  tf += jref.ref_trq;

  if( jref.type == RK_CONTACT_SF )
    rkJointGetSFric( joint, &fmax );
  else
    rkJointGetKFric( joint, &fmax );

  fmax = fabs(fmax);
  if( fabs( tf ) > fmax ){
    tf = tf > 0 ? fmax: -fmax;
    if( doUpRef ){
      jref.type = RK_CONTACT_KF;
      rkJointSetRef( joint, &jref );
    }
  } else{
    if( doUpRef ){
      jref.type = RK_CONTACT_SF;
      rkJointSetRef( joint, &jref );
    }
  }
  rkJointSetFric( joint, &tf );
}

void rkFDJointFriction(rkFDChainArray *chains, double dt, double weight, bool doUpRef)
{
  rkFDChain **fdc;
  rkChain *chain;
  rkJoint *joint;
  register int i;

  rkFDArrayForEach( chains, fdc ){
    chain = rkFDChainBase( *fdc );
    if( rkChainJointSize( chain ) == 0 ) continue;
    for( i=0; i<rkChainNum(chain); i++ ){
      joint = rkChainLinkJoint(chain,i);
      /* 1DoF joint and DC motor only */
      if( rkJointSize(joint) == 1 && rkJointMotorType(joint) == RK_MOTOR_DC )
        rkFDJointFrictionRevolDC( joint, dt, doUpRef );
      else
        rkFDJointFrictionAll( joint, weight );
    }
  }
}

