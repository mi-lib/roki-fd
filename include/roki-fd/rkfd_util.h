/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_util - utility functions for forward dynamics computation
 * contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_UTIL_H__
#define __RKFD_UTIL_H__

/* NOTE: never include this header file in user programs exept solver classes. */

#include <roki-fd/rkfd_defs.h>
#include <roki-fd/rkfd_property.h>
#include <roki-fd/rkfd_chain.h>
#include <roki/rk_cd.h>

__BEGIN_DECLS

/******************************************************************************/
/* relative velocity */
__EXPORT zVec3D *rkFDLinkPointWldVel(rkLink *link, zVec3D *p, zVec3D *v);
__EXPORT void rkFDLinkAddSlideVel(rkCDCell *cell, zVec3D *p, zVec3D *n, zVec3D *v);
__EXPORT zVec3D *rkFDChainPointRelativeVel(rkCDPairDat *pd, zVec3D *p, zVec3D *n, rkCDCell *cell, zVec3D *v);
__EXPORT zVec6D *rkFDLinkPointWldVel6D(rkLink *link, zVec3D *p, zVec6D *v);
__EXPORT zVec6D *rkFDChainPointRelativeVel6D(rkCDPairDat *pd, zVec3D *p, zVec3D *n, zVec6D *v);

/* relative acceleration */
__EXPORT zVec3D *rkFDLinkPointWldAcc(rkLink *link, zVec3D *p, zVec3D *a);
__EXPORT zVec3D *rkFDChainPointRelativeAcc(rkCDPairDat *pd, zVec3D *p, rkCDCell *cell, zVec3D *a);
__EXPORT zVec6D *rkFDLinkPointWldAcc6D(rkLink *link, zVec3D *p, zVec6D *a);
__EXPORT zVec6D *rkFDChainPointRelativeAcc6D(rkCDPairDat *pd, zVec3D *p, zVec6D *a);

/******************************************************************************/
/* update and set ABIPrp */
__EXPORT void rkFDUpdateAccBias(rkFDChainArray *chains);
__EXPORT void rkFDChainUpdateAccAddExForceTwo(rkCDPairDat *pd, rkWrench *w[]); /* TODO: Rename */
__EXPORT void rkFDChainABIPopPrpExForceTwo(rkCDPairDat *pd); /* TODO: Rename */

/******************************************************************************/
__EXPORT void rkFDChainExtWrenchDestroy(rkChain *chain);
__EXPORT double rkFDKineticFrictionWeight(double w, double fs);
__EXPORT bool rkFDCrateSinCosTable(zVec table[2], int num, double offset);

/******************************************************************************/
__EXPORT void rkFDUpdateRefSlide(rkCDPairDat *pd, rkCDVert *cdv, double dt);
__EXPORT void rkFDContactForceModifyFriction(rkFDPrp *prp, rkCDPairDat *pd, rkCDVert *cdv, zVec3D v, bool doUpRef);
__EXPORT void rkFDContactForcePushWrench(rkCDPairDat *pd, rkCDVert *cdv);

/******************************************************************************/
__EXPORT void rkFDUpdateJointPrevDrivingTrq(rkFDChainArray *chains);
__EXPORT void rkFDJointFrictionAll(rkJoint *joint, double weight);
__EXPORT void rkFDJointFrictionRevolDC(rkJoint *joint, double dt, bool doUpRef);
__EXPORT void rkFDJointFriction(rkFDChainArray *chains, double dt, double weight, bool doUpRef);


__END_DECLS

#endif /* __RKFD_UTIL_H__ */
