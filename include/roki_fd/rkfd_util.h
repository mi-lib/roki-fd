/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_util - utility functions for forward dynamics computation
 * contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_UTIL_H__
#define __RKFD_UTIL_H__

/* NOTE: never include this header file in user programs exept solver classes. */

#include <roki_fd/rkfd_defs.h>
#include <roki_fd/rkfd_property.h>
#include <roki_fd/rkfd_chain.h>
#include <roki/rk_cd.h>

__BEGIN_DECLS

/******************************************************************************/
/* relative velocity */
__ROKI_FD_EXPORT zVec3D *rkFDLinkPointWldVel(rkLink *link, zVec3D *p, zVec3D *v);
__ROKI_FD_EXPORT void rkFDLinkAddSlideVel(rkCDCell *cell, zVec3D *p, zVec3D *n, zVec3D *v);
__ROKI_FD_EXPORT zVec3D *rkFDChainPointRelativeVel(rkCDPairDat *pd, zVec3D *p, zVec3D *n, rkCDCell *cell, zVec3D *v);
__ROKI_FD_EXPORT zVec6D *rkFDLinkPointWldVel6D(rkLink *link, zVec3D *p, zVec6D *v);
__ROKI_FD_EXPORT zVec6D *rkFDChainPointRelativeVel6D(rkCDPairDat *pd, zVec3D *p, zVec3D *n, zVec6D *v);

/* relative acceleration */
__ROKI_FD_EXPORT zVec3D *rkFDLinkPointWldAcc(rkLink *link, zVec3D *p, zVec3D *a);
__ROKI_FD_EXPORT zVec3D *rkFDChainPointRelativeAcc(rkCDPairDat *pd, zVec3D *p, rkCDCell *cell, zVec3D *a);
__ROKI_FD_EXPORT zVec6D *rkFDLinkPointWldAcc6D(rkLink *link, zVec3D *p, zVec6D *a);
__ROKI_FD_EXPORT zVec6D *rkFDChainPointRelativeAcc6D(rkCDPairDat *pd, zVec3D *p, zVec6D *a);

/******************************************************************************/
/* update and set ABIPrp */
__ROKI_FD_EXPORT void rkFDUpdateAccBias(rkFDChainArray *chains);
__ROKI_FD_EXPORT void rkFDChainUpdateCachedABIPair(rkCDPairDat *pd, rkWrench *w[]);
__ROKI_FD_EXPORT void rkFDChainRestoreABIAccBiasPair(rkCDPairDat *pd);

/******************************************************************************/
__ROKI_FD_EXPORT void rkFDChainExtWrenchDestroy(rkChain *chain);
__ROKI_FD_EXPORT double rkFDKineticFrictionWeight(double w, double fs);
__ROKI_FD_EXPORT bool rkFDCrateSinCosTable(zVec table[2], int num, double offset);

/******************************************************************************/
__ROKI_FD_EXPORT void rkFDUpdateRefSlide(rkCDPairDat *pd, rkCDVert *cdv, double dt);
__ROKI_FD_EXPORT void rkFDContactForceModifyFriction(rkFDPrp *prp, rkCDPairDat *pd, rkCDVert *cdv, zVec3D v, bool doUpRef);
__ROKI_FD_EXPORT void rkFDContactForcePushWrench(rkCDPairDat *pd, rkCDVert *cdv);

/******************************************************************************/
__ROKI_FD_EXPORT void rkFDUpdateJointPrevDrivingTrq(rkFDChainArray *chains);
__ROKI_FD_EXPORT void rkFDJointFrictionAll(rkJoint *joint, double weight);
__ROKI_FD_EXPORT void rkFDJointFrictionRevolDC(rkJoint *joint, double dt, bool doUpRef);
__ROKI_FD_EXPORT void rkFDJointFriction(rkFDChainArray *chains, double dt, double weight, bool doUpRef);

__END_DECLS

#endif /* __RKFD_UTIL_H__ */
