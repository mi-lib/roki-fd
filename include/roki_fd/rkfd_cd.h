/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_cd - collision detection class
 * contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_CD_H__
#define __RKFD_CD_H__

#include <roki/rk_cd.h>
#include <roki_fd/rkfd_defs.h>
#include <roki_fd/rkfd_array.h>

__BEGIN_DECLS

zArrayClass( rkCDPairArray, rkCDPairDat* );
/* typedef struct{ */
/*   rkCDPairDat* pd; */
/*   void *prp; */
/* } rkFDCDPair; */

typedef struct{
  rkCD cd;
  rkCDPairArray elast_pairs;
  rkCDPairArray rigid_pairs;
} rkFDCD;

#define rkFDCDBase(c) ( (rkCD*)(c) )
#define rkFDCDElastNum(c)    zArraySize(&(c)->elast_pairs)
#define rkFDCDElastPair(c,i) ( *zArrayElem(&(c)->elast_pairs,i) )
#define rkFDCDRigidNum(c)    zArraySize(&(c)->rigid_pairs)
#define rkFDCDRigidPair(c,i) ( *zArrayElem(&(c)->rigid_pairs,i) )

#define rkFDCDForEachElastPair(c,p) rkFDArrayForEach(&(c)->elast_pairs, p)
#define rkFDCDForEachRigidPair(c,p) rkFDArrayForEach(&(c)->rigid_pairs, p)

__ROKI_FD_EXPORT void rkFDCDInit(rkFDCD *cd);
__ROKI_FD_EXPORT void rkFDCDDestroy(rkFDCD *cd);

__ROKI_FD_EXPORT bool rkFDCDUpdateInit(rkFDCD *cd);
__ROKI_FD_EXPORT void rkFDCDUpdate(rkFDCD *cd);
__ROKI_FD_EXPORT void rkFDCDUpdateDestroy(rkFDCD *cd);

__END_DECLS

#endif /* __RKFD_CD_H__ */
