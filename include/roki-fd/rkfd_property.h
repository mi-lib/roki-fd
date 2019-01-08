/* RoKiFD - Robot Forward Dynamics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_property - properties for forward dynamics simulation
 * contributer: 2014-2018 Naoki Wakisaka
 */

#ifndef __RKFD_PROPERTY_H__
#define __RKFD_PROPERTY_H__

#include <roki-fd/rkfd_defs.h>

__BEGIN_DECLS

typedef struct{
  double dt;
  int pyramid;
  double fric_weight;

  int max_iter;
  double vel_eps;
} rkFDPrp;

#define rkFDPrpDT(p)         (p)->dt
#define rkFDPrpPyramid(p)    (p)->pyramid
#define rkFDPrpFricWeight(p) (p)->fric_weight
#define rkFDPrpMaxIter(p)    (p)->max_iter
#define rkFDPrpVelEps(p)     (p)->vel_eps

#define rkFDPrpSetDT(f,t)         ( (f)->prp.dt = (t) )
#define rkFDPrpSetPyramid(f,n)    ( (f)->prp.pyramid = (n) )
#define rkFDPrpSetFricWeight(f,w) ( (f)->prp.fric_weight = (w) )
#define rkFDPrpSetMaxIter(f,i)    ( (f)->prp.max_iter = (i) )
#define rkFDPrpSetVelEps(f,e)     ( (f)->prp.vel_eps = (e) )

__EXPORT bool rkFDPrpInit(rkFDPrp *prp);
__EXPORT void rkFDPrpDestroy(rkFDPrp *prp);

__END_DECLS

#endif /* __RKFD_PROPERTY_H__ */
