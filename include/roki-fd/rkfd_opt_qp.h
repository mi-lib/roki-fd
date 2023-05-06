/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_opt_qp - optimization tools: quadratic programming.
 * contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_OPT_QP_H__
#define __RKFD_OPT_QP_H__

#include <zm/zm_mat.h>
#include <zeda/zeda_index.h>

__BEGIN_DECLS

__EXPORT bool rkFDQPSolveASM(zMat q, zVec c, zMat a, zVec b, zVec ans, zIndex idx, zVec init(zMat,zVec,zVec,void*), double cond(zMat,zVec,int,void*), void *util);

__END_DECLS

#endif /* __RKFD_OPT_QP_H__ */