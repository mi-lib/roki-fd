/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_volume - volumetric contact model
 * contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_VOLUME_H__
#define __RKFD_VOLUME_H__

/* NOTE: never include this header file in user programs. */

#include <roki-fd/rkfd_solver.h>
#include <roki/rk_force.h>

__BEGIN_DECLS

typedef struct{
  int colnum;
  zMat a;
  zVec b, t;

  int fsize, csize;
  zMat q, nf;
  zVec c, d, f;
  zIndex idx;

  zVec sc_table[2];
  rkWrench *w[2];

  int pvsize;
  zMat ma;
  zVec mb, mc, mf;
} rkFDSolverPrpVolume;

RKFD_SOLVER_FUNCTION_DEFAULT( Volume )
RKFD_SOLVER_CREATE_DEFAULT( Volume )

__END_DECLS

#endif /* __RKFD_VOLUME_H__ */
