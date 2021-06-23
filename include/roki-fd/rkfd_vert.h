/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_vert - vertex / plane contact model
 * contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_VERT_H__
#define __RKFD_VERT_H__

/* NOTE: never include this header file in user programs. */

#include <roki-fd/rkfd_solver.h>
#include <roki/rk_force.h>

__BEGIN_DECLS

typedef struct{
  int colnum;
  zMat a;
  zVec b, t;

  int qp_size;
  zMat q, nf;
  zVec c, d, f;
  zIndex idx;

  zVec sc_table[2];
  rkWrench *w[2];
} rkFDSolverPrpVert;

RKFD_SOLVER_FUNCTION_DEFAULT( Vert )
RKFD_SOLVER_CREATE_DEFAULT( Vert )

__END_DECLS

#endif /* __RKFD_VERT_H__ */
