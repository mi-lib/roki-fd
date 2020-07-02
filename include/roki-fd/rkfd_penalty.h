/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_penalty - contact force computation based on penalty method
 * additional contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_PENALTY_H__
#define __RKFD_PENALTY_H__

#include <roki-fd/rkfd_solver.h>

__BEGIN_DECLS

__EXPORT void rkFDSolverPenalty(rkFDSolver *solver, bool doUpRef);
__EXPORT void rkFDSolverPenaltyVolumeBased(rkFDSolver *solver, bool doUpRef);

__END_DECLS

#endif /* __RKFD_PENALTY_H__ */
