/* RoKiFD - Robot Forward Dynamics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_def - definitions
 * contributer: 2014-2018 Naoki Wakisaka
 */

#ifndef __RKFD_DEFS_H__
#define __RKFD_DEFS_H__

#define RK_FD_DT_DEFAULT           0.001
#define RK_FD_CDT_DEFAULT          0.002
#define RK_FD_JOINT_COMP_K_DEFAULT 100
#define RK_FD_JOINT_COMP_L_DEFAULT 0.01
#define RK_FD_KINETIC_FRIC_WEIGHT_DEFAULT 100
#define RK_FD_FRIC_PYRAMID_ORDER_DEFAULT 8

#define RK_FD_MAX_ITER_DEFAULT 10
#define RK_FD_VEL_EPSILON_DEFAULT 1e-8

#include <zeda/zeda.h>

#endif /* __RKFD_DEFS_H__ */
