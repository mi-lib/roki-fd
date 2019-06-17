/* RoKiFD - Robot Forward Dynamics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_sim - forward dynamics simulation
 * contributer: 2014-2018 Naoki Wakisaka
 */

#ifndef __RKFD_SIM_H__
#define __RKFD_SIM_H__

#include <roki-fd/rkfd_property.h>
#include <roki-fd/rkfd_chain.h>
#include <roki-fd/rkfd_cd.h>
#include <roki-fd/rkfd_solver.h>

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkFD
   forward dynamics class
 * ********************************************************** */
typedef struct{
  rkFDChain fc;    /* derived by rkChain */
  int _offset;     /* offset */
  zVecStruct _dis; /* displacement */
  zVecStruct _vel; /* velocity */
  zVecStruct _acc; /* acceleration */
} rkFDCellDat;

zListClass( rkFDCellList, rkFDCell, rkFDCellDat );

#define rkFDCellDatChain(d) rkFDChainBase(&(d)->fc)
#define rkFDCellChain(c)    rkFDCellDatChain(&(c)->data)

/* forward dynamics simulator */
typedef struct _rkFD{
  double t;
  rkFDPrp prp;
  rkFDSolver solver;    /* solver */
  rkFDCellList list;    /* chain list */
  rkContactInfoPool ci; /* contact information list */
  rkContactInfo cidef;  /* default contact infomation */
  rkFDCD cd;            /* contact maneger */

  zODE2 ode;
  int ode_step;
  zVec dis, vel;        /* total joint state */
  zVec acc;
  int size;             /* total joint size */
} rkFD;

#define rkFDTime(f)   (f)->t
#define rkFDDT(f)     (f)->prp.dt
#define rkFDGetPrp(f) ( &(f)->prp )

__EXPORT rkFD *rkFDCreate(rkFD *fd);
__EXPORT void rkFDDestroy(rkFD *fd);

__EXPORT rkFDCell *rkFDChainReg(rkFD *fd, rkChain *chain);
__EXPORT rkFDCell *rkFDChainRegFile(rkFD *fd, char filename[]);
__EXPORT bool rkFDChainUnreg(rkFD *fd, rkFDCell *cell);

__EXPORT void rkFDFK(rkFD *fd, zVec dis);
__EXPORT void rkFDUpdateRate(rkFD *fd, zVec vel, zVec acc);
__EXPORT void rkFDUpdateFKRate(rkFD *fd);

__EXPORT void rkFDChainSetDis(rkFDCell *lc, zVec dis);
__EXPORT void rkFDChainSetVel(rkFDCell *lc, zVec vel);
__EXPORT bool rkFDContactInfoScanFile(rkFD *fd, char filename[]);

/* for a fake-crawler */
__EXPORT void rkFDCDCellSetSlideMode(rkCDCell *cell, bool mode);
__EXPORT void rkFDCDCellSetSlideVel(rkCDCell *cell, double vel);
__EXPORT void rkFDCDCellSetSlideAxis(rkCDCell *cell, zVec3D *axis);
__EXPORT rkCDCell *rkFDShape3DGetCDCell(rkFD *fd, zShape3D *shape);
__EXPORT rkCDCell *rkFDShape3DSetSlideMode(rkFD *fd, zShape3D *shape, bool mode);
__EXPORT rkCDCell *rkFDShape3DSetSlideVel(rkFD *fd, zShape3D *shape, double vel);
__EXPORT rkCDCell *rkFDShape3DSetSlideAxis(rkFD *fd, zShape3D *shape, zVec3D *axis);

/* ODE updater */
__EXPORT zVec rkFDODECatDefault(zVec x, double k, zVec v, zVec xnew, void *util);
__EXPORT zVec rkFDODESubDefault(zVec x1, zVec x2, zVec dx, void *util);
#define rkFDODE2Assign(f,t)        zODE2Assign( &(f)->ode, t, rkFDODECatDefault, NULL, rkFDODESubDefault, NULL )
#define rkFDODE2AssignRegular(f,t) zODE2AssignRegular( &(f)->ode, t )

/* solver */
#define rkFDSetSolver(f,type) do{                                 \
    rkFDSolverReset( &(f)->solver );                              \
    rkFDSolverCreate_##type( &(f)->solver );                      \
    rkFDSolverGetDefaultContactInfo( &(f)->solver, &(f)->cidef ); \
  } while(0)

__EXPORT rkFD *rkFDSolve(rkFD *fd);
__EXPORT void rkFDUpdateInit(rkFD *fd);
__EXPORT rkFD *rkFDUpdate(rkFD *fd);
__EXPORT void rkFDUpdateDestroy(rkFD *fd);
#define rkFDSolveContact(f,b) (f)->_solve_contact(f,b)

/* for debug */
__EXPORT void rkFDPrint(rkFD *fd);

__END_DECLS

#endif /* __RKFD_SIM_H__ */
