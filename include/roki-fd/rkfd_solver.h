/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_solver - contact force computation solver
 * contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_SOLVER_H__
#define __RKFD_SOLVER_H__

#include <roki-fd/rkfd_property.h>
#include <roki-fd/rkfd_chain.h>
#include <roki-fd/rkfd_cd.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkFDSolver
   contact force computation solver abstract class
 * ********************************************************** */
/* abstruct solver function */
struct _rkFDSolver;
typedef struct{
  void (*_defci)(struct _rkFDSolver*,rkContactInfo*);
  bool (*_init)(struct _rkFDSolver*);
  void (*_colchk)(struct _rkFDSolver*,bool);
  bool (*_update)(struct _rkFDSolver*,bool);
  void (*_update_ref)(struct _rkFDSolver*);
  void (*_destroy)(struct _rkFDSolver*);
} rkFDSolverCom;

typedef struct _rkFDSolver{
  void *prp;
  rkFDSolverCom *com;

  /* created by rkFD */
  double t;
  rkFDPrp *fdprp;
  rkFDCD *cd;
  rkFDChainArray chains;
} rkFDSolver;

#define rkFDSolverT(s)       (s)->t
#define rkFDSolverDT(s)      rkFDPrpDT((s)->fdprp)
#define rkFDSolverFDPrp(s)   (s)->fdprp
#define rkFDSolverCD(s)      (s)->cd
#define rkFDSolverChains(s)  &(s)->chains
#define rkFDSolverChain(s,i) ( *zArrayElem(&(s)->chains,i) )
#define rkFDSolverIsEmpty(s) ( (s)->prp == NULL && (s)->com == NULL )

__EXPORT void rkFDSolverInit(rkFDSolver *solver);
__EXPORT void rkFDSolverReset(rkFDSolver *solver);
__EXPORT void rkFDSolverDestroy(rkFDSolver *solver);

#define rkFDSolverGetDefaultContactInfo(s,c) (s)->com->_defci(s,c)
#define rkFDSolverUpdateInit(s)              (s)->com->_init(s)
#define rkFDSolverColChk(s,b)                (s)->com->_colchk(s,b)
#define rkFDSolverUpdate(s,b)                (s)->com->_update(s,b)
#define rkFDSolverUpdatePrevDrivingTrq(s)    (s)->com->_update_ref(s)
#define rkFDSolverUpdateDestroy(s)           (s)->com->_destroy(s)

/* **********************************************************
 * solver function generator
 *
 * Vert solver example:
 * in rkfd_vert.h:
 * typedef struct{
 *   ...
 * } rkFDSolverPrpVert;
 *
 * RKFD_SOLVER_FUNCTION_DEFAULT( Vert )
 * RKFD_SOLVER_CREATE_DEFAULT( Vert )
 *
 *
 * in rkfd_vert.c:
 * void rkFDSolverGetDefaultContactInfo_Vert(rkFDSolver *s, rkContactInfo *ci){...}
 * bool rkFDSolverUpdateInit_Vert(rkFDSolver *s){...}
 * void rkFDSolverColChk_Vert(rkFDSolver *s, bool doUpRef){...}
 * bool rkFDSolverUpdate_Vert(rkFDSolver *s, bool doUpRef){...}
 * void rkFDSolverUpdatePrevDrivingTrq_Vert(rkFDSolver *s){...}
 * void rkFDSolverUpdateDestroy_Vert(rkFDSolver *s){...}
 *
 * RKFD_SOLVER_DEFAULT_GENERATOR( Vert )
 * ********************************************************** */
#define RKFD_SOLVER_FUNCTION_DEFAULT(type)                              \
  void rkFDSolverGetDefaultContactInfo_##type(rkFDSolver *s, rkContactInfo *ci); \
  bool rkFDSolverUpdateInit_##type(rkFDSolver *s);                      \
  void rkFDSolverColChk_##type(rkFDSolver *s, bool doUpRef);            \
  bool rkFDSolverUpdate_##type(rkFDSolver *s, bool doUpRef);            \
  void rkFDSolverUpdatePrevDrivingTrq_##type(rkFDSolver *s);            \
  void rkFDSolverUpdateDestroy_##type(rkFDSolver *s);

#define RKFD_SOLVER_FUNCTION_DEFAULT_INST(type) \
  static rkFDSolverCom rkfd_solver_##type = {   \
    rkFDSolverGetDefaultContactInfo_##type,     \
    rkFDSolverUpdateInit_##type,                \
    rkFDSolverColChk_##type,                    \
    rkFDSolverUpdate_##type,                    \
    rkFDSolverUpdatePrevDrivingTrq_##type,      \
    rkFDSolverUpdateDestroy_##type              \
  };

#define RKFD_SOLVER_CREATE_DEFAULT(type) \
  rkFDSolver *rkFDSolverCreate_##type(rkFDSolver *s);

#define RKFD_SOLVER_CREATE_DEFAULT_IMPL(type)            \
  rkFDSolver *rkFDSolverCreate_##type(rkFDSolver *s)     \
  {                                                      \
    if( !( s->prp = zAlloc( rkFDSolverPrp##type, 1 ) ) ) \
      return NULL;                                       \
    s->com = &rkfd_solver_##type;                        \
    return s;                                            \
  }

#define RKFD_SOLVER_DEFAULT_GENERATOR(type) \
  RKFD_SOLVER_FUNCTION_DEFAULT_INST(type)   \
  RKFD_SOLVER_CREATE_DEFAULT_IMPL(type)

__END_DECLS

#endif /* __RKFD_SOLVER_H__ */
