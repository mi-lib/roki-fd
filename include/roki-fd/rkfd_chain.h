/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_chain - derived chain class
 * additional contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_CHAIN_H__
#define __RKFD_CHAIN_H__

#include <roki/rk_chain.h>

/* derived chain class */
typedef struct{
  rkChain chain;
  bool has_rigid_col;
  bool done_abi_init;
} rkFDChain;

#define rkFDChainBase(c)    ( (rkChain*)(c) )
#define rkFDChainDerived(c) ( (rkFDChain*)(c) )

zArrayClass( rkFDChainArray, rkFDChain* );
#define rkFDChainArrayNum(c)    zArrayNum(c)
#define rkFDChainArrayElem(c,i) *zArrayElem(c,i)

#define rkFDChainIsCol(c) (c)->iscol
#define rkFDChainReset(c) do{   \
    (c)->has_rigid_col = false; \
    (c)->done_abi_init = false; \
  } while(0)

#endif /* __RKFD_CHAIN_H__ */
