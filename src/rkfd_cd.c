/* RoKiFD - Robot Forward Dynamics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_cd - collision detection class
 * contributer: 2014-2018 Naoki Wakisaka
 */

#include <roki-fd/rkfd_cd.h>

void rkFDCDInit(rkFDCD *cd)
{
  rkCDCreate( &cd->cd );
  zArrayInit( &cd->elast_pairs );
  zArrayInit( &cd->rigid_pairs );
}

void rkFDCDDestroy(rkFDCD *cd)
{
  rkCDDestroy( &cd->cd );
}

bool rkFDCDUpdateInit(rkFDCD *cd){
  if( zListNum( &cd->cd.plist ) == 0 )
    return true;

  zArrayAlloc( &cd->elast_pairs, rkCDPairDat*, zListNum( &cd->cd.plist ) );
  zArrayAlloc( &cd->rigid_pairs, rkCDPairDat*, zListNum( &cd->cd.plist ) );
  if( zArrayNum( &cd->elast_pairs ) == 0 || zArrayNum( &cd->rigid_pairs ) == 0 )
    return false;
  return true;
}

void rkFDCDUpdate(rkFDCD *cd)
{
  rkCDPair *cdp;

  zArraySetNum( &cd->elast_pairs, 0 );
  zArraySetNum( &cd->rigid_pairs, 0 );
  zListForEach( &cd->cd.plist, cdp )
    if( cdp->data.is_col ){
      if( rkContactInfoType(cdp->data.ci) == RK_CONTACT_ELASTIC ){
        *zArrayElemNC(&cd->elast_pairs,zArrayNum(&cd->elast_pairs)) = &cdp->data;
        zArrayNum(&cd->elast_pairs)++;
      } else if( rkContactInfoType(cdp->data.ci) == RK_CONTACT_RIGID ){
        *zArrayElemNC(&cd->rigid_pairs,zArrayNum(&cd->rigid_pairs)) = &cdp->data;
        zArrayNum(&cd->rigid_pairs)++;
      }
    }
}

void rkFDCDUpdateDestroy(rkFDCD *cd)
{
  zArrayFree( &cd->elast_pairs );
  zArrayFree( &cd->rigid_pairs );
}

