/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_cd - collision detection class
 * contributer: 2014- Naoki Wakisaka
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

bool rkFDCDUpdateInit(rkFDCD *cd)
{
  if( zListSize( &cd->cd.plist ) == 0 ) return true;

  zArrayAlloc( &cd->elast_pairs, rkCDPairDat*, zListSize( &cd->cd.plist ) );
  zArrayAlloc( &cd->rigid_pairs, rkCDPairDat*, zListSize( &cd->cd.plist ) );
  if( zArraySize( &cd->elast_pairs ) == 0 || zArraySize( &cd->rigid_pairs ) == 0 )
    return false;
  return true;
}

void rkFDCDUpdate(rkFDCD *cd)
{
  rkCDPair *cdp;

  zArraySize(&cd->elast_pairs) = 0;
  zArraySize(&cd->rigid_pairs) = 0;
  zListForEach( &cd->cd.plist, cdp )
    if( cdp->data.is_col ){
      if( rkContactInfoType(cdp->data.ci) == RK_CONTACT_ELASTIC ){
        *zArrayElemNC(&cd->elast_pairs,zArraySize(&cd->elast_pairs)) = &cdp->data;
        zArraySize(&cd->elast_pairs)++;
      } else if( rkContactInfoType(cdp->data.ci) == RK_CONTACT_RIGID ){
        *zArrayElemNC(&cd->rigid_pairs,zArraySize(&cd->rigid_pairs)) = &cdp->data;
        zArraySize(&cd->rigid_pairs)++;
      }
    }
}

void rkFDCDUpdateDestroy(rkFDCD *cd)
{
  zArrayFree( &cd->elast_pairs );
  zArrayFree( &cd->rigid_pairs );
}
