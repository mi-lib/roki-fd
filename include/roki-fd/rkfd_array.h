/* RoKiFD - Robot Forward Dynamics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_array -
 * contributer: 2014-2018 Naoki Wakisaka
 */

#ifndef __RKFD_ARRAY_H__
#define __RKFD_ARRAY_H__

#define rkFDArrayForEach(c,p) \
  for( (p)=zArrayBuf(c); (p) - zArrayBuf(c) < zArrayNum(c); (p)++ )

#endif /* __RKFD_ARRAY_H__ */
