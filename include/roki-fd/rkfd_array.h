/* RoKi-FD - Robot Kinetics library: forward dynamics extention
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rkfd_array - utility for array manipulation
 * additional contributer: 2014- Naoki Wakisaka
 */

#ifndef __RKFD_ARRAY_H__
#define __RKFD_ARRAY_H__

#define rkFDArrayForEach(c,p) \
  for( (p)=zArrayBuf(c); (p) - zArrayBuf(c) < zArraySize(c); (p)++ )

#endif /* __RKFD_ARRAY_H__ */
