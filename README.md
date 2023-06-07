RoKi-FD - Robot Simulation library  
Copyright (C) 2018 Tomomichi Sugihara (Zhidao)

| *master* |
|----------|
| ![Build Status](https://github.com/mi-lib/roki-fd/actions/workflows/build.yml/badge.svg) |

# What is this?

RoKi-FD is a software library for robot forward dynamics simulation including:
* contact force computation
* joint friction computation

ZEDA, ZM, Zeo and RoKi are required.

# Installation / Uninstallation

### install
0. Install [ZEDA](https://github.com/zhidao/zeda), [ZM](https://github.com/zhidao/zm), [Zeo](https://github.com/zhidao/zeo) and [RoKi](https://github.com/zhidao/roki) in advance.

1. Clone this repository.

  `git clone https://github.com/n-wakisaka/roki-fd`

2. Enter the directory.

  `cd roki-fd`

3. Edit config file if necessary.

  * PREFIX
    
    directory where the library is installed.
    ~/usr as a default. In this case, header files and library are installed under ~/usr/include and ~/usr/lib, respectively.

4. Make it.

  `make`

5. Install it.

  `make install`

  Or,

  `cp -a lib/libroki-fd.so $PREFIX/lib/`  
  `cp -a include/roki $PREFIX/include/`  
  `cp -a bin/* $PREFIX/bin/`

### uninstall
1.Uninstall it

  `make uninstall`
  
  Or, 
  
  delete $PREFIX/lib/libroki-fd.so, $PREFIX/include/roki-fd/roki-fd.h and $PREFIX/include/roki-fd/rkfd*.h.

# How to use

You may need to set your PATH and LD_LIBRARY_PATH environment variables.  
This is done by:  
`export PATH=$PATH:$PREFIX/bin`  
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PREFIX/lib`
if your working shell is Bourne shell (bash, zsh, etc.),

or by:  
`set path = ( $path $PREFIX/bin )`  
`setenv LD_LIBRARY_PATH $LD_LIBRARY_PATH:$PREFIX/lib`  
if your working shell is C shell (csh, tcsh, etc.).

When you want to compile your code test.c, for example, the following line will work.

```gcc `roki-fd-config -L` `roki-fd-config -I` test.c `roki-fd-config -l` ```

# Contact

zhidao@ieee.org

### Contributor

Following people contributed to this library:
* Naoki Wakisaka (naoki.wakisaka@ams.eng.osaka-u.ac.jp)
