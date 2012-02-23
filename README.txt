Stereo Parallel Tracking and Mapping for Small AR Workspaces
-----------------------------------------------------
This is an implementation of PTAM with stereo cameras. The original PTAM
work can be found in the paper `Parallel Tracking and Mapping 
for Small AR Workspaces' by Georg Klein and David Murray, which 
appeared in the proceedings of the IEEE/ACM International Symposium 
on Mixed and Augmented Reality (ISMAR) 2007.

The code has only been tested on linux but was originally designed for
OS X and Windows so may still have some functionality on these platforms

Processor Requirements: 
-----------------------
The software runs at least two processing-intensive threads at the
same time and so needs at least a dual-core machine. Intel Core 2 Duo
processors 2.4GHz+ are fine.

Graphics:
---------
The software requires accelerated OpenGL graphics output. It has been
written and tested only with nVidia cards: this is primarily of
concern under Linux, where the use of an nVidia card and the
proprietary nVidia display drivers are highly recommended. Since the
Linux code compiles directly against the nVidia driver's GL headers,
use of a different GL driver may require some modifications to the
code.

Video Input:
------------
The software requires a video camera with a wide-angle lens, capable
of 640x480x30Hz video capture and an appropriate driver installation
(which is supported by libCVD.) Only monochrome capture is needed for
tracking, colour images are used only for the AR display. A futher
discussion of video input follows later in this file.

Libraries:
----------
The software has three principal dependencies: 

1. TooN - a header library for linear algebra 
2. libCVD - a library for image handling, video capture and computer
vision
3. Gvars3 - a run-time configuration/scripting library, this is a
sub-project of libCVD.

All three above are written by member of the Cambridge Machine
Intelligence lab and are licensed under the LGPL.

Current versions are available from Savannah via CVS:
http://savannah.nongnu.org/projects/toon (for TooN)
http://savannah.nongnu.org/projects/libcvd (for libCVD and GVars)

The latest version of these libraries can be obtained via CVS and ssh:

# export CVS_RSH=ssh
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/toon co TooN
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co libcvd
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co gvars3

It should be noted, however, that the libraries change rapidly. To
ensure compatibility, it may be useful to download the libraries
corresponding to a time at which they were known to be compatible. To
do so, use the following commands:

# export CVS_RSH=ssh
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/toon co -D "Mon May 11 16:29:26 BST 2009" TooN
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co -D "Mon May 11 16:29:26 BST 2009" libcvd
# cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/libcvd co -D "Mon May 11 16:29:26 BST 2009" gvars3

The installation of these libraries is described below.

INSTALLATION
------------
Installation of the dependencies
--------------------------------

The three dependent libraries are all compiled and installed using the
familiar ./configure; make; make install system, however the following
points are worth noting.

On Linux, the following libraries (and their -devel versions) are
required: blas, lapack, perhaps libgfortran, ncurses and libreadline
(optional, for GVars3), libdc1394 (and maybe libraw1394)
for firewire capture, optionally libtiff, libjpeg, libpng.

(On OSX, these or their equivalents should all be available with the
system.)

The order of installation should be 
1. TooN, 2. libCVD, 3. GVars3;

TooN installation is trivial, since it's only a bunch of headers.

For libCVD, I recommend the following configure options:
# export CXXFLAGS=-D_REENTRANT
# ./configure --without-ffmpeg

if the compiler hangs on one of the FAST detectors these can be
disabled with configure --disable-fast7 (for example.)
Documentation can be generated (if doxygen is installed) by running
# make docs

For GVars3, I recommend the following configure options:
# ./configure --disable-widgets

Compiling the Software
----------------------
The source code is in the PTAM directory.

The first step is to copy the appropriate platform build files to the
PTAM source directory. Eg. for linux, copy all the files from
PTAM/Build/Linux to PTAM. The Makefile can then be edited to
reference any custom include or linker paths which might be necessary
(depending on where the dependencies were installed.)

The second step, for Linux, is to set up the correct video source. Two
files are provided, VideoSource_Linux_DV.cc and
VideoSource_Linux_V4L.cc, which work with the Unibrain Fire-i and the
Logitech Quickcam Pro 5000 respectively. The DV version is compiled by
default; edit the Makefile to switch to the V4L version instead, if
needed. Other cameras may require manual editing of the video input
files, e.g. to change the videobuffer's colourspace.

Other video source classes are available with libCVD. Finally, if a
custom video source not supported by libCVD is required, the code for
it will have to be put into some VideoSource_XYZ.cc file (the
interface for this file is very simple.)

