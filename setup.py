#!/usr/bin/env python

"""
SkeletonJelly setup file
"""

import os
from distutils.core import setup, Extension

OPENNI_LIB = os.environ['OPEN_NI_LIB']
OPENNI_INC = os.environ['OPEN_NI_INCLUDE']

skj_mod = Extension('_skeletonjelly',
    sources=['src/skeletonjelly.cpp', 'swig/skeletonjelly.i'],
    include_dirs = [OPENNI_INC],
    library_dirs = [OPENNI_LIB],
    libraries = ['openni'],
    language = 'c++',
    swig_opts=['-c++'],
    extra_compile_args = ['/Zi', '/EHsc'],
    extra_link_args = ['/DEBUG'],
)

setup(name = "skeletonjelly",
    version = "0.2",
    description = "Kinect Skeleton tracking for people with mild brain damage",
    ext_modules = [skj_mod],
    package_dir = {'' : 'swig'},
    py_modules = ["skeletonjelly"],
)
