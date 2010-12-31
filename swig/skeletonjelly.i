%module skeletonjelly

%{
    #include "../src/skeletonjelly.hpp"
%}

#ifdef SWIGPYTHON
    %include "skeletonjelly_py.i"
#endif

%include "../src/skeletonjelly.hpp"
