%{
    void _pyKinectCallback(
        Kinect *kinect, 
        Kinect::CallbackType t, 
        XnUserID id, 
        void *data)
    {
        PyObject *arglist;
        PyObject *result;

        PyGILState_STATE gstate;

        if (kinect->isThreaded())
            gstate = PyGILState_Ensure();

        arglist = Py_BuildValue("(ii)", t, id);
        result = PyObject_CallObject((PyObject *)data, arglist);

        Py_XDECREF(result);
        Py_DECREF(arglist);

        if (kinect->isThreaded())
            PyGILState_Release(gstate);
    }

%}

%typemap(in) unsigned char *buffer {
    PyObject *ctypes, *ptr;

    ctypes = PyImport_ImportModule("ctypes");
    ptr = PyObject_CallMethod(ctypes, "addressof", "O", $input);
    if (ptr == NULL)
        return NULL;

    $1 = (unsigned char *)PyInt_AsLong(ptr);

    Py_INCREF($input);
    Py_DECREF(ptr);
    Py_DECREF(ctypes);
}

%typemap(out) XnUInt32XYPair* {
    if ($1 == NULL) {
        $result = Py_BuildValue("");
    } else {
        $result = Py_BuildValue("(ii)", $1->X, $1->Y);
    }
}

%typemap(out) XnPoint3D* {
    if ($1 == NULL) {
        $result = Py_BuildValue("");
    } else {
        $result = Py_BuildValue("(fff)", $1->X, $1->Y, $1->Z);
    }
}

%typemap(in) (Kinect::Callback callback, void *userData) {
    Py_INCREF($input);
    $1 = _pyKinectCallback;
    $2 = (void *)$input;
}

%typemap(out) XnStatus {
    if ($1 != XN_STATUS_OK) {
        PyErr_SetString(PyExc_RuntimeError, xnGetStatusString($1));
        return NULL;
    }

    $result = Py_BuildValue("");
}

