def options(opt):
    opt.load('compiler_c++')

def configure(conf):
    conf.load('compiler_c++') 
    conf.env.FRAMEWORK = ['OpenGL', 'GLUT']

def build(bld):
    bld.program(source=['src/skeletonjelly.cpp', 'examples/position/main.cpp'],
        lib = ['openni'], 
        libpath = ['/usr/lib'],
        includes = ['src', '/usr/include/ni', 'examples/position'],
        target='kinect_test')
