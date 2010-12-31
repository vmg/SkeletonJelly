import skeletonjelly
import pyglet
import ctypes
from pyglet.gl import *

window = pyglet.window.Window(visible=False)

@window.event
def on_draw():
    frame.set_data("RGBA", w * -4, texture)
    frame.blit(0, 0, 0)

def kinect_tick(t):
    k.tick()

def kinect_callback(cb_type, user_id):
    messages = [
        "New user",
        "Lost user",
        "Pose detected",
        "Calibration start",
        "Calibration failed",
        "Calibration success!"
    ]
    print "CB [User %d]: %s" % (user_id, messages[cb_type])

if __name__ == '__main__':
    k = skeletonjelly.Kinect()

    k.setEventCallback(kinect_callback)
    k.setTicksPerSecond(0)
    k.setRenderMode(skeletonjelly.Kinect.RENDER_DEPTH_FRAME)

    k.init()

    w, h = k.getFrameResolution()
    size = w * h * 4
    texture = ctypes.create_string_buffer(size)
    k.setRenderTarget(texture, size, 640 * 4)

    frame = pyglet.image.create(w, h)

    window.width = w
    window.height = h
    window.set_visible()

    pyglet.clock.schedule_interval(kinect_tick, 1.0 / 30.0)
    pyglet.app.run()

