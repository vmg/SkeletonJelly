import skeletonjelly
import pyglet
import ctypes
from pyglet.gl import *

window = pyglet.window.Window(visible=False)

@window.event
def on_draw():
    background.blit_tiled(0, 0, 0, window.width, window.height)
    frame.blit(0, 0, 0)

def kinect_tick(t):
    k.tick()

    k.renderImage(texture, w)
    frame.set_data("RGBA", w * -4, texture)

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

    k.init(skeletonjelly.Kinect.SENSOR_VGA_30FPS,
            skeletonjelly.Kinect.SENSOR_VGA_30FPS)

    w, h = k.getDepthResolution()
    size = k.getDepthTexSize()

    texture = ctypes.create_string_buffer(size)
    frame = pyglet.image.create(w, h)

    checks = pyglet.image.create(32, 32, pyglet.image.CheckerImagePattern())
    background = pyglet.image.TileableTexture.create_for_image(checks)

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    window.width = w
    window.height = h
    window.set_visible()

    pyglet.clock.schedule_interval(kinect_tick, 1.0 / 30.0)
    pyglet.app.run()

