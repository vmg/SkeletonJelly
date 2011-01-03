import skeletonjelly
import pyglet
import ctypes
from pyglet.gl import *

window_img = pyglet.window.Window(visible=False)
window_depth = pyglet.window.Window(visible=False)

@window_depth.event
def on_draw():
    background.blit_tiled(0, 0, 0, w, h)
    frame_depth.blit(0, 0, 0)

@window_img.event
def on_draw():
    frame_img.blit(0, 0, 0)

def kinect_tick(t):
    k.tick()

    k.renderImage(texture_img, w)
    frame_img.set_data("RGBA", w * -4, texture_img)

    k.renderDepth(texture_depth, True, w)
    frame_depth.set_data("RGBA", w * -4, texture_depth)

def kinect_callback(cb_type, user_id):
    messages = [
        "New user",
        "Lost user",
        "Pose detected",
        "Calibration start",
        "Calibration success!",
        "Calibration failed"
    ]
    print "CB [User %d]: %s" % (user_id, messages[cb_type])
    if cb_type == 4:
        calibrated = True
        print k.getCoM(True)

if __name__ == '__main__':
    calibrated = False
    k = skeletonjelly.Kinect()

    k.setEventCallback(kinect_callback)
    k.setTicksPerSecond(30)

    k.init(skeletonjelly.Kinect.SENSOR_VGA_30FPS,
            skeletonjelly.Kinect.SENSOR_VGA_30FPS)

    w, h = k.getDepthResolution()
    size = k.getDepthTexSize()

    texture_depth = ctypes.create_string_buffer(size)
    texture_img = ctypes.create_string_buffer(size)

    frame_depth = pyglet.image.create(w, h)
    frame_img = pyglet.image.create(w, h)

    checks = pyglet.image.create(32, 32, pyglet.image.CheckerImagePattern())
    background = pyglet.image.TileableTexture.create_for_image(checks)

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    window_depth.width = w
    window_depth.height = h
    window_depth.set_visible()

    window_img.width = w
    window_img.height = h
    window_img.set_visible()

    pyglet.clock.schedule_interval(kinect_tick, 1.0 / 30.0)
    pyglet.app.run()

