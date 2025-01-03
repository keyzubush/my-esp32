import socketpool
import wifi
from adafruit_httpserver import Server, Request, Response

import board
import espcamera

cam = espcamera.Camera(
    data_pins=board.CAMERA_DATA,
    external_clock_pin=board.CAMERA_XCLK,
    pixel_clock_pin=board.CAMERA_PCLK,
    vsync_pin=board.CAMERA_VSYNC,
    href_pin=board.CAMERA_HREF,
    powerdown_pin=board.CAMERA_PWDN,
    reset_pin=None,
    i2c=board.I2C(),
    external_clock_frequency=20_000_000,
    pixel_format=espcamera.PixelFormat.JPEG,
    frame_size=espcamera.FrameSize.SVGA,
    # jpeg_quality=0,
    # framebuffer_count=0,
    # grab_mode=espcamera.GrabMode.WHEN_EMPTYi
    )

from adafruit_httpserver import Server, Request, ChunkedResponse

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, debug=True)

@server.route("/pic")
def chunked(request: Request):
    frame = cam.take(1)
    return Response(request, content_type="image/jpeg", body=frame)

server.serve_forever(str(wifi.radio.ipv4_address))

