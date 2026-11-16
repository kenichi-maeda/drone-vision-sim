"""STUB: camera feed from a Daffy (ROS 1 pidrone_pkg) drone.

Not implemented. The physical drone's software stack has not been decided yet.
An Ente/PX4 drone does not need this: it uses the link feed with
overwatch_link running on the drone.

What the Daffy drone provides (from the DD24 manual and h2r/pidrone_pkg):
  * raspicam_node publishes /raspicam_node/image/compressed
    (sensor_msgs/CompressedImage, 320x240 at 30 fps, launch/raspicam_node.launch)
  * web_video_server (started from pi.screenrc, not by default) serves MJPEG at
      http://<drone>:8080/stream?topic=/raspicam_node/image&type=ros_compressed

To implement: read the multipart MJPEG stream (boundary from the Content-Type
header), and for each JPEG part call
    self._emit(Frame(seq=n, jpeg=part, received_at=monotonic(), stamp=None, source="mjpeg"))
Never queue parts; if the consumer is slow, frames are simply replaced.
"""

from __future__ import annotations

from .base import FeedSource, FrameCallback


class MjpegFeed(FeedSource):
    name = "mjpeg"

    def __init__(self, url: str) -> None:
        super().__init__()
        self.url = url
        raise NotImplementedError(
            "The Daffy MJPEG feed is a stub (overwatch/feed/mjpeg.py). For an Ente/PX4 "
            "drone, use the 'live' source with overwatch_link instead.")

    async def start(self, callback: FrameCallback) -> None:  # pragma: no cover - stub
        raise NotImplementedError

    async def stop(self) -> None:  # pragma: no cover - stub
        return None

    def connected(self) -> bool:  # pragma: no cover - stub
        return False
