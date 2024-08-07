#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
import cv2
from av import VideoFrame

BYE = "BYE"  # Define the BYE constant

class CameraVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            raise IOError("Cannot open video device")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        ret, frame = self.cap.read()
        if not ret:
            raise IOError("Failed to read frame from video device")

        # Convert the frame from BGR to YUV format
        yuv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)
        video_frame = VideoFrame.from_ndarray(yuv_frame, format="yuv420p")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        self.cap.release()

class WebRTCStreamer(Node):
    def __init__(self):
        super().__init__('webrtc_streamer')
        self.declare_parameter('signaling_host', '192.168.1.100')  # Replace with your laptop's IP address
        self.declare_parameter('signaling_port', 9000)
        self.pc = RTCPeerConnection()
        self.signaling_host = self.get_parameter('signaling_host').get_parameter_value().string_value
        self.signaling_port = self.get_parameter('signaling_port').get_parameter_value().integer_value
        self.signaling = TcpSocketSignaling(self.signaling_host, self.signaling_port)
        self.video_track = CameraVideoStreamTrack()
        self.pc.addTrack(self.video_track)

    async def run(self):
        await self.signaling.connect()

        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            self.get_logger().info(f"ICE connection state is {self.pc.iceConnectionState}")
            if self.pc.iceConnectionState == "failed":
                await self.pc.close()
                await self.signaling.close()

        while True:
            obj = await self.signaling.receive()
            if isinstance(obj, RTCSessionDescription):
                await self.pc.setRemoteDescription(obj)
                if obj.type == "offer":
                    await self.pc.setLocalDescription(await self.pc.createAnswer())
                    await self.signaling.send(self.pc.localDescription)
            elif obj == BYE:
                break

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCStreamer()
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.video_track.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
