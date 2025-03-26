import os
import asyncio
import logging
from pyrtmp import StreamClosedException
from pyrtmp.flv import FLVFileWriter, FLVMediaType
from pyrtmp.session_manager import SessionManager
from pyrtmp.rtmp import SimpleRTMPController, RTMPProtocol, SimpleRTMPServer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("RTMPServer")
logger.setLevel(logging.DEBUG)


# ✅ 封裝 session 狀態，確保擴充性
class SessionState:
    def __init__(self, file_path):
        self.writer = FLVFileWriter(output=file_path)

    def write(self, timestamp, payload, media_type):
        self.writer.write(timestamp, payload, media_type)

    def close(self):
        self.writer.close()


# ✅ RTMP 控制器：處理 metadata, video, audio 串流
class RTMP2FLVController(SimpleRTMPController):

    def __init__(self, output_directory: str):
        self.output_directory = output_directory
        super().__init__()

    async def on_ns_publish(self, session, message) -> None:
        name = message.publishing_name or "unnamed"
        file_path = os.path.join(self.output_directory, f"{name}.flv")
        session.state = SessionState(file_path)
        logger.info(f"開始錄製 RTMP 串流: {file_path}")
        await super().on_ns_publish(session, message)

    async def on_metadata(self, session, message) -> None:
        session.state.write(0, message.to_raw_meta(), FLVMediaType.OBJECT)
        await super().on_metadata(session, message)

    async def on_video_message(self, session, message) -> None:
        session.state.write(message.timestamp, message.payload, FLVMediaType.VIDEO)
        await super().on_video_message(session, message)

    async def on_audio_message(self, session, message) -> None:
        session.state.write(message.timestamp, message.payload, FLVMediaType.AUDIO)
        await super().on_audio_message(session, message)

    async def on_stream_closed(self, session: SessionManager, exception: StreamClosedException) -> None:
        logger.info("RTMP 串流結束，關閉檔案")
        session.state.close()
        await super().on_stream_closed(session, exception)


# ✅ RTMP Server 類別
class SimpleServer(SimpleRTMPServer):
    def __init__(self, output_directory: str):
        self.output_directory = output_directory
        super().__init__()

    async def create(self, host: str, port: int):
        loop = asyncio.get_event_loop()
        self.server = await loop.create_server(
            lambda: RTMPProtocol(controller=RTMP2FLVController(self.output_directory)),
            host=host,
            port=port,
        )


# ✅ 主程式
async def main():
    output_path = os.path.join(os.getcwd(), "recordings")
    os.makedirs(output_path, exist_ok=True)

    server = SimpleServer(output_directory=output_path)
    await server.create(host="0.0.0.0", port=1935)
    logger.info("✅ RTMP Server 啟動於 rtmp://0.0.0.0:1935")
    await server.start()
    await server.wait_closed()


# ✅ 啟動 asyncio RTMP server
if __name__ == "__main__":
    asyncio.run(main())
