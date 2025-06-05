import asyncio
import websockets
import json

async def main():
    uri = "ws://localhost:3000/control-msg"
    async with websockets.connect(uri) as websocket:
        print(f"已连接到 {uri}")
        while True:
            try:
                msg = await websocket.recv()
                try:
                    data = json.loads(msg)
                    print("收到数据:", json.dumps(data, ensure_ascii=False, indent=2))
                except json.JSONDecodeError:
                    print("收到非JSON消息:", msg)
            except websockets.ConnectionClosed as e:
                print(f"连接已关闭: {e}")
                break

if __name__ == "__main__":
    asyncio.run(main())