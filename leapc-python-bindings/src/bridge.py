import asyncio
import threading
import websockets
import json
import numpy as np

def convertopython(data):
    if isinstance(data, np.ndarray):
        return data.tolist()
    if isinstance(data, (np.float32, np.float64, np.int32, np.int64)):
        return data.item()
    if isinstance(data, dict):
        return {key: convertopython(value) for key, value in data.items()}
    if isinstance(data, (list, tuple)):
        return [convertopython(item) for item in data]
    return data

class Bridge:
    def __init__(self, uri="ws://0.0.0.0:8765"):
        self.uri = uri
        self.loop = asyncio.get_event_loop()
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()
        self.outgoing_queue = asyncio.Queue()
        self.connected = False
    
    def run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connection_loop())
    
    async def connection_loop(self):
        while True:
            try:
                print(f"[Bridge]: Connecting to {self.uri}")
                async with websockets.connect(self.uri) as websocket:
                    print("[Bridge]: Connected")
                    self.connected = True
                    await self.send_async(websocket)
            except Exception as e:
                print(f"Error connecting to {self.uri}: {e}")
                self.connected = False
                await asyncio.sleep(1)
    
    async def send_async(self, websocket):
        while True:
            data = await self.outgoing_queue.get()
            payload = json.dumps(convertopython(data))
            await websocket.send(payload)
            print(f"[Bridge]: send {payload}")
    
    def send(self, data: dict):
        if not isinstance(data, dict):
            raise ValueError("Data must be a dictionary")
        asyncio.run_coroutine_threadsafe(self.outgoing_queue.put(data), self.loop)
        