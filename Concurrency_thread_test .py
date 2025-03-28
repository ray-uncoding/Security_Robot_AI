import queue
import threading

class SharedFlow:
    def __init__(self):
        self._queue = queue.Queue()
        self._subscribers = []

    def subscribe(self, subscriber):
        self._subscribers.append(subscriber)
        def worker():
            while True:
                value = self._queue.get()
                subscriber(value)
        threading.Thread(target=worker, daemon=True).start()

    def emit(self, value):
        self._queue.put(value)
        for subscriber in self._subscribers:
            subscriber(value)

# 使用示例
def subscriber1(value):
    print(f"订阅者1收到: {value}")

def subscriber2(value):
    print(f"订阅者2收到: {value}")

flow = SharedFlow()
flow.subscribe(subscriber1)
flow.subscribe(subscriber2)

flow.emit("状态1")  # 输出: 订阅者1收到: 状态1
                    #       订阅者2收到: 状态1
flow.emit("状态2")  # 输出: 订阅者1收到: 状态2
                    #       订阅者2收到: 状态2