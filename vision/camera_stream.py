class CameraStream:
    def __init__(self, source: int = 0) -> None:
        self.source = source
        self.running = False

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False
