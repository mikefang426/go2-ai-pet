class WakeWordDetector:
    def __init__(self, wake_word: str = "go2") -> None:
        self.wake_word = wake_word.lower()

    def detect(self, text: str) -> bool:
        return self.wake_word in text.lower()
