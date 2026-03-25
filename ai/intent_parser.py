from dataclasses import dataclass, field


@dataclass
class Intent:
    name: str
    slots: dict[str, str] = field(default_factory=dict)


class IntentParser:
    def parse(self, text: str) -> Intent:
        t = text.lower().strip()
        if any(k in t for k in ["跟随", "follow"]):
            return Intent("follow_user")
        if any(k in t for k in ["巡逻", "patrol"]):
            return Intent("patrol")
        if any(k in t for k in ["打招呼", "greet", "greeting", "hello", "hi"]):
            return Intent("greet_person")
        if any(k in t for k in ["翻跟头", "后空翻", "flip", "backflip"]):
            return Intent("flip")
        if any(k in t for k in ["坐下", "sit"]):
            return Intent("sit")
        if any(k in t for k in ["站起", "stand"]):
            return Intent("stand")
        if any(k in t for k in ["停止", "stop"]):
            return Intent("stop")
        return Intent("unknown", {"raw": text})
