from .intent_parser import IntentParser, Intent
from .memory import MemoryStore


class LLMAgent:
    """Placeholder LLM orchestrator; can be replaced by OpenAI/other provider."""

    def __init__(self) -> None:
        self.parser = IntentParser()
        self.memory = MemoryStore()

    def infer_intent(self, user_text: str) -> Intent:
        self.memory.add("user", user_text)
        intent = self.parser.parse(user_text)
        self.memory.add("assistant", f"intent={intent.name}")
        return intent
