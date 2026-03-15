from collections import deque
from dataclasses import dataclass
from typing import Deque


@dataclass
class MemoryItem:
    role: str
    content: str


class MemoryStore:
    def __init__(self, max_items: int = 50) -> None:
        self._items: Deque[MemoryItem] = deque(maxlen=max_items)

    def add(self, role: str, content: str) -> None:
        self._items.append(MemoryItem(role=role, content=content))

    def list(self) -> list[MemoryItem]:
        return list(self._items)
