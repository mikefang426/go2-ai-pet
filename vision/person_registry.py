class PersonRegistry:
    def __init__(self) -> None:
        self._people: dict[str, dict] = {}

    def upsert(self, person_id: str, profile: dict | None = None) -> None:
        self._people[person_id] = profile or {}

    def get(self, person_id: str) -> dict | None:
        return self._people.get(person_id)
