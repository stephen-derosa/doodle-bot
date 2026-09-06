from __future__ import annotations

from importlib.metadata import entry_points

from .contracts import Stage


class StageRegistry:
    def __init__(self) -> None:
        self._stages: dict[str, Stage] = {}

    def register(self, stage: Stage) -> None:
        if stage.type_name in self._stages:
            raise ValueError(f"Duplicate stage type: {stage.type_name}")
        self._stages[stage.type_name] = stage

    def get(self, type_name: str) -> Stage:
        try:
            return self._stages[type_name]
        except KeyError as error:
            known = ", ".join(sorted(self._stages)) or "none"
            raise KeyError(f"Unknown stage type '{type_name}'. Registered: {known}") from error

    def load_plugins(self) -> None:
        for entry_point in entry_points(group="drawing_bot.stages"):
            loaded = entry_point.load()
            self.register(loaded() if isinstance(loaded, type) else loaded)
