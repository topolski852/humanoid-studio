import json
from pathlib import Path

_SETTINGS_PATH = Path.home() / ".config" / "humanoid-studio" / "settings.json"
_DEFAULT_CONFIG = str(Path(__file__).parents[2] / "configs" / "humanoid_lite.json")


def load() -> dict:
    base = {"config_path": _DEFAULT_CONFIG}
    if _SETTINGS_PATH.exists():
        try:
            return {**base, **json.loads(_SETTINGS_PATH.read_text())}
        except Exception:
            pass
    return base


def save(data: dict) -> None:
    _SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    _SETTINGS_PATH.write_text(json.dumps({**load(), **data}, indent=2))
