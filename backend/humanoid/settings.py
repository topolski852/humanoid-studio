import json
from pathlib import Path

_SETTINGS_PATH = Path.home() / ".config" / "humanoid-studio" / "settings.json"
_DEFAULT_CONFIG = str(Path(__file__).parents[2] / "configs" / "humanoid_lite.json")


def load() -> dict:
    base = {
        "config_path": _DEFAULT_CONFIG,
        # Flash Wizard defaults. phase_order is a hardware constant (identical for
        # identically-wired motors), so the commutation check that can flip it is
        # OFF by default — it only works reliably on a motor free to spin and can
        # corrupt an assembled/loaded joint. The default phase is inverted (True),
        # the correct value for the standard wiring; override per-run in the wizard.
        "commutation_check": False,       # run the auto commutation check by default?
        "default_phase_inverted": True,   # starting phase_order for commissioning
    }
    if _SETTINGS_PATH.exists():
        try:
            return {**base, **json.loads(_SETTINGS_PATH.read_text())}
        except Exception:
            pass
    return base


def save(data: dict) -> None:
    _SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    _SETTINGS_PATH.write_text(json.dumps({**load(), **data}, indent=2))
