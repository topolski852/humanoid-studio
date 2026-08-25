import json
import os
import shutil
from pathlib import Path

_USER_DIR = Path.home() / ".config" / "humanoid-studio"
_SETTINGS_PATH = _USER_DIR / "settings.json"

# The config shipped with the app. In a source checkout this is the repo's
# configs/ directory and is writable in place. In a packaged build (AppImage)
# it lives inside the read-only squashfs mount, e.g.
#   /tmp/.mount_humanoXXXXXX/resources/configs/humanoid_lite.json
_BUNDLED_CONFIG = Path(__file__).parents[2] / "configs" / "humanoid_lite.json"

# Writable copy used when the bundled one is read-only. Calibration, commission
# results and CAN assignments are all persisted by rewriting the config file, so
# a read-only path fails every save (Flash Wizard included).
_USER_CONFIG = _USER_DIR / "humanoid_lite.json"


def _bundled_is_writable() -> bool:
    # The directory matters, not the file: to_json() rewrites in place, and
    # can_adapter also drops ui_state.json alongside it.
    return os.access(_BUNDLED_CONFIG.parent, os.W_OK)


def resolve_config_path() -> Path:
    """
    Return a config path that is actually writable.

    Source checkout: the repo config, used in place.
    Packaged build:  a per-user copy under ~/.config/humanoid-studio, seeded
                     once from the bundled read-only default.
    """
    if _bundled_is_writable():
        return _BUNDLED_CONFIG

    if not _USER_CONFIG.exists() and _BUNDLED_CONFIG.exists():
        _USER_DIR.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(_BUNDLED_CONFIG, _USER_CONFIG)

    return _USER_CONFIG


def _is_stale(path: Path) -> bool:
    """
    A saved config_path that can no longer be used.

    An AppImage mounts under a fresh /tmp/.mount_<random> every launch, so a
    path saved from a previous run points at a mount that no longer exists —
    and even when it does resolve, it is read-only.
    """
    if not path.exists():
        return True
    return "/.mount_" in str(path) and not os.access(path.parent, os.W_OK)


def load() -> dict:
    base = {
        "config_path": str(resolve_config_path()),
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
            merged = {**base, **json.loads(_SETTINGS_PATH.read_text())}
            if _is_stale(Path(merged["config_path"])):
                merged["config_path"] = base["config_path"]
            return merged
        except Exception:
            pass
    return base


def save(data: dict) -> None:
    _SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    _SETTINGS_PATH.write_text(json.dumps({**load(), **data}, indent=2))
