from .routes_devices import router as devices_router
from .routes_motors import router as motors_router
from .routes_flash import router as flash_router
from .routes_robot import router as robot_router
from .routes_debug import router as debug_router
from .routes_settings import router as settings_router

__all__ = ["devices_router", "motors_router", "flash_router", "robot_router", "debug_router", "settings_router"]
