"""
App settings endpoints.

GET /settings   — return current settings (config_path, etc.)
PUT /settings   — update settings; live-updates config_path in app state
"""
from __future__ import annotations

from pathlib import Path

from fastapi import APIRouter, Request

from humanoid import settings as _settings

router = APIRouter(tags=["settings"])


@router.get("/settings", response_model=None)
async def get_settings() -> dict:
    return {"success": True, "data": _settings.load(), "error": None}


@router.put("/settings", response_model=None)
async def put_settings(request: Request) -> dict:
    body = await request.json()
    _settings.save(body)
    if "config_path" in body:
        request.app.state.config_path = Path(body["config_path"])
    return {"success": True, "data": _settings.load(), "error": None}
