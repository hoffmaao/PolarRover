"""FastAPI application factory for the PolarRover mission startup web app."""

from pathlib import Path
from typing import Optional

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates


def create_app(scenarios_dir: Optional[Path] = None) -> FastAPI:
    app = FastAPI(title="PolarRover Mission Startup")

    pkg_dir = Path(__file__).parent
    app.mount("/static", StaticFiles(directory=pkg_dir / "static"), name="static")

    templates = Jinja2Templates(directory=str(pkg_dir / "templates"))
    app.state.templates = templates
    app.state.scenarios_dir = scenarios_dir or Path("~/.rover_sim/scenarios").expanduser()
    app.state.scenarios_dir.mkdir(parents=True, exist_ok=True)

    from rover_sim_startup.routes.pages import router as pages_router
    from rover_sim_startup.routes.api import router as api_router

    app.include_router(pages_router)
    app.include_router(api_router, prefix="/api")

    return app
