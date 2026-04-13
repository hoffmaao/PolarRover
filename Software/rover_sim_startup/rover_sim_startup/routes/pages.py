"""HTML page routes (Jinja2 server-rendered)."""

from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse

from rover_sim_startup.storage import list_scenarios, load_scenario

router = APIRouter()


@router.get("/", response_class=HTMLResponse)
async def index(request: Request):
    scenarios = list_scenarios(request.app.state.scenarios_dir)
    return request.app.state.templates.TemplateResponse(
        request, "index.html", {"scenarios": scenarios}
    )


@router.get("/scenarios/new", response_class=HTMLResponse)
async def new_scenario(request: Request):
    return request.app.state.templates.TemplateResponse(
        request, "scenario_edit.html", {"scenario": None, "is_new": True}
    )


@router.get("/scenarios/{name}/edit", response_class=HTMLResponse)
async def edit_scenario(request: Request, name: str):
    try:
        cfg = load_scenario(request.app.state.scenarios_dir, name)
    except FileNotFoundError:
        return HTMLResponse(f"Scenario {name!r} not found", status_code=404)
    return request.app.state.templates.TemplateResponse(
        request, "scenario_edit.html", {"scenario": cfg, "is_new": False}
    )


@router.get("/scenarios/{name}/mission", response_class=HTMLResponse)
async def mission_map(request: Request, name: str):
    try:
        cfg = load_scenario(request.app.state.scenarios_dir, name)
    except FileNotFoundError:
        return HTMLResponse(f"Scenario {name!r} not found", status_code=404)
    return request.app.state.templates.TemplateResponse(
        request, "mission_map.html", {"scenario": cfg}
    )


@router.get("/log-viewer", response_class=HTMLResponse)
async def log_viewer(request: Request):
    return request.app.state.templates.TemplateResponse(
        request, "log_viewer.html"
    )
