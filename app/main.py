import os
import signal
import asyncio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

app = FastAPI()

cors_origin = os.getenv("CORS_ORIGIN", "*")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[cors_origin],
    allow_methods=["*"],
    allow_headers=["*"],
)

_shutdown_event = asyncio.Event()


def _handle_sigterm(*_):
    _shutdown_event.set()


signal.signal(signal.SIGTERM, _handle_sigterm)


@app.get("/health")
async def health():
    return {"status": "ok"}


@app.get("/")
async def index():
    return FileResponse("app/static/index.html")
