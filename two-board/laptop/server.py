"""Laptop-side voice server for the rover.

    pip install -r requirements.txt
    uvicorn server:app --host 0.0.0.0 --port 8000

The ESP32 POSTs raw 16 kHz / 16-bit mono PCM to /command. The server transcribes it with
faster-whisper, runs it through planner.Dialogue, and returns JSON containing the sentence the
rover should speak. Only run this on a network you trust: there is no authentication.
"""
import os
import time
import wave

import numpy as np
from fastapi import FastAPI, Request, Response
from fastapi.concurrency import run_in_threadpool
from faster_whisper import WhisperModel
from pydantic import BaseModel

import planner

SAMPLE_RATE = 16000
DEFAULT_POS = (float(os.getenv("START_LAT", "51.5074")), float(os.getenv("START_LON", "-0.1278")))  # TODO: your start
# Nudges Whisper towards the words we expect; edit for your area.
PROMPT = "Tesco, Sainsbury's, Waitrose, Cromwell Road, Gloucester Road, Old Brompton Road. Yes. No. Stop. Cancel."

app = FastAPI(title="Rover voice server")
model = WhisperModel(os.getenv("WHISPER_MODEL", "base.en"), device="cpu", compute_type="int8")
dialogue = planner.Dialogue()


def save_wav(pcm: bytes, path: str = "last.wav") -> None:
    """Keep the last utterance so you can listen to what the mic actually heard."""
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(pcm)


def transcribe(pcm: bytes) -> str:
    samples = np.frombuffer(pcm[: len(pcm) // 2 * 2], dtype="<i2").astype(np.float32) / 32768.0
    if samples.size == 0:
        return ""
    # The PDM mic is quiet and the firmware sends raw audio, so lift quiet recordings for Whisper.
    peak = float(np.max(np.abs(samples)))
    if 0.0 < peak < 0.3:
        samples = samples * (0.7 / peak)
    segments, _ = model.transcribe(samples, language="en", beam_size=1, vad_filter=True, initial_prompt=PROMPT)
    return " ".join(s.text.strip() for s in segments).strip()


def process(pcm: bytes, here: tuple[float, float]) -> dict:
    t0 = time.time()
    save_wav(pcm)
    text = transcribe(pcm)
    result = dialogue.handle(text, here)
    result["transcript"] = text
    result["latency_s"] = round(time.time() - t0, 2)
    return result


@app.post("/command")
async def command(req: Request):
    pcm = await req.body()
    here = (float(req.headers.get("x-lat", DEFAULT_POS[0])), float(req.headers.get("x-lon", DEFAULT_POS[1])))
    return await run_in_threadpool(process, pcm, here)


class TextIn(BaseModel):
    text: str
    lat: float = DEFAULT_POS[0]
    lon: float = DEFAULT_POS[1]


@app.post("/text")
async def text_command(body: TextIn):
    """Same as /command but skips speech-to-text. Handy for testing the planner with curl."""
    result = await run_in_threadpool(dialogue.handle, body.text, (body.lat, body.lon))
    result["transcript"] = body.text
    return result


@app.get("/route/local.bin")
def active_route_local():
    """Compact binary route for the ESP32 (layout documented in planner.pack_route_local)."""
    if not dialogue.active:
        return Response(status_code=404)
    return Response(content=planner.pack_route_local(dialogue.active[1]), media_type="application/octet-stream")


@app.get("/route")
def active_route():
    """Waypoints of the confirmed route (lat/lon every ~1.5 m) for the motor controller."""
    if not dialogue.active:
        return {"active": False}
    dest, route = dialogue.active
    return {"active": True, "destination": dest.label(), "distance_m": round(route.distance_m),
            "waypoints": route.waypoints, "maneuvers": route.maneuvers}
