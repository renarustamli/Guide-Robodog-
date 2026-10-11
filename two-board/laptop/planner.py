"""Intent parsing, geocoding and pedestrian routing for the guide rover.

Run it standalone first, no microphone needed:

    python planner.py "I want to go to the Tesco on Cromwell Road" --lat 51.4945 --lon -0.1741 --yes

Services used (both free, both meant for light prototype use only):
  - Nominatim (OpenStreetMap) for geocoding.  Set USER_AGENT to something that identifies you.
  - Valhalla public instance run by OpenStreetMap Germany for walking routes.
For anything beyond testing, self-host Valhalla or use OpenRouteService with an API key.
"""
from __future__ import annotations

import argparse
import math
import re
import struct
from dataclasses import dataclass, field

import requests

USER_AGENT = "rover-prototype/0.1 (personal project; contact: you@example.com)"  # TODO: put your email
NOMINATIM = "https://nominatim.openstreetmap.org"
VALHALLA = "https://valhalla1.openstreetmap.de/route"
HTTP_TIMEOUT = 15


# ----------------------------------------------------------------------------- geometry
def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    r = 6371000.0
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp = p2 - p1
    dl = math.radians(b[1] - a[1])
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def decode_polyline(s: str, precision: int = 6) -> list[tuple[float, float]]:
    """Decode a Google-style encoded polyline (Valhalla uses 6 digits, lat then lon)."""
    coords, idx, lat, lon = [], 0, 0, 0
    factor = 10 ** precision
    while idx < len(s):
        for which in (0, 1):
            shift = result = 0
            while True:
                b = ord(s[idx]) - 63
                idx += 1
                result |= (b & 0x1F) << shift
                shift += 5
                if b < 0x20:
                    break
            delta = ~(result >> 1) if result & 1 else result >> 1
            if which == 0:
                lat += delta
            else:
                lon += delta
        coords.append((lat / factor, lon / factor))
    return coords


def resample(points: list[tuple[float, float]], step_m: float = 1.5) -> list[tuple[float, float]]:
    """Return points spaced step_m apart along the polyline (what a path follower wants)."""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    need = step_m
    for a, b in zip(points, points[1:]):
        seg = haversine_m(a, b)
        if seg == 0:
            continue
        pos = 0.0
        while seg - pos >= need:
            pos += need
            t = pos / seg
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            need = step_m
        need -= seg - pos
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out


# ----------------------------------------------------------------------------- intent
@dataclass
class Intent:
    kind: str                    # navigate | confirm | cancel | stop | where | unknown
    place: str | None = None
    area: str | None = None
    raw: str = ""


_CONFIRM = {"yes", "yeah", "yep", "yup", "ok", "okay", "go", "go ahead", "start", "confirm",
            "lets go", "let's go", "sure", "do it"}
_CANCEL = {"no", "nope", "cancel", "never mind", "nevermind", "abort", "forget it"}
_STOP = {"stop", "halt", "wait", "pause"}
_WHERE = {"where am i", "where are we", "where are we now"}
_NAV = re.compile(
    r"\b(?:go|walk|get|navigate|head|take me|bring me|guide me|directions?|route)\s+(?:over\s+)?to\s+"
    r"(?:the\s+|a\s+|an\s+)?(.+)$"
)
_SPLIT = re.compile(r"^(.+?)\s+(?:on|in|at|near|by|off)\s+(.+)$")


def normalise(text: str) -> str:
    t = text.lower().replace("\u2019", "'")
    t = re.sub(r"[^a-z0-9' -]", " ", t)
    return re.sub(r"\s+", " ", t).strip()


def parse_intent(text: str) -> Intent:
    t = normalise(text)
    if t in _CONFIRM:
        return Intent("confirm", raw=t)
    if t in _CANCEL:
        return Intent("cancel", raw=t)
    if t in _STOP:
        return Intent("stop", raw=t)
    if t in _WHERE:
        return Intent("where", raw=t)
    m = _NAV.search(t)
    if m:
        dest = m.group(1).strip()
        if dest.startswith("the "):
            dest = dest[4:]
        s = _SPLIT.match(dest)
        if s:
            return Intent("navigate", place=s.group(1).strip(), area=s.group(2).strip(), raw=t)
        return Intent("navigate", place=dest, raw=t)
    return Intent("unknown", raw=t)


# ----------------------------------------------------------------------------- geocoding
@dataclass
class Place:
    name: str
    road: str
    lat: float
    lon: float
    dist_m: float
    display: str = ""

    def label(self) -> str:
        return f"{self.name}, {self.road}" if self.name and self.road else (self.name or self.road or self.display)


def _nominatim_search(q: str, near: tuple[float, float], bounded: bool) -> list[dict]:
    d = 0.15  # roughly a 30 km box around the user
    params = {
        "q": q, "format": "jsonv2", "addressdetails": 1, "limit": 10, "countrycodes": "gb",
        "viewbox": f"{near[1] - d * 1.6},{near[0] + d},{near[1] + d * 1.6},{near[0] - d}",
        "bounded": 1 if bounded else 0,
    }
    r = requests.get(f"{NOMINATIM}/search", params=params, headers={"User-Agent": USER_AGENT}, timeout=HTTP_TIMEOUT)
    r.raise_for_status()
    return r.json()


def geocode(place: str, area: str | None, near: tuple[float, float]) -> list[Place]:
    """Find candidates for e.g. ("tesco", "cromwell road") near a position, nearest first."""
    queries = [f"{place} {area}", f"{place}, {area}"] if area else [place]
    raw: list[dict] = []
    for q in queries:
        for bounded in (True, False):
            raw = _nominatim_search(q, near, bounded)
            if raw:
                break
        if raw:
            break
    places = []
    for item in raw:
        addr = item.get("address", {})
        p = Place(
            name=item.get("name") or addr.get("shop") or "",
            road=addr.get("road", ""),
            lat=float(item["lat"]), lon=float(item["lon"]),
            dist_m=haversine_m(near, (float(item["lat"]), float(item["lon"]))),
            display=item.get("display_name", ""),
        )
        places.append(p)
    if area:  # prefer results that actually mention the street
        on_street = [p for p in places if area.lower() in p.display.lower()]
        places = on_street or places
    return sorted(places, key=lambda p: p.dist_m)


def reverse_geocode(pos: tuple[float, float]) -> str:
    r = requests.get(f"{NOMINATIM}/reverse", params={"lat": pos[0], "lon": pos[1], "format": "jsonv2"},
                     headers={"User-Agent": USER_AGENT}, timeout=HTTP_TIMEOUT)
    r.raise_for_status()
    return r.json().get("display_name", "an unknown location")


# ----------------------------------------------------------------------------- routing
@dataclass
class Route:
    distance_m: float
    duration_s: float
    shape: list[tuple[float, float]]
    waypoints: list[tuple[float, float]]
    maneuvers: list[dict] = field(default_factory=list)
    holds: list[int] = field(default_factory=list)   # waypoint indices where the rover waits for confirmation


def get_route(start: tuple[float, float], end: tuple[float, float], step_m: float = 1.5) -> Route:
    body = {
        "locations": [{"lat": start[0], "lon": start[1]}, {"lat": end[0], "lon": end[1]}],
        "costing": "pedestrian",
        "costing_options": {"pedestrian": {"step_penalty": 30, "walking_speed": 4.0}},
        "directions_options": {"units": "kilometers", "language": "en-GB"},
    }
    r = requests.post(VALHALLA, json=body, headers={"User-Agent": USER_AGENT}, timeout=HTTP_TIMEOUT)
    r.raise_for_status()
    trip = r.json()["trip"]
    leg = trip["legs"][0]
    shape = decode_polyline(leg["shape"])
    maneuvers = []
    for m in leg["maneuvers"]:
        lat, lon = shape[min(m.get("begin_shape_index", 0), len(shape) - 1)]
        maneuvers.append({
            "instruction": m.get("instruction", ""),
            "distance_m": round(m.get("length", 0) * 1000),
            "lat": lat, "lon": lon,
        })
    waypoints = resample(shape, step_m)
    return Route(
        distance_m=trip["summary"]["length"] * 1000,
        duration_s=trip["summary"]["time"],
        shape=shape,
        waypoints=waypoints,
        maneuvers=maneuvers,
        holds=find_holds(waypoints, maneuvers),
    )


def find_holds(waypoints: list[tuple[float, float]], maneuvers: list[dict]) -> list[int]:
    """Waypoint indices where the rover must stop and wait for the person to confirm.

    HEURISTIC: any maneuver whose instruction mentions "cross" gets a hold one waypoint before it.
    Valhalla's wording is not guaranteed, so this can MISS crossings. A sturdier source is OSM
    `highway=crossing` nodes near the route (Overpass) or Valhalla trace_attributes
    (edge use = pedestrian_crossing). Check the hold count printed at the start of every route.
    """
    holds: list[int] = []
    for m in maneuvers:
        if "cross" not in m["instruction"].lower():
            continue
        here = (m["lat"], m["lon"])
        idx = min(range(len(waypoints)), key=lambda i: haversine_m(here, waypoints[i]))
        h = max(1, idx - 1)
        if h < len(waypoints) - 1 and (not holds or h > holds[-1]):
            holds.append(h)
    return holds


def to_local_xy(origin: tuple[float, float], pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """lat/lon -> metres east/north of origin (flat-earth approximation, fine over a few km)."""
    kx = 111320.0 * math.cos(math.radians(origin[0]))
    ky = 110540.0
    return [((lon - origin[1]) * kx, (lat - origin[0]) * ky) for lat, lon in pts]


def pack_route_local(route: Route) -> bytes:
    """Compact binary route for the ESP32 (little-endian):
        uint32 n, uint32 nHolds, float64 originLat, float64 originLon,
        n x (float32 x, float32 y)   x = metres east, y = metres north of the origin,
        nHolds x uint32              waypoint indices.
    """
    origin = route.waypoints[0]
    flat = [c for xy in to_local_xy(origin, route.waypoints) for c in xy]
    return (struct.pack("<IIdd", len(route.waypoints), len(route.holds), origin[0], origin[1])
            + struct.pack(f"<{len(flat)}f", *flat)
            + struct.pack(f"<{len(route.holds)}I", *route.holds))


# ----------------------------------------------------------------------------- dialogue
def spoken_distance(m: float) -> str:
    if m >= 1000:
        return f"{m / 1000:.1f} kilometres"
    if m >= 100:
        return f"{int(round(m, -1))} metres"
    return f"{int(m)} metres"


class Dialogue:
    """Tiny state machine: navigate -> (pending, wait for yes) -> active."""

    def __init__(self) -> None:
        self.pending: tuple[Place, Route] | None = None
        self.active: tuple[Place, Route] | None = None

    @staticmethod
    def _reply(action: str, spoken: str, intent: Intent, route: Route | None = None) -> dict:
        out = {"action": action, "spoken": spoken, "intent": intent.kind}
        if route:
            out["route"] = {"distance_m": round(route.distance_m), "duration_s": round(route.duration_s),
                            "waypoints": len(route.waypoints), "steps": len(route.maneuvers),
                            "holds": len(route.holds)}
        return out

    def handle(self, text: str, here: tuple[float, float]) -> dict:
        intent = parse_intent(text)
        try:
            return self._handle(intent, here)
        except requests.RequestException as exc:
            return self._reply("error", "Sorry, I can't reach the map service right now.", intent) | {"error": str(exc)}

    def _handle(self, intent: Intent, here: tuple[float, float]) -> dict:
        if intent.kind == "navigate":
            cands = geocode(intent.place or "", intent.area, here)
            wanted = f"{intent.place} on {intent.area}" if intent.area else intent.place
            if not cands:
                return self._reply("none", f"Sorry, I couldn't find {wanted}. Please try again.", intent)
            dest = cands[0]
            route = get_route(here, (dest.lat, dest.lon))
            self.pending = (dest, route)
            mins = max(1, round(route.duration_s / 60))
            spoken = (f"{dest.label()}. {spoken_distance(route.distance_m)}, about {mins} "
                      f"minute{'s' if mins != 1 else ''} walking. Say yes to start, or no to cancel.")
            if len(cands) > 1 and cands[1].dist_m < 1.5 * dest.dist_m:
                spoken += " There are other matches nearby."
            return self._reply("confirm", spoken, intent, route)

        if intent.kind == "confirm":
            if not self.pending:
                return self._reply("none", "There is nothing to confirm.", intent)
            self.active, self.pending = self.pending, None
            first = self.active[1].maneuvers[0]["instruction"] if self.active[1].maneuvers else "Head to the destination."
            return self._reply("start", f"Starting. {first}", intent, self.active[1])

        if intent.kind == "cancel":
            self.pending = None
            return self._reply("cancel", "Cancelled.", intent)

        if intent.kind == "stop":
            self.active = None
            return self._reply("stop", "Stopping.", intent)

        if intent.kind == "where":
            return self._reply("none", f"We are near {reverse_geocode(here)}.", intent)

        return self._reply("none", "Sorry, I didn't catch that. Try saying: take me to Tesco on Cromwell Road.", intent)


# ----------------------------------------------------------------------------- CLI
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("text")
    ap.add_argument("--lat", type=float, default=51.5074)
    ap.add_argument("--lon", type=float, default=-0.1278)
    ap.add_argument("--yes", action="store_true", help="also confirm and print the waypoints")
    args = ap.parse_args()
    here = (args.lat, args.lon)

    d = Dialogue()
    res = d.handle(args.text, here)
    print(res)
    if args.yes and d.pending:
        print(d.handle("yes", here))
        route = d.active[1]
        print(f"\n{len(route.waypoints)} waypoints, first 5: {route.waypoints[:5]}")
        for m in route.maneuvers:
            print(f"  {m['distance_m']:>5} m  {m['instruction']}")
