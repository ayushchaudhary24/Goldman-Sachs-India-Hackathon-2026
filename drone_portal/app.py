#!/usr/bin/env python3
# ============================================================================
#  Multi-Agent Drone Routing Simulation System
#  Flask Backend  --  app.py
# ----------------------------------------------------------------------------
#  This backend treats the existing Python solver (solver/score62.py) as a
#  BLACK BOX computation engine.  The flow is strictly:
#
#       Input JSON  ->  Python Solver  ->  Flight Manifest  ->  Website
#
#  The solver is invoked as an external subprocess exactly the way the
#  contest grader invokes it: the input JSON is written to stdin and the
#  flight-manifest JSON is read back from stdout.  The solver source is NOT
#  imported, modified, rewritten or optimised in any way.
#
#  Every statistic, score, battery value, timeline and route shown on the
#  website is derived ONLY from the manifest the solver returns.  Nothing is
#  fabricated.
# ============================================================================

import os
import sys
import json
import math
import subprocess

from flask import Flask, render_template, request, jsonify

# ---------------------------------------------------------------------------
#  Paths
# ---------------------------------------------------------------------------
BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
SOLVER_DIR = os.path.join(BASE_DIR, "solver")
SOLVER     = os.path.join(SOLVER_DIR, "score62.py")

# ---------------------------------------------------------------------------
#  Physical constants (taken verbatim from the problem statement).  These are
#  used ONLY to re-derive per-step battery/energy figures from the manifest
#  the solver produced.  They never alter the solver's own computation.
# ---------------------------------------------------------------------------
BATTERY_CAP = 500.0     # energy units, full charge
CHARGE_RATE = 2.0       # energy units gained per timestep at a station
SPEED       = 1.0       # distance units per timestep

app = Flask(__name__)


# ===========================================================================
#  SOLVER INVOCATION
# ===========================================================================
def run_solver(input_data, timeout=30):
    """Run the existing Python solver as a subprocess.

    The solver reads a JSON object from stdin and prints a JSON object of the
    form {"flight_manifest": [...]} to stdout.  We do not interpret or change
    the algorithm -- we only marshal data in and out.

    Returns: (manifest_dict, error_string_or_None)
    """
    payload = json.dumps(input_data)
    try:
        proc = subprocess.run(
            [sys.executable, SOLVER],
            input=payload,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return None, "Solver timed out (computation exceeded the time limit)."
    except Exception as exc:                       # pragma: no cover
        return None, "Failed to launch solver process: %s" % exc

    if proc.returncode != 0:
        return None, "Solver exited with an error:\n" + (proc.stderr or "unknown")

    out = (proc.stdout or "").strip()
    if not out:
        return None, "Solver produced no output."

    try:
        result = json.loads(out)
    except json.JSONDecodeError as exc:
        return None, "Solver output was not valid JSON: %s" % exc

    return result, None


# ===========================================================================
#  GEOMETRY  (re-derivation helpers only -- never used to route)
# ===========================================================================
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def energy_cost(distance, payload):
    """E_leg = distance * (1 + current_payload_weight)  -- problem statement."""
    return distance * (1.0 + payload)


# ===========================================================================
#  STATISTICS  --  everything below is derived purely from the manifest
# ===========================================================================
def analyse_manifest(manifest, data):
    """Walk the manifest returned by the solver and reconstruct, step by step,
    the battery / energy / payload trajectory of every drone.

    This is a faithful replay of the manifest -- the same accounting rules the
    grader uses -- so that the website can display real numbers.  No value is
    invented; if the solver did not deliver something, it does not appear.
    """
    ms = data.get("map_size", [100, 100])
    warehouse = [float(ms[0]) / 2.0, float(ms[1]) / 2.0]

    # delivery lookup by id (for weights / deadlines)
    dmap = {}
    for d in data.get("deliveries", []):
        dmap[d["id"]] = {
            "id": d["id"],
            "x": float(d["x"]),
            "y": float(d["y"]),
            "weight": float(d["weight"]),
            "deadline": float(d["deadline"]),
        }

    warehouse_key = (round(warehouse[0], 3), round(warehouse[1], 3))
    station_keys = {
        (round(float(s["x"]), 3), round(float(s["y"]), 3))
        for s in data.get("charging_stations", [])
    }

    total_energy   = 0.0
    makespan       = 0.0
    delivered_ids  = set()
    late_ids       = set()
    drone_reports  = []
    sim_log        = []     # right-side scrolling simulation log

    for flight in manifest.get("flight_manifest", []):
        drone_id = flight.get("drone_id", "?")
        path     = flight.get("path", [])

        battery  = BATTERY_CAP
        payload  = 0.0
        carried  = {}        # delivery_id -> weight currently on board
        prev     = None

        d_energy  = 0.0
        d_wait    = 0.0
        d_charge  = 0.0
        d_deliv   = 0
        annotated = []       # enriched path steps for the front-end

        for step in path:
            x = float(step["x"])
            y = float(step["y"])
            t = float(step["t"])
            action = step.get("action", "")

            # --- energy / battery accounting on the travelled leg ---
            leg = 0.0
            if prev is not None:
                leg = dist((prev["x"], prev["y"]), (x, y))
                if leg > 1e-9:
                    e = energy_cost(leg, payload)
                    battery     -= e
                    d_energy    += e
                    total_energy += e

            # --- action semantics ---
            wait_here   = 0.0
            charge_here = 0.0

            if action == "PICKUP":
                # fully recharged at warehouse, payload reset & loaded
                battery = BATTERY_CAP
                carried = {}
                for did in step.get("delivery_ids", []):
                    if did in dmap:
                        carried[did] = dmap[did]["weight"]
                payload = sum(carried.values())

            elif action == "DELIVER":
                did = step.get("delivery_id")
                # count success only if on time and not already delivered
                if did in dmap and did not in delivered_ids:
                    if t <= dmap[did]["deadline"] + 1e-6:
                        delivered_ids.add(did)
                        d_deliv += 1
                    else:
                        late_ids.add(did)
                if did in carried:
                    payload -= carried[did]
                    del carried[did]
                    if payload < 0:
                        payload = 0.0

            elif action == "WAIT":
                if prev is not None:
                    wait_here = t - float(prev["t"]) - leg
                    if wait_here < 0:
                        wait_here = 0.0
                    d_wait += wait_here

            elif action == "CHARGE":
                pass  # arrival at station; charging accrues until CHARGE_COMPLETE

            elif action == "CHARGE_COMPLETE":
                if prev is not None and prev.get("action") == "CHARGE":
                    dur = t - float(prev["t"])
                    if dur > 0:
                        charge_here = dur
                        d_charge   += dur
                        battery = min(BATTERY_CAP, battery + dur * CHARGE_RATE)

            elif action == "RETURN":
                # returning to the warehouse fully recharges the drone
                if (round(x, 3), round(y, 3)) == warehouse_key:
                    battery = BATTERY_CAP

            if battery < 0:
                battery = 0.0
            makespan = max(makespan, t)

            annotated.append({
                "x": round(x, 3),
                "y": round(y, 3),
                "t": round(t, 3),
                "action": action,
                "delivery_id": step.get("delivery_id"),
                "delivery_ids": step.get("delivery_ids"),
                "battery": round(battery, 2),
                "payload": round(payload, 3),
                "leg": round(leg, 3),
                "wait": round(wait_here, 3),
                "charge": round(charge_here, 3),
            })

            sim_log.append({
                "t": round(t, 2),
                "drone": drone_id,
                "action": action,
                "status": _status_for(action, step, dmap, t),
            })

            prev = step

        drone_reports.append({
            "drone_id": drone_id,
            "path": annotated,
            "energy": round(d_energy, 2),
            "deliveries": d_deliv,
            "wait_time": round(d_wait, 2),
            "charge_time": round(d_charge, 2),
            "final_t": round(float(path[-1]["t"]), 2) if path else 0.0,
        })

    # --- overall scoring (exactly the contest formula) ---
    successful = len(delivered_ids)
    raw_score = (successful * 100.0) - (total_energy * 0.1) - (makespan * 0.05)

    total_requested = len(dmap)
    missed = total_requested - successful
    success_rate = (100.0 * successful / total_requested) if total_requested else 0.0

    # drone utilisation = fraction of drones that flew at least one trip
    flying = sum(1 for r in drone_reports if r["path"])
    total_drones = len(data.get("drones", []))
    utilisation = (100.0 * flying / total_drones) if total_drones else 0.0

    total_wait   = sum(r["wait_time"] for r in drone_reports)
    total_charge = sum(r["charge_time"] for r in drone_reports)

    # sort the unified log by time for the scrolling panel
    sim_log.sort(key=lambda r: r["t"])

    stats = {
        "successful_deliveries": successful,
        "missed_deliveries": missed,
        "total_requested": total_requested,
        "total_energy": round(total_energy, 2),
        "makespan": round(makespan, 2),
        "raw_score": round(raw_score, 2),
        "success_rate": round(success_rate, 1),
        "drone_utilisation": round(utilisation, 1),
        "total_wait": round(total_wait, 2),
        "total_charge": round(total_charge, 2),
        "drones_used": flying,
        "drones_total": total_drones,
        "late_deliveries": sorted(late_ids),
    }

    return {
        "drones": drone_reports,
        "stats": stats,
        "sim_log": sim_log,
        "warehouse": [round(warehouse[0], 3), round(warehouse[1], 3)],
        "map_size": [float(ms[0]), float(ms[1])],
        "charging_stations": data.get("charging_stations", []),
        "deliveries": data.get("deliveries", []),
        "no_fly_zones": data.get("no_fly_zones", []),
        "delivered_ids": sorted(delivered_ids),
    }


def _status_for(action, step, dmap, t):
    """Human-readable status string for the scrolling simulation log."""
    if action == "PICKUP":
        ids = step.get("delivery_ids", []) or []
        return "Loaded %d package(s): %s" % (len(ids), ", ".join(ids))
    if action == "DELIVER":
        did = step.get("delivery_id")
        if did in dmap:
            dl = dmap[did]["deadline"]
            if t <= dl + 1e-6:
                return "Delivered %s (on time, deadline %.0f)" % (did, dl)
            return "Delivered %s LATE (deadline %.0f)" % (did, dl)
        return "Delivered %s" % did
    if action == "WAIT":
        return "Waiting for No-Fly Zone to clear"
    if action == "CHARGE":
        return "Arrived at charging station"
    if action == "CHARGE_COMPLETE":
        return "Charging complete, departing"
    if action == "WAYPOINT":
        return "Navigation waypoint"
    if action == "RETURN":
        return "Returned to base"
    return action


# ===========================================================================
#  INPUT VALIDATION  (old-style: collect all problems, return as a list)
# ===========================================================================
def validate_input(data):
    errors = []

    if not isinstance(data, dict):
        return ["Input must be a JSON object."]

    ms = data.get("map_size")
    if (not isinstance(ms, list) or len(ms) != 2
            or not all(isinstance(v, (int, float)) for v in ms)):
        errors.append("map_size must be a list of two numbers [Width, Height].")
    else:
        if ms[0] <= 0 or ms[1] <= 0:
            errors.append("map_size dimensions must be positive.")

    drones = data.get("drones", [])
    if not isinstance(drones, list) or len(drones) == 0:
        errors.append("At least one drone is required.")
    else:
        seen = set()
        for i, d in enumerate(drones):
            if "id" not in d:
                errors.append("Drone #%d is missing an id." % (i + 1))
            elif d["id"] in seen:
                errors.append("Duplicate drone id: %s" % d["id"])
            else:
                seen.add(d["id"])
            mp = d.get("max_payload")
            if not isinstance(mp, (int, float)) or mp <= 0:
                errors.append("Drone %s has an invalid max_payload (must be > 0)."
                              % d.get("id", i + 1))

    deliveries = data.get("deliveries", [])
    if not isinstance(deliveries, list) or len(deliveries) == 0:
        errors.append("At least one delivery is required.")
    else:
        seen = set()
        for i, d in enumerate(deliveries):
            tag = d.get("id", "#%d" % (i + 1))
            if "id" not in d:
                errors.append("Delivery #%d is missing an id." % (i + 1))
            elif d["id"] in seen:
                errors.append("Duplicate delivery id: %s" % d["id"])
            else:
                seen.add(d["id"])
            for f in ("x", "y", "weight", "deadline"):
                if not isinstance(d.get(f), (int, float)):
                    errors.append("Delivery %s has a non-numeric '%s'." % (tag, f))
            if isinstance(d.get("x"), (int, float)) and d["x"] < 0:
                errors.append("Delivery %s has a negative X coordinate." % tag)
            if isinstance(d.get("y"), (int, float)) and d["y"] < 0:
                errors.append("Delivery %s has a negative Y coordinate." % tag)
            if isinstance(d.get("weight"), (int, float)) and d["weight"] <= 0:
                errors.append("Delivery %s has an invalid weight (must be > 0)." % tag)
            if isinstance(d.get("deadline"), (int, float)) and d["deadline"] <= 0:
                errors.append("Delivery %s has an invalid deadline (must be > 0)." % tag)

    stations = data.get("charging_stations", [])
    if not isinstance(stations, list):
        errors.append("charging_stations must be a list.")
    else:
        for i, s in enumerate(stations):
            for f in ("x", "y"):
                if not isinstance(s.get(f), (int, float)):
                    errors.append("Charging station #%d has a non-numeric '%s'."
                                  % (i + 1, f))

    nfzs = data.get("no_fly_zones", [])
    if not isinstance(nfzs, list):
        errors.append("no_fly_zones must be a list.")
    else:
        for i, n in enumerate(nfzs):
            shape = n.get("shape")
            if shape not in ("circle", "rectangle"):
                errors.append("No-Fly Zone #%d has an invalid shape." % (i + 1))
            if shape == "circle":
                if (not isinstance(n.get("center"), list)
                        or len(n.get("center", [])) != 2):
                    errors.append("Circular NFZ #%d needs a center [x, y]." % (i + 1))
                if not isinstance(n.get("radius"), (int, float)) or n.get("radius", 0) <= 0:
                    errors.append("Circular NFZ #%d needs a positive radius." % (i + 1))
            elif shape == "rectangle":
                c = n.get("corners")
                if not isinstance(c, list) or len(c) != 2:
                    errors.append("Rectangular NFZ #%d needs two corners." % (i + 1))
            for f in ("T_start", "T_end"):
                if not isinstance(n.get(f), (int, float)):
                    errors.append("NFZ #%d has a non-numeric '%s'." % (i + 1, f))

    return errors


# ===========================================================================
#  PAGE ROUTES
# ===========================================================================
@app.route("/")
def home():
    return render_template("home.html", active="home")


@app.route("/overview")
def overview():
    return render_template("overview.html", active="overview")


@app.route("/simulation")
def simulation():
    return render_template("simulation.html", active="simulation")


@app.route("/documentation")
def documentation():
    return render_template("documentation.html", active="documentation")


@app.route("/about")
def about():
    return render_template("about.html", active="about")


# ===========================================================================
#  API ROUTE  --  run the simulation
# ===========================================================================
@app.route("/api/simulate", methods=["POST"])
def api_simulate():
    try:
        data = request.get_json(force=True)
    except Exception:
        return jsonify({"ok": False,
                        "errors": ["Request body was not valid JSON."]}), 400

    if data is None:
        return jsonify({"ok": False,
                        "errors": ["No input data received."]}), 400

    # 1) validate
    errors = validate_input(data)
    if errors:
        return jsonify({"ok": False, "errors": errors}), 400

    # 2) run the black-box solver
    manifest, err = run_solver(data)
    if err:
        return jsonify({"ok": False, "errors": [err]}), 500

    # 3) derive statistics purely from the manifest
    report = analyse_manifest(manifest, data)

    return jsonify({
        "ok": True,
        "input": data,
        "manifest": manifest,
        "report": report,
    })


# ===========================================================================
if __name__ == "__main__":
    # Debug server -- intended for local use.
    app.run(host="127.0.0.1", port=5000, debug=True)
