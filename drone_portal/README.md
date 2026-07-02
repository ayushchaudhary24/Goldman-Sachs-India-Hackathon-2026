# Multi-Agent Drone Routing Simulator

A complete web portal built around an **existing Python routing solver**. The
website lets a user configure a city grid, a drone fleet and delivery requests
(by form or by JSON upload), runs the solver, and visualises the resulting
**Flight Manifest** — routes, animation, timeline, statistics and score.

The solver (`solver/score62.py`) is treated strictly as a **black box**. It is
never modified, rewritten or optimised. The data flow is:

```
Input JSON  ->  Python Solver  ->  Flight Manifest  ->  Website Visualisation
```

Every visualisation, animation, timeline, statistic, score, battery value and
route shown is derived **only** from the solver's output. Nothing is fabricated.

---

## Project Structure

```
drone_portal/
├── app.py                  Flask backend (validates input, runs solver, derives stats)
├── requirements.txt
├── README.md
├── solver/
│   ├── __init__.py
│   └── score62.py          The existing routing engine (UNCHANGED)
├── templates/
│   ├── base.html           Shared page layout
│   ├── home.html
│   ├── overview.html       Problem overview
│   ├── simulation.html     Simulation portal (form + canvas + log + manifest + stats)
│   ├── documentation.html
│   └── about.html
└── static/
    ├── css/
    │   └── style.css       Classic fixed-width portal styling
    ├── js/
    │   ├── form.js         Dynamic form rows, JSON generate/apply, samples
    │   ├── visualize.js    HTML5 Canvas animation of routes
    │   └── simulation.js   Run orchestration + manifest/log/stats rendering
    └── images/
        ├── logo.png        Neutral drone logo
        ├── bgtile.png      Background tile
        └── drone.png       Small drone icon
```

---

## How It Works

1. **Input** — The user fills the form (Map, Drones, Deliveries, Charging
   Stations, No-Fly Zones) or uploads/pastes a JSON file. "Generate JSON from
   Form" assembles the input object; "Apply JSON to Form" loads JSON back into
   the form.
2. **Validation** — `app.py` validates all inputs (invalid payloads, negative
   coordinates, impossible deadlines, malformed JSON) and returns old-style
   alert boxes on error.
3. **Solve** — The validated input is written to the solver's **stdin**; the
   solver prints the Flight Manifest JSON to **stdout** (the same interface the
   contest grader uses). The solver runs as a subprocess and is never imported
   or altered.
4. **Replay** — `app.py` re-walks the returned manifest using the
   accounting rules (energy `= distance × (1 + payload)`, charge rate 2/timestep,
   battery cap 500, full recharge on RETURN/PICKUP) to derive per-step battery,
   payload, energy, waiting and charging — and the overall score
   `raw_score = (deliveries × 100) − (energy × 0.1) − (makespan × 0.05)`.
5. **Display** — The browser renders the animated route on an HTML5 Canvas, a
   scrolling simulation log, the full Flight Manifest table and the statistics.

---

## Running Locally

Requires **Python 3.8+**.

```bash
cd drone_portal
pip install -r requirements.txt
python app.py
```

Then open <http://127.0.0.1:5000/> in a browser.

On the **Simulation Portal** page you can press *Load Sample 0* / *Load Sample 1*
(from the left panel) to populate the sample inputs, then *Run
Simulation*.

---

## Notes

- The solver is invoked with the same Python interpreter running Flask
  (`sys.executable`), so no extra setup is needed.
- This is an open, non-commercial project, not affiliated with any company or authority. Any place names or sample data are fictional.
- The portal does not perform any routing of its own; it only prepares input,
  invokes the engine, and replays the engine's manifest.
