/* ==========================================================================
   visualize.js
   --------------------------------------------------------------------------
   HTML5 Canvas visualisation + animation of the Flight Manifest returned by
   the solver.  It renders the warehouse, charging stations, deliveries, NFZs
   (with time-aware active/inactive colouring) and the drone routes, and
   animates the drones along their paths.

   IMPORTANT: positions, times, battery and payload are interpolated ONLY from
   the solver's manifest steps.  No motion or value is invented.
   ========================================================================== */

var Vis = (function () {
    var canvas, ctx;
    var report = null;          // the analysed report from the server
    var W = 100, H = 100;       // map dimensions (world units)
    var pad = 26;               // pixel padding inside canvas
    var sx = 1, sy = 1;         // world->pixel scale
    var makespan = 0;

    var playing = false;
    var simT = 0;               // current simulation time (world units)
    var speed = 12;             // world-units per real second
    var lastFrame = null;
    var rafId = null;

    /* drone colour palette */
    var COLORS = ["#1a4a8b", "#b30000", "#1c7a1c", "#8a4dbf",
                  "#c47a00", "#0d7d7d", "#a01f6e", "#4d6a1f"];

    function worldToPx(x, y) {
        return [pad + x * sx, canvas.height - pad - y * sy];
    }

    function init(rep) {
        canvas = document.getElementById("field");
        ctx = canvas.getContext("2d");
        report = rep;
        W = rep.map_size[0];
        H = rep.map_size[1];
        sx = (canvas.width - 2 * pad) / W;
        sy = (canvas.height - 2 * pad) / H;

        makespan = rep.stats.makespan || 0;
        document.getElementById("hudMax").textContent = makespan.toFixed(2);

        simT = 0;
        playing = true;
        lastFrame = null;
        document.getElementById("playBtn").value = "Pause";

        buildLegend();
        if (rafId) cancelAnimationFrame(rafId);
        loop();
    }

    /* ---- which deliveries are completed by time t (for colouring) ----- */
    function deliveredBy(t) {
        var done = {};
        report.drones.forEach(function (dr) {
            dr.path.forEach(function (s) {
                if (s.action === "DELIVER" && s.t <= t + 1e-6 && s.delivery_id) {
                    done[s.delivery_id] = true;
                }
            });
        });
        return done;
    }

    /* ---- interpolate a single drone's state at time t ----------------- */
    function droneStateAt(path, t) {
        if (!path.length) return null;
        if (t <= path[0].t) {
            return { x: path[0].x, y: path[0].y,
                     battery: path[0].battery, payload: path[0].payload,
                     action: path[0].action };
        }
        var last = path[path.length - 1];
        if (t >= last.t) {
            return { x: last.x, y: last.y,
                     battery: last.battery, payload: last.payload,
                     action: last.action, done: true };
        }
        for (var i = 0; i < path.length - 1; i++) {
            var a = path[i], b = path[i + 1];
            if (t >= a.t && t <= b.t) {
                var span = b.t - a.t;
                var f = span > 1e-9 ? (t - a.t) / span : 0;
                return {
                    x: a.x + (b.x - a.x) * f,
                    y: a.y + (b.y - a.y) * f,
                    /* battery shown is the value recorded AT each step b;
                       we interpolate toward it for a smooth readout */
                    battery: a.battery + (b.battery - a.battery) * f,
                    payload: a.payload,
                    action: a.action
                };
            }
        }
        return { x: last.x, y: last.y, battery: last.battery,
                 payload: last.payload, action: last.action };
    }

    /* ---- NFZ active test ---------------------------------------------- */
    function nfzActive(n, t) {
        return t >= n.T_start && t < n.T_end;
    }

    /* ---- main draw ---------------------------------------------------- */
    function draw() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        /* background */
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        drawGrid();

        /* NFZs (under everything else) */
        (report.no_fly_zones || []).forEach(function (n) {
            drawNfz(n, nfzActive(n, simT));
        });

        /* routes */
        report.drones.forEach(function (dr, idx) {
            drawRoute(dr.path, COLORS[idx % COLORS.length]);
        });

        /* charging stations */
        (report.charging_stations || []).forEach(function (s) {
            var p = worldToPx(s.x, s.y);
            ctx.fillStyle = "#2f9e2f";
            ctx.fillRect(p[0] - 5, p[1] - 5, 10, 10);
            ctx.strokeStyle = "#145214";
            ctx.strokeRect(p[0] - 5, p[1] - 5, 10, 10);
            ctx.fillStyle = "#145214";
            ctx.font = "9px Arial";
            ctx.fillText("\u26A1", p[0] - 3, p[1] + 3);
        });

        /* deliveries */
        var done = deliveredBy(simT);
        (report.deliveries || []).forEach(function (d) {
            var p = worldToPx(d.x, d.y);
            var isDone = done[d.id];
            ctx.beginPath();
            ctx.arc(p[0], p[1], 4, 0, 2 * Math.PI);
            ctx.fillStyle = isDone ? "#888888" : "#cc8800";
            ctx.fill();
            ctx.strokeStyle = "#555555";
            ctx.stroke();
            ctx.fillStyle = "#333333";
            ctx.font = "9px Arial";
            ctx.fillText(d.id, p[0] + 6, p[1] - 4);
        });

        /* warehouse */
        var w = worldToPx(report.warehouse[0], report.warehouse[1]);
        ctx.fillStyle = "#1a4a8b";
        ctx.fillRect(w[0] - 7, w[1] - 7, 14, 14);
        ctx.strokeStyle = "#0c2851";
        ctx.strokeRect(w[0] - 7, w[1] - 7, 14, 14);
        ctx.fillStyle = "#ffffff";
        ctx.font = "bold 9px Arial";
        ctx.fillText("WH", w[0] - 7, w[1] + 3);

        /* drones (current animated position) */
        var hudParts = [];
        report.drones.forEach(function (dr, idx) {
            var st = droneStateAt(dr.path, simT);
            if (!st) return;
            var p = worldToPx(st.x, st.y);
            var col = COLORS[idx % COLORS.length];
            /* body */
            ctx.beginPath();
            ctx.arc(p[0], p[1], 6, 0, 2 * Math.PI);
            ctx.fillStyle = col;
            ctx.fill();
            ctx.strokeStyle = "#000000";
            ctx.stroke();
            /* rotor cross */
            ctx.strokeStyle = "#ffffff";
            ctx.beginPath();
            ctx.moveTo(p[0] - 4, p[1]); ctx.lineTo(p[0] + 4, p[1]);
            ctx.moveTo(p[0], p[1] - 4); ctx.lineTo(p[0], p[1] + 4);
            ctx.stroke();
            /* label */
            ctx.fillStyle = col;
            ctx.font = "bold 9px Arial";
            ctx.fillText(dr.drone_id, p[0] + 8, p[1] + 3);

            hudParts.push({
                id: dr.drone_id, col: col,
                battery: st.battery, payload: st.payload
            });
        });

        updateHud(hudParts);
    }

    function drawGrid() {
        ctx.strokeStyle = "#eef0f3";
        ctx.lineWidth = 1;
        var step = niceStep(W);
        for (var gx = 0; gx <= W; gx += step) {
            var p = worldToPx(gx, 0), q = worldToPx(gx, H);
            ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
        }
        var stepY = niceStep(H);
        for (var gy = 0; gy <= H; gy += stepY) {
            var a = worldToPx(0, gy), b = worldToPx(W, gy);
            ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        }
        /* border */
        ctx.strokeStyle = "#aaaaaa";
        var o = worldToPx(0, 0), e = worldToPx(W, H);
        ctx.strokeRect(o[0], e[1], e[0] - o[0], o[1] - e[1]);
    }

    function niceStep(range) {
        if (range <= 50) return 10;
        if (range <= 120) return 20;
        if (range <= 300) return 50;
        return 100;
    }

    function drawNfz(n, active) {
        ctx.fillStyle = active ? "rgba(204,0,0,0.28)" : "rgba(232,180,180,0.30)";
        ctx.strokeStyle = active ? "#cc0000" : "#c79a9a";
        ctx.lineWidth = 1;
        if (n.shape === "circle") {
            var c = worldToPx(n.center[0], n.center[1]);
            ctx.beginPath();
            ctx.arc(c[0], c[1], n.radius * sx, 0, 2 * Math.PI);
            ctx.fill(); ctx.stroke();
        } else if (n.shape === "rectangle") {
            var x0 = Math.min(n.corners[0][0], n.corners[1][0]);
            var y0 = Math.min(n.corners[0][1], n.corners[1][1]);
            var x1 = Math.max(n.corners[0][0], n.corners[1][0]);
            var y1 = Math.max(n.corners[0][1], n.corners[1][1]);
            var a = worldToPx(x0, y1), b = worldToPx(x1, y0);
            ctx.fillRect(a[0], a[1], b[0] - a[0], b[1] - a[1]);
            ctx.strokeRect(a[0], a[1], b[0] - a[0], b[1] - a[1]);
        }
    }

    function drawRoute(path, col) {
        if (path.length < 2) return;
        ctx.strokeStyle = col;
        ctx.lineWidth = 1.4;
        ctx.globalAlpha = 0.55;
        ctx.beginPath();
        var p0 = worldToPx(path[0].x, path[0].y);
        ctx.moveTo(p0[0], p0[1]);
        for (var i = 1; i < path.length; i++) {
            var p = worldToPx(path[i].x, path[i].y);
            ctx.lineTo(p[0], p[1]);
        }
        ctx.stroke();
        ctx.globalAlpha = 1.0;
        ctx.lineWidth = 1;
    }

    /* ---- HUD readouts ------------------------------------------------- */
    function updateHud(parts) {
        document.getElementById("hudT").textContent = simT.toFixed(2);
        document.getElementById("clockReadout").textContent = "t = " + simT.toFixed(2);

        var hud = document.getElementById("hud");
        /* keep the first two static spans, rebuild the per-drone spans */
        var html = '<span>SIM CLOCK: <b id="hudT">' + simT.toFixed(2) +
                   '</b></span><span>MAKESPAN: <b id="hudMax">' +
                   makespan.toFixed(2) + '</b></span>';
        parts.forEach(function (p) {
            html += '<span style="color:' + p.col + '">' + p.id +
                    ' BAT:' + p.battery.toFixed(0) +
                    ' PL:' + p.payload.toFixed(2) + '</span>';
        });
        hud.innerHTML = html;
    }

    function buildLegend() { /* legend is static in HTML; nothing dynamic needed */ }

    /* ---- animation loop ----------------------------------------------- */
    function loop() {
        rafId = requestAnimationFrame(loop);
        var now = performance.now();
        if (lastFrame === null) lastFrame = now;
        var dt = (now - lastFrame) / 1000.0;
        lastFrame = now;

        if (playing) {
            simT += dt * speed;
            if (simT >= makespan) {
                simT = makespan;
                playing = false;
                document.getElementById("playBtn").value = "Replay";
            }
        }
        draw();
    }

    /* ---- public controls ---------------------------------------------- */
    function togglePlay() {
        if (!report) return;
        if (simT >= makespan) { simT = 0; }   /* replay from start */
        playing = !playing;
        document.getElementById("playBtn").value = playing ? "Pause" : "Resume";
        lastFrame = null;
    }
    function restart() {
        if (!report) return;
        simT = 0; playing = true; lastFrame = null;
        document.getElementById("playBtn").value = "Pause";
    }
    function setSpeed(v) { speed = v; }

    return {
        init: init, togglePlay: togglePlay, restart: restart, setSpeed: setSpeed
    };
})();

/* ---- wire up canvas controls (present on the simulation page) --------- */
document.addEventListener("DOMContentLoaded", function () {
    var playBtn = document.getElementById("playBtn");
    var restartBtn = document.getElementById("restartBtn");
    var speed = document.getElementById("speed");
    if (playBtn) playBtn.onclick = function () { Vis.togglePlay(); };
    if (restartBtn) restartBtn.onclick = function () { Vis.restart(); };
    if (speed) speed.oninput = function () {
        Vis.setSpeed(parseInt(speed.value, 10));
        document.getElementById("speedVal").textContent = speed.value + "x";
    };
});
