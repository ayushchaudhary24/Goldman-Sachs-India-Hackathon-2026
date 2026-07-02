/* ==========================================================================
   simulation.js
   --------------------------------------------------------------------------
   Orchestrates a simulation run:
     1. gathers the input (from the JSON box if present, else the form),
     2. POSTs it to /api/simulate,
     3. renders alerts, the Flight Manifest, the scrolling log, the statistics,
     4. hands the analysed report to the canvas visualiser.

   All displayed values come from the server's response, which is derived from
   the solver's manifest.  Nothing here fabricates data.
   ========================================================================== */

/* ---- old-style alert boxes ------------------------------------------- */
function showAlert(kind, messages) {
    var area = document.getElementById("alertArea");
    if (!area) { alert(messages.join("\n")); return; }
    var cls = "info";
    if (kind === "error") cls = "error";
    else if (kind === "ok") cls = "ok";
    else if (kind === "warn") cls = "warn";

    var html = '<div class="msgbox ' + cls + '">';
    if (messages.length === 1) {
        html += messages[0];
    } else {
        html += "<strong>Please review the following:</strong><ul>";
        messages.forEach(function (m) { html += "<li>" + escapeHtml(m) + "</li>"; });
        html += "</ul>";
    }
    html += "</div>";
    area.innerHTML = html;
    area.scrollIntoView({ behavior: "smooth", block: "nearest" });
}

function escapeHtml(s) {
    return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;")
                    .replace(/>/g, "&gt;");
}

/* ---- determine the input object to send ------------------------------ */
function gatherInput() {
    /* If the JSON box has content, that is authoritative (covers upload). */
    var txt = document.getElementById("jsonText").value.trim();
    if (txt) {
        try {
            return { data: JSON.parse(txt), error: null };
        } catch (e) {
            return { data: null, error: "Malformed JSON in the input box: " + e.message };
        }
    }
    /* otherwise assemble from the form */
    return { data: collectInput(), error: null };
}

/* ---- run ------------------------------------------------------------- */
function runSimulation() {
    var area = document.getElementById("alertArea");
    if (area) area.innerHTML = "";

    var got = gatherInput();
    if (got.error) { showAlert("error", [got.error]); return; }

    var status = document.getElementById("runStatus");
    status.textContent = "  Running routing engine, please wait...";
    document.getElementById("runBtn").disabled = true;

    fetch("/api/simulate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(got.data)
    }).then(function (resp) {
        return resp.json().then(function (j) { return { ok: resp.ok, body: j }; });
    }).then(function (r) {
        status.textContent = "";
        document.getElementById("runBtn").disabled = false;
        if (!r.body.ok) {
            showAlert("error", r.body.errors || ["Unknown error."]);
            return;
        }
        var report = r.body.report;
        showAlert("ok", ["Simulation completed. Raw Score: " +
                          report.stats.raw_score.toFixed(2) +
                          "  |  Successful Deliveries: " +
                          report.stats.successful_deliveries + " / " +
                          report.stats.total_requested + "."]);
        renderManifest(report);
        renderLog(report);
        renderStats(report);
        Vis.init(report);
    }).catch(function (err) {
        status.textContent = "";
        document.getElementById("runBtn").disabled = false;
        showAlert("error", ["Network or server error: " + err.message]);
    });
}

/* ---- Flight Manifest table ------------------------------------------- */
function renderManifest(report) {
    var html = "";
    report.drones.forEach(function (dr, idx) {
        html += '<table class="grid"><caption>Drone: ' +
                escapeHtml(dr.drone_id) + '</caption>';
        html += "<tr><th>#</th><th>Time</th><th>Action</th><th>X</th><th>Y</th>" +
                "<th>Delivery</th><th>Battery</th><th>Payload</th>" +
                "<th>Wait</th><th>Charge</th></tr>";
        dr.path.forEach(function (s, i) {
            var dlv = s.delivery_id ? s.delivery_id :
                      (s.delivery_ids ? s.delivery_ids.join(", ") : "");
            html += "<tr" + (i % 2 ? ' class="alt"' : "") + ">";
            html += "<td>" + (i + 1) + "</td>";
            html += '<td class="num mono">' + s.t.toFixed(2) + "</td>";
            html += "<td>" + s.action + "</td>";
            html += '<td class="num">' + s.x + "</td>";
            html += '<td class="num">' + s.y + "</td>";
            html += "<td>" + escapeHtml(dlv) + "</td>";
            html += '<td class="num">' + s.battery.toFixed(1) + "</td>";
            html += '<td class="num">' + s.payload.toFixed(2) + "</td>";
            html += '<td class="num">' + (s.wait ? s.wait.toFixed(1) : "") + "</td>";
            html += '<td class="num">' + (s.charge ? s.charge.toFixed(1) : "") + "</td>";
            html += "</tr>";
        });
        html += "</table>";
    });
    if (!report.drones.length) {
        html = '<div class="msgbox warn">The manifest is empty &mdash; ' +
               'no feasible deliveries were scheduled for these inputs.</div>';
    }
    document.getElementById("manifestArea").innerHTML = html;
}

/* ---- scrolling simulation log ---------------------------------------- */
function renderLog(report) {
    var body = document.getElementById("logBody");
    var html = "";
    report.sim_log.forEach(function (e) {
        html += "<tr><td class='mono'>" + e.t.toFixed(2) + "</td>" +
                "<td>" + escapeHtml(e.drone) + "</td>" +
                "<td><b>" + e.action + "</b> &ndash; " +
                escapeHtml(e.status) + "</td></tr>";
    });
    if (!html) html = '<tr><td colspan="3" class="small-muted">No log entries.</td></tr>';
    body.innerHTML = html;
}

/* ---- statistics & performance summary -------------------------------- */
function renderStats(report) {
    var s = report.stats;
    var html = "";

    /* top-line big numbers */
    html += '<table class="statgrid"><tr>';
    html += statCell(s.raw_score.toFixed(2), "Raw Score");
    html += statCell(s.successful_deliveries + " / " + s.total_requested,
                     "Successful Deliveries");
    html += statCell(s.success_rate.toFixed(1) + " %", "Success Rate");
    html += statCell(s.makespan.toFixed(2), "Makespan");
    html += "</tr></table>";

    /* detailed performance summary */
    html += '<table class="grid" style="margin-top:10px"><caption>Performance Summary</caption>';
    html += "<tr><th style='width:50%'>Metric</th><th>Value</th></tr>";
    html += metricRow("Successful Deliveries", s.successful_deliveries, false);
    html += metricRow("Missed Deliveries", s.missed_deliveries, true);
    html += metricRow("Total Energy", s.total_energy.toFixed(2), false);
    html += metricRow("Makespan", s.makespan.toFixed(2), true);
    html += metricRow("Raw Score", s.raw_score.toFixed(2), false);
    html += metricRow("Drone Utilisation", s.drone_utilisation.toFixed(1) + " %  (" +
                      s.drones_used + " of " + s.drones_total + " used)", true);
    html += metricRow("Total Waiting Time", s.total_wait.toFixed(2), false);
    html += metricRow("Total Charging Time", s.total_charge.toFixed(2), true);
    if (s.late_deliveries && s.late_deliveries.length) {
        html += metricRow("Late (not counted)", s.late_deliveries.join(", "), false);
    }
    html += "</table>";

    /* per-drone breakdown */
    html += '<table class="grid" style="margin-top:10px"><caption>Per-Drone Breakdown</caption>';
    html += "<tr><th>Drone</th><th>Deliveries</th><th>Energy</th>" +
            "<th>Wait</th><th>Charge</th><th>Finish (t)</th></tr>";
    report.drones.forEach(function (dr, i) {
        html += "<tr" + (i % 2 ? ' class="alt"' : "") + "><td>" +
                escapeHtml(dr.drone_id) + "</td>" +
                "<td class='num'>" + dr.deliveries + "</td>" +
                "<td class='num'>" + dr.energy.toFixed(2) + "</td>" +
                "<td class='num'>" + dr.wait_time.toFixed(2) + "</td>" +
                "<td class='num'>" + dr.charge_time.toFixed(2) + "</td>" +
                "<td class='num'>" + dr.final_t.toFixed(2) + "</td></tr>";
    });
    html += "</table>";

    /* scoring formula breakdown (transparency) */
    html += '<div class="msgbox info mono">raw_score = (' +
            s.successful_deliveries + ' \u00D7 100) \u2212 (' +
            s.total_energy.toFixed(2) + ' \u00D7 0.1) \u2212 (' +
            s.makespan.toFixed(2) + ' \u00D7 0.05) = ' +
            s.raw_score.toFixed(2) + '</div>';

    document.getElementById("statsArea").innerHTML = html;
}

function statCell(big, cap) {
    return '<td><div class="big">' + big + '</div>' +
           '<div class="cap">' + cap + '</div></td>';
}
function metricRow(label, value, alt) {
    return "<tr" + (alt ? ' class="alt"' : "") + "><td>" + label +
           "</td><td class='mono'>" + value + "</td></tr>";
}

/* ---- reset ----------------------------------------------------------- */
function resetAll() {
    if (!confirm("Reset all inputs and outputs?")) return;
    document.getElementById("jsonText").value = "";
    document.getElementById("manifestArea").innerHTML =
        '<p class="small-muted">Run a simulation to populate the Flight Manifest.</p>';
    document.getElementById("statsArea").innerHTML =
        '<p class="small-muted">Run a simulation to populate statistics.</p>';
    document.getElementById("logBody").innerHTML =
        '<tr><td colspan="3" class="small-muted">No simulation run yet.</td></tr>';
    var area = document.getElementById("alertArea");
    if (area) area.innerHTML = "";
    applyInput(SAMPLE0);  /* restore default seed input */
}

/* ---- wire up --------------------------------------------------------- */
document.addEventListener("DOMContentLoaded", function () {
    var runBtn = document.getElementById("runBtn");
    var resetBtn = document.getElementById("resetBtn");
    if (runBtn) runBtn.onclick = runSimulation;
    if (resetBtn) resetBtn.onclick = resetAll;
});
