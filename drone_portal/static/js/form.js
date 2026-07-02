/* ==========================================================================
   form.js
   --------------------------------------------------------------------------
   Builds and manages the dynamic input form: add/remove rows for drones,
   deliveries, charging stations and No-Fly Zones; assembles the input JSON
   from the form; and applies an uploaded/pasted JSON back into the form.
   No routing happens here -- this only prepares the input object.
   ========================================================================== */

/* ---- small helpers ---------------------------------------------------- */
function el(tag, attrs, html) {
    var e = document.createElement(tag);
    if (attrs) { for (var k in attrs) e.setAttribute(k, attrs[k]); }
    if (html !== undefined) e.innerHTML = html;
    return e;
}
function numCell(value, w) {
    var td = el("td");
    var inp = el("input", { type: "number", value: (value === undefined ? "" : value) });
    if (w) inp.style.width = w;
    td.appendChild(inp);
    return td;
}
function textCell(value) {
    var td = el("td");
    var inp = el("input", { type: "text", value: (value === undefined ? "" : value) });
    td.appendChild(inp);
    return td;
}
function delBtnCell(row) {
    var td = el("td");
    var b = el("input", { type: "button", value: "Remove", "class": "btn small danger" });
    b.onclick = function () { row.parentNode.removeChild(row); };
    td.appendChild(b);
    return td;
}

/* ---- DRONES ----------------------------------------------------------- */
function addDroneRow(id, payload) {
    var tbl = document.getElementById("droneTable");
    var row = el("tr");
    row.appendChild(textCell(id !== undefined ? id : "drone_" + tbl.rows.length));
    row.appendChild(numCell(payload !== undefined ? payload : 1.0));
    row.appendChild(delBtnCell(row));
    tbl.appendChild(row);
}

/* ---- DELIVERIES ------------------------------------------------------- */
function addDeliveryRow(d) {
    d = d || {};
    var tbl = document.getElementById("deliveryTable");
    var row = el("tr");
    row.appendChild(textCell(d.id !== undefined ? d.id : "d" + tbl.rows.length));
    row.appendChild(numCell(d.x !== undefined ? d.x : 0));
    row.appendChild(numCell(d.y !== undefined ? d.y : 0));
    row.appendChild(numCell(d.weight !== undefined ? d.weight : 0.3));
    row.appendChild(numCell(d.deadline !== undefined ? d.deadline : 200));
    row.appendChild(delBtnCell(row));
    tbl.appendChild(row);
}

/* ---- CHARGING STATIONS ------------------------------------------------ */
function addStationRow(s) {
    s = s || {};
    var tbl = document.getElementById("stationTable");
    var row = el("tr");
    row.appendChild(numCell(s.x !== undefined ? s.x : 50));
    row.appendChild(numCell(s.y !== undefined ? s.y : 50));
    row.appendChild(numCell(s.slots !== undefined ? s.slots : 1));
    row.appendChild(delBtnCell(row));
    tbl.appendChild(row);
}

/* ---- NO-FLY ZONES ----------------------------------------------------- */
function addNfzRow(n) {
    n = n || {};
    var tbl = document.getElementById("nfzTable");
    var row = el("tr");

    /* shape selector */
    var tdShape = el("td");
    var sel = el("select");
    sel.appendChild(el("option", { value: "circle" }, "circle"));
    sel.appendChild(el("option", { value: "rectangle" }, "rectangle"));
    sel.value = n.shape || "circle";
    tdShape.appendChild(sel);
    row.appendChild(tdShape);

    /* circle fields: cx, cy, radius */
    var tdCircle = el("td");
    var cx = el("input", { type: "number" }); cx.style.width = "50px";
    var cy = el("input", { type: "number" }); cy.style.width = "50px";
    var rad = el("input", { type: "number" }); rad.style.width = "50px";
    if (n.shape === "circle" && n.center) { cx.value = n.center[0]; cy.value = n.center[1]; rad.value = n.radius; }
    tdCircle.appendChild(document.createTextNode("C("));
    tdCircle.appendChild(cx);
    tdCircle.appendChild(document.createTextNode(","));
    tdCircle.appendChild(cy);
    tdCircle.appendChild(document.createTextNode(") r="));
    tdCircle.appendChild(rad);
    row.appendChild(tdCircle);

    /* rectangle fields: x0,y0 x1,y1 */
    var tdRect = el("td");
    var x0 = el("input", { type: "number" }); x0.style.width = "45px";
    var y0 = el("input", { type: "number" }); y0.style.width = "45px";
    var x1 = el("input", { type: "number" }); x1.style.width = "45px";
    var y1 = el("input", { type: "number" }); y1.style.width = "45px";
    if (n.shape === "rectangle" && n.corners) {
        x0.value = n.corners[0][0]; y0.value = n.corners[0][1];
        x1.value = n.corners[1][0]; y1.value = n.corners[1][1];
    }
    tdRect.appendChild(document.createTextNode("("));
    tdRect.appendChild(x0); tdRect.appendChild(document.createTextNode(","));
    tdRect.appendChild(y0); tdRect.appendChild(document.createTextNode(")-("));
    tdRect.appendChild(x1); tdRect.appendChild(document.createTextNode(","));
    tdRect.appendChild(y1); tdRect.appendChild(document.createTextNode(")"));
    row.appendChild(tdRect);

    row.appendChild(numCell(n.T_start !== undefined ? n.T_start : 0, "50px"));
    row.appendChild(numCell(n.T_end !== undefined ? n.T_end : 150, "50px"));
    row.appendChild(delBtnCell(row));
    tbl.appendChild(row);
}

/* ---- collect the form into an input object ---------------------------- */
function collectInput() {
    var data = {};
    data.map_size = [
        parseFloat(document.getElementById("mapW").value),
        parseFloat(document.getElementById("mapH").value)
    ];

    /* drones (skip header row 0) */
    data.drones = [];
    var dt = document.getElementById("droneTable").rows;
    for (var i = 1; i < dt.length; i++) {
        var c = dt[i].getElementsByTagName("input");
        if (!c.length) continue;
        data.drones.push({
            id: c[0].value.trim(),
            max_payload: parseFloat(c[1].value)
        });
    }

    /* deliveries */
    data.deliveries = [];
    var vt = document.getElementById("deliveryTable").rows;
    for (i = 1; i < vt.length; i++) {
        var v = vt[i].getElementsByTagName("input");
        if (!v.length) continue;
        data.deliveries.push({
            id: v[0].value.trim(),
            x: parseFloat(v[1].value),
            y: parseFloat(v[2].value),
            weight: parseFloat(v[3].value),
            deadline: parseFloat(v[4].value)
        });
    }

    /* charging stations */
    data.charging_stations = [];
    var st = document.getElementById("stationTable").rows;
    for (i = 1; i < st.length; i++) {
        var s = st[i].getElementsByTagName("input");
        if (!s.length) continue;
        var station = { x: parseFloat(s[0].value), y: parseFloat(s[1].value) };
        var slots = parseInt(s[2].value, 10);
        if (!isNaN(slots)) station.slots = slots;
        data.charging_stations.push(station);
    }

    /* no-fly zones */
    data.no_fly_zones = [];
    var nt = document.getElementById("nfzTable").rows;
    for (i = 1; i < nt.length; i++) {
        var row = nt[i];
        var sel = row.getElementsByTagName("select")[0];
        if (!sel) continue;
        var inp = row.getElementsByTagName("input");
        /* layout of inputs: cx,cy,rad,x0,y0,x1,y1,Tstart,Tend */
        var shape = sel.value;
        var nfz = { shape: shape };
        if (shape === "circle") {
            nfz.center = [parseFloat(inp[0].value), parseFloat(inp[1].value)];
            nfz.radius = parseFloat(inp[2].value);
        } else {
            nfz.corners = [
                [parseFloat(inp[3].value), parseFloat(inp[4].value)],
                [parseFloat(inp[5].value), parseFloat(inp[6].value)]
            ];
        }
        nfz.T_start = parseFloat(inp[7].value);
        nfz.T_end = parseFloat(inp[8].value);
        data.no_fly_zones.push(nfz);
    }

    return data;
}

/* ---- clear and repopulate the form from a JSON object ----------------- */
function applyInput(data) {
    /* map */
    if (data.map_size) {
        document.getElementById("mapW").value = data.map_size[0];
        document.getElementById("mapH").value = data.map_size[1];
        updateWarehouseInfo();
    }
    /* clear all dynamic tables back to header rows */
    ["droneTable", "deliveryTable", "stationTable", "nfzTable"].forEach(function (id) {
        var t = document.getElementById(id);
        while (t.rows.length > 1) t.deleteRow(1);
    });
    (data.drones || []).forEach(function (d) { addDroneRow(d.id, d.max_payload); });
    (data.deliveries || []).forEach(function (d) { addDeliveryRow(d); });
    (data.charging_stations || []).forEach(function (s) { addStationRow(s); });
    (data.no_fly_zones || []).forEach(function (n) { addNfzRow(n); });
}

/* ---- warehouse readout ------------------------------------------------ */
function updateWarehouseInfo() {
    var w = parseFloat(document.getElementById("mapW").value);
    var h = parseFloat(document.getElementById("mapH").value);
    var info = document.getElementById("whInfo");
    if (!isNaN(w) && !isNaN(h)) {
        info.textContent = "(" + (w / 2) + ", " + (h / 2) + ") \u2014 auto, centre of map";
    }
}

/* ---- sample inputs (from the problem statement) ----------------------- */
var SAMPLE0 = {
    "map_size": [100, 100],
    "drones": [{ "id": "drone_1", "max_payload": 1.0 }],
    "deliveries": [
        { "id": "d1", "x": 70, "y": 60, "weight": 0.3, "deadline": 200 },
        { "id": "d2", "x": 30, "y": 80, "weight": 0.4, "deadline": 200 }
    ],
    "charging_stations": [],
    "no_fly_zones": []
};
var SAMPLE1 = {
    "map_size": [200, 200],
    "drones": [{ "id": "drone_1", "max_payload": 1.0 }],
    "deliveries": [
        { "id": "d1", "x": 10, "y": 100, "weight": 0.3, "deadline": 200.0 },
        { "id": "d2", "x": 10, "y": 10, "weight": 0.3, "deadline": 350.0 },
        { "id": "d3", "x": 100, "y": 10, "weight": 0.3, "deadline": 500.0 }
    ],
    "charging_stations": [{ "x": 100, "y": 10, "slots": 1 }],
    "no_fly_zones": [
        { "shape": "circle", "center": [100, 55], "radius": 15,
          "T_start": 0.0, "T_end": 150.0 }
    ]
};

/* ---- wire up the form ------------------------------------------------- */
document.addEventListener("DOMContentLoaded", function () {
    document.getElementById("addDrone").onclick = function () { addDroneRow(); };
    document.getElementById("addDelivery").onclick = function () { addDeliveryRow(); };
    document.getElementById("addStation").onclick = function () { addStationRow(); };
    document.getElementById("addNfz").onclick = function () { addNfzRow(); };

    document.getElementById("mapW").onchange = updateWarehouseInfo;
    document.getElementById("mapH").onchange = updateWarehouseInfo;

    /* Generate JSON from form into the textarea */
    document.getElementById("genJson").onclick = function () {
        var data = collectInput();
        document.getElementById("jsonText").value = JSON.stringify(data, null, 2);
    };

    /* Apply the JSON text (typed or uploaded) into the form */
    document.getElementById("loadFromText").onclick = function () {
        var txt = document.getElementById("jsonText").value.trim();
        if (!txt) { showAlert("error", ["The JSON textarea is empty."]); return; }
        try {
            var data = JSON.parse(txt);
            applyInput(data);
            showAlert("ok", ["JSON applied to the form successfully."]);
        } catch (e) {
            showAlert("error", ["Malformed JSON: " + e.message]);
        }
    };

    /* file upload -> textarea */
    document.getElementById("jsonFile").onchange = function (ev) {
        var f = ev.target.files[0];
        if (!f) return;
        var reader = new FileReader();
        reader.onload = function () {
            document.getElementById("jsonText").value = reader.result;
            showAlert("info", ["File loaded into the JSON box. Press 'Apply JSON to Form' to populate the form, or 'Run Simulation' to run it directly."]);
        };
        reader.readAsText(f);
    };

    /* sample loaders */
    document.getElementById("loadSample0").onclick = function (e) {
        e.preventDefault(); applyInput(SAMPLE0);
        document.getElementById("jsonText").value = JSON.stringify(SAMPLE0, null, 2);
    };
    document.getElementById("loadSample1").onclick = function (e) {
        e.preventDefault(); applyInput(SAMPLE1);
        document.getElementById("jsonText").value = JSON.stringify(SAMPLE1, null, 2);
    };

    /* seed the form with Sample 0 so it is never empty */
    applyInput(SAMPLE0);
});
