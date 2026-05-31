/**
 * Control panel — sends commands to POST /command.
 *
 * All single-robot navigation uses navigate_to_goal (A* + pure pursuit).
 * Test paths (straight / turnaround / L) still use geometric waypoints for
 * quick debugging without needing a full A* plan.
 */

const DEFAULT_DRIVE_SPEED  = 200;
const DEFAULT_TURN_SPEED   = 150;
const DEFAULT_TEST_DIST_CM = 30;
const ARENA_CM = 400;

let _robotStates = {};
let _pathIdCounter = Math.floor(Date.now() / 1000);

// DOM refs
let _robotSelect     = null;
let _robotOneSel     = null;
let _robotTwoSel     = null;
let _goalOneRowEl    = null;
let _goalOneColEl    = null;
let _goalTwoRowEl    = null;
let _goalTwoColEl    = null;
let _waypointXEl     = null;
let _waypointYEl     = null;
let _statusEl        = null;
let _tagTableBody    = null;

// Tag registry (mirrors server state, updated from snapshot)
const _tags = {};  // tag_id -> {tag_id, role, row, col, x_cm, y_cm}

export function init(elements) {
    _robotSelect  = elements.robotSelect;
    _robotOneSel  = elements.robotOneSel;
    _robotTwoSel  = elements.robotTwoSel;
    _goalOneRowEl = elements.goalOneRowEl;
    _goalOneColEl = elements.goalOneColEl;
    _goalTwoRowEl = elements.goalTwoRowEl;
    _goalTwoColEl = elements.goalTwoColEl;
    _waypointXEl  = elements.waypointXEl;
    _waypointYEl  = elements.waypointYEl;
    _statusEl     = elements.statusEl;
    _tagTableBody = elements.tagTableBody;

    elements.btnPause.addEventListener("click",       sendPause);
    elements.btnResume.addEventListener("click",      sendResume);
    elements.btnStop.addEventListener("click",        sendStop);
    elements.btnGripper.addEventListener("click",     sendToggleGripper);
    elements.btnStraight.addEventListener("click",    sendStraightTest);
    elements.btnTurnaround.addEventListener("click",  sendTurnaroundTest);
    elements.btnLPath.addEventListener("click",       sendLTest);
    elements.btnGoWaypoint.addEventListener("click",  sendNavigateToGoal);
    elements.btnCoordTraverse.addEventListener("click", sendCoordTraverse);

    elements.btnAddTag?.addEventListener("click", _handleAddTag);
}

export function updateRobots(robots) {
    _robotStates = robots;
    const ids = Object.keys(robots).sort();
    _updateSelect(_robotSelect, ids);
    _updateSelect(_robotOneSel, ids, 0);
    _updateSelect(_robotTwoSel, ids, 1);
}

export function updateTagRegistry(obstacles, goals) {
    // Merge server-side obstacle/goal lists into local map
    for (const t of obstacles) _tags[t.tag_id] = { ...t, role: "obstacle" };
    for (const t of goals)     _tags[t.tag_id] = { ...t, role: "goal" };
    _renderTagTable();
}

/** Called by arena.js click-to-goal — populates the waypoint fields. */
export function setWaypointFromClick(x_cm, y_cm) {
    if (_waypointXEl) _waypointXEl.value = x_cm.toFixed(1);
    if (_waypointYEl) _waypointYEl.value = y_cm.toFixed(1);
    _status(`Click → (${x_cm.toFixed(1)}, ${y_cm.toFixed(1)}) — click Navigate to dispatch`);
}

// ------------------------------------------------------------------
// Navigation (A* + pure pursuit)
// ------------------------------------------------------------------

export function sendNavigateToGoal() {
    const id = _selectedRobot();
    if (!id) return _status("Select a robot first");
    const x = parseFloat(_waypointXEl?.value);
    const y = parseFloat(_waypointYEl?.value);
    if (isNaN(x) || isNaN(y)) return _status("Enter valid x and y coordinates (cm)");
    if (x < 0 || x > ARENA_CM || y < 0 || y > ARENA_CM)
        return _status(`Coordinates must be within 0–${ARENA_CM} cm`);

    _send({
        type: "navigate_to_goal",
        robot_id: id,
        goal_x_cm: x,
        goal_y_cm: y,
        motion: { turn_speed_deg_per_sec: DEFAULT_TURN_SPEED, drive_speed_deg_per_sec: DEFAULT_DRIVE_SPEED },
    });
}

// ------------------------------------------------------------------
// Simple motion controls
// ------------------------------------------------------------------

export function sendPause() {
    const id = _selectedRobot();
    if (!id) return _status("Select a robot first");
    _send({ type: "pause", robot_id: id, reason: "web_pause_button" });
}

export function sendResume() {
    const id = _selectedRobot();
    if (!id) return _status("Select a robot first");
    _send({ type: "resume", robot_id: id });
}

export function sendStop() {
    const id = _selectedRobot();
    if (!id) return _status("Select a robot first");
    _send({ type: "stop", robot_id: id, reason: "web_stop_button" });
}

export function sendToggleGripper() {
    const id = _selectedRobot();
    if (!id) return _status("Select a robot first");
    _send({ type: "toggle_gripper", robot_id: id });
}

// ------------------------------------------------------------------
// Test paths (geometric — bypass A* for quick debug moves)
// ------------------------------------------------------------------

export function sendStraightTest() {
    const { id, x, y, theta_rad } = _selectedPose() ?? {};
    if (!id) return;
    const d = DEFAULT_TEST_DIST_CM;
    _sendPath(id, [{
        x_cm: _clip(x + d * Math.cos(theta_rad)),
        y_cm: _clip(y + d * Math.sin(theta_rad)),
    }]);
}

export function sendTurnaroundTest() {
    const { id, x, y, theta_rad } = _selectedPose() ?? {};
    if (!id) return;
    const d = DEFAULT_TEST_DIST_CM;
    _sendPath(id, [{
        x_cm: _clip(x - d * Math.cos(theta_rad)),
        y_cm: _clip(y - d * Math.sin(theta_rad)),
    }]);
}

export function sendLTest() {
    const { id, x, y } = _selectedPose() ?? {};
    if (!id) return;
    _sendPath(id, [
        { x_cm: _clip(x + 20), y_cm: _clip(y) },
        { x_cm: _clip(x + 40), y_cm: _clip(y) },
        { x_cm: _clip(x + 40), y_cm: _clip(y + 40) },
    ]);
}

// ------------------------------------------------------------------
// Coordinated two-robot traverse (uses existing A* arbiter logic)
// ------------------------------------------------------------------

export function sendCoordTraverse() {
    const r1 = _robotOneSel?.value;
    const r2 = _robotTwoSel?.value;
    if (!r1 || !r2 || r1 === r2) return _status("Select two different robots");

    const row1 = parseInt(_goalOneRowEl?.value);
    const col1 = parseInt(_goalOneColEl?.value);
    const row2 = parseInt(_goalTwoRowEl?.value);
    const col2 = parseInt(_goalTwoColEl?.value);

    for (const v of [row1, col1, row2, col2]) {
        if (isNaN(v) || v < 0 || v > 39) return _status("Goal cells must be integers 0–39");
    }
    if (row1 === row2 && col1 === col2) return _status("Goal cells must be different");

    _send({
        type: "coordinated_traverse",
        robots: [
            { robot_id: r1, goal_row: row1, goal_col: col1 },
            { robot_id: r2, goal_row: row2, goal_col: col2 },
        ],
    });
    _status(`Coordinated: ${r1}→(${row1},${col1}), ${r2}→(${row2},${col2})`);
}

// ------------------------------------------------------------------
// AprilTag role management
// ------------------------------------------------------------------

function _handleAddTag() {
    const tagIdEl = document.getElementById("tag-id-input");
    const roleEl  = document.getElementById("tag-role-input");
    if (!tagIdEl || !roleEl) return;

    const tagId = parseInt(tagIdEl.value);
    const role  = roleEl.value;
    if (isNaN(tagId) || tagId < 0) return _status("Enter a valid tag ID (≥ 0)");

    // Optimistic local update
    if (role === "none") {
        delete _tags[tagId];
    } else {
        _tags[tagId] = { tag_id: tagId, role, row: null, col: null, x_cm: null, y_cm: null };
    }
    _renderTagTable();

    _send({ type: "set_tag_role", tag_id: tagId, role });
    tagIdEl.value = "";
}

export function removeTag(tagId) {
    delete _tags[tagId];
    _renderTagTable();
    _send({ type: "set_tag_role", tag_id: tagId, role: "none" });
}

function _renderTagTable() {
    if (!_tagTableBody) return;
    _tagTableBody.innerHTML = "";
    const entries = Object.values(_tags).sort((a, b) => a.tag_id - b.tag_id);
    if (!entries.length) {
        const tr = document.createElement("tr");
        tr.innerHTML = `<td colspan="4" style="color:#4b5563;text-align:center">No tags registered</td>`;
        _tagTableBody.appendChild(tr);
        return;
    }
    for (const t of entries) {
        const tr = document.createElement("tr");
        const posStr = t.x_cm != null
            ? `(${(+t.x_cm).toFixed(0)}, ${(+t.y_cm).toFixed(0)})`
            : t.row != null ? `cell(${t.row},${t.col})` : "—";
        const roleColor = t.role === "obstacle" ? "#fca5a5" : "#6ee7b7";
        tr.innerHTML = `
            <td>${t.tag_id}</td>
            <td style="color:${roleColor}">${t.role}</td>
            <td>${posStr}</td>
            <td><button onclick="window._removeTag(${t.tag_id})" style="padding:2px 6px;font-size:10px">×</button></td>
        `;
        _tagTableBody.appendChild(tr);
    }
    // Expose removeTag globally for the inline onclick
    window._removeTag = removeTag;
}

// ------------------------------------------------------------------
// Internal helpers
// ------------------------------------------------------------------

function _nextPathId() { return ++_pathIdCounter; }

function _selectedRobot() { return _robotSelect?.value || null; }

function _selectedPose() {
    const id = _selectedRobot();
    const state = _robotStates[id];
    if (!id || !state) { _status("Select a robot with telemetry first"); return null; }
    return {
        id,
        x: parseFloat(state.x_cm) || 0,
        y: parseFloat(state.y_cm) || 0,
        theta_rad: ((parseFloat(state.theta_deg) || 0) * Math.PI) / 180,
    };
}

function _clip(v) { return Math.max(0, Math.min(ARENA_CM, v)); }

function _sendPath(robotId, waypoints) {
    _send({
        type: "path_assignment",
        robot_id: robotId,
        path_id: _nextPathId(),
        replace_existing: true,
        waypoints,
        motion: { turn_speed_deg_per_sec: DEFAULT_TURN_SPEED, drive_speed_deg_per_sec: DEFAULT_DRIVE_SPEED },
    });
}

async function _send(msg) {
    try {
        const resp = await fetch("/command", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(msg),
        });
        const result = await resp.json();
        if (result.ok) {
            _status(`✓ ${msg.type}${msg.robot_id ? ` → ${msg.robot_id}` : ""}`);
        } else {
            _status(`✗ ${result.error}`);
        }
    } catch (e) {
        _status(`Network error: ${e.message}`);
    }
}

function _updateSelect(el, ids, preferIndex) {
    if (!el) return;
    const current = el.value;
    el.innerHTML = "";
    for (const id of ids) {
        const opt = document.createElement("option");
        opt.value = id;
        opt.textContent = id;
        el.appendChild(opt);
    }
    if (ids.includes(current))  { el.value = current; }
    else if (ids.length > 0)    { el.value = ids[preferIndex ?? 0] ?? ids[0]; }
}

function _status(msg) {
    if (_statusEl) _statusEl.textContent = msg;
}
