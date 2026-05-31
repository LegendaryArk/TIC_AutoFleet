/**
 * Arena canvas renderer.
 * Input: full snapshot object {robots, obstacles, goals} from ws_client.js
 *
 * Coordinate transform: real-world (x_cm, y_cm) → canvas pixels
 *   canvas_x = x_cm * scale
 *   canvas_y = (ARENA_CM - y_cm) * scale   (flip Y: world y=0 is canvas bottom)
 */

const ARENA_CM = 400;
const GRID_CELL_CM = 10;
const SAFETY_BOX_HALF_CM = 20;
const ARROW_LEN_CM = 10;
const ARROW_HEAD_CM = 4;
const TRAIL_MAX = 100;
const ECHO_WINDOW_S = 10;
const MIN_ULTRASONIC_CM = 1;
const MAX_ULTRASONIC_CM = 200;

// Sensor geometry (mirrors gui.py constants)
const FRONT_FWD_CM = 9.5;
const FRONT_LAT_CM = 0.0;
const FRONT_ANG_DEG = 0.0;
const LEFT_FWD_CM = 0.0;
const LEFT_LAT_CM = 16.0;
const LEFT_ANG_DEG = 90.0;

const ROBOT_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
    "#bcbd22", "#17becf",
];

let _canvas = null;
let _ctx    = null;
const _history  = {};   // per-robot trail + echo history
const _colorMap = {};
let   _colorIdx = 0;
let   _clickCb  = null;

export function init(canvasEl, onCanvasClick) {
    _canvas = canvasEl;
    _ctx    = canvasEl.getContext("2d");
    _clickCb = onCanvasClick || null;

    const ro = new ResizeObserver(() => _resize());
    ro.observe(_canvas.parentElement);
    _resize();

    _canvas.addEventListener("click", _onCanvasClick);
}

/** Main entry point — called by index.html on every WebSocket snapshot. */
export function render(snapshot) {
    if (!_canvas) return;
    const { robots = {}, obstacles = [], goals = [] } = snapshot;
    _updateHistory(robots);
    _draw(robots, obstacles, goals);
}

// ------------------------------------------------------------------
// Coordinate helpers
// ------------------------------------------------------------------

function _scale() { return _canvas.width / ARENA_CM; }

function _toCanvas(x_cm, y_cm) {
    const s = _scale();
    return [x_cm * s, (ARENA_CM - y_cm) * s];
}

function _fromCanvas(px, py) {
    const s = _scale();
    return [px / s, ARENA_CM - py / s];
}

// ------------------------------------------------------------------
// History (trail + echo clouds)
// ------------------------------------------------------------------

function _colorFor(robotId) {
    if (!(robotId in _colorMap)) {
        _colorMap[robotId] = ROBOT_COLORS[_colorIdx % ROBOT_COLORS.length];
        _colorIdx++;
    }
    return _colorMap[robotId];
}

function _ensureHistory(robotId) {
    if (!_history[robotId]) {
        _history[robotId] = {
            trail_x: [], trail_y: [],
            front_echo: [], left_echo: [],
            last_front_cm: null, last_left_cm: null,
        };
    }
    return _history[robotId];
}

function _updateHistory(robots) {
    const now = Date.now() / 1000;
    for (const [robotId, state] of Object.entries(robots)) {
        const hist  = _ensureHistory(robotId);
        const x     = state.x_cm;
        const y     = state.y_cm;
        const theta = state.theta_deg;

        if (x != null && y != null) {
            hist.trail_x.push(x);
            hist.trail_y.push(y);
            if (hist.trail_x.length > TRAIL_MAX) { hist.trail_x.shift(); hist.trail_y.shift(); }
        }

        if (x != null && y != null && theta != null) {
            const front = state.front_ultrasonic_cm;
            const left  = state.left_ultrasonic_cm;
            if (front != null) {
                hist.last_front_cm = front;
                const ep = _echoPoint(x, y, theta, front, FRONT_FWD_CM, FRONT_LAT_CM, FRONT_ANG_DEG);
                if (ep) hist.front_echo.push({ t: now, x: ep[0], y: ep[1] });
            }
            if (left != null) {
                hist.last_left_cm = left;
                const ep = _echoPoint(x, y, theta, left, LEFT_FWD_CM, LEFT_LAT_CM, LEFT_ANG_DEG);
                if (ep) hist.left_echo.push({ t: now, x: ep[0], y: ep[1] });
            }
        }

        const cutoff = now - ECHO_WINDOW_S;
        hist.front_echo = hist.front_echo.filter(p => p.t >= cutoff);
        hist.left_echo  = hist.left_echo.filter(p => p.t >= cutoff);
    }
}

// ------------------------------------------------------------------
// Sensor math
// ------------------------------------------------------------------

function _sensorWorldPos(x, y, theta_deg, fwd, lat) {
    const r = theta_deg * Math.PI / 180;
    return [x + fwd * Math.cos(r) - lat * Math.sin(r),
            y + fwd * Math.sin(r) + lat * Math.cos(r)];
}

function _echoPoint(x, y, theta_deg, dist, fwd, lat, ang_offset) {
    if (dist < MIN_ULTRASONIC_CM || dist > MAX_ULTRASONIC_CM) return null;
    const [sx, sy] = _sensorWorldPos(x, y, theta_deg, fwd, lat);
    const r = (theta_deg + ang_offset) * Math.PI / 180;
    return [sx + dist * Math.cos(r), sy + dist * Math.sin(r)];
}

// ------------------------------------------------------------------
// Top-level draw
// ------------------------------------------------------------------

function _resize() {
    const parent = _canvas.parentElement;
    const size = Math.min(parent.clientWidth, parent.clientHeight);
    _canvas.width  = size;
    _canvas.height = size;
}

function _draw(robots, obstacles, goals) {
    const ctx = _ctx;
    ctx.clearRect(0, 0, _canvas.width, _canvas.width);

    _drawGrid(ctx);
    _drawObstacleCells(ctx, obstacles);
    _drawGoalMarkers(ctx, goals);

    for (const [robotId, state] of Object.entries(robots)) {
        const hist  = _ensureHistory(robotId);
        const color = _colorFor(robotId);
        _drawNavPath(ctx, state.nav_path_waypoints ?? [], color);
        _drawRobot(ctx, robotId, state, hist, color);
    }
}

// ------------------------------------------------------------------
// Grid
// ------------------------------------------------------------------

function _drawGrid(ctx) {
    const s = _scale();
    const w = _canvas.width;
    ctx.save();

    ctx.strokeStyle = "#1e3a5f";
    ctx.lineWidth   = 0.5;
    for (let i = 0; i <= ARENA_CM; i += GRID_CELL_CM) {
        const p = i * s;
        ctx.beginPath(); ctx.moveTo(p, 0); ctx.lineTo(p, w); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, p); ctx.lineTo(w, p); ctx.stroke();
    }

    ctx.strokeStyle = "#2a4a7f";
    ctx.lineWidth   = 1.0;
    ctx.fillStyle   = "#4a6fa5";
    ctx.font        = `${Math.round(s * 7)}px sans-serif`;
    for (let i = 0; i <= ARENA_CM; i += 50) {
        const p = i * s;
        ctx.beginPath(); ctx.moveTo(p, 0); ctx.lineTo(p, w); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, p); ctx.lineTo(w, p); ctx.stroke();
        if (i > 0 && i < ARENA_CM) {
            ctx.fillText(`${i}`, p + 2, w - 2);
            ctx.fillText(`${i}`, 2, (ARENA_CM - i) * s - 2);
        }
    }

    ctx.strokeStyle = "#4a7fba";
    ctx.lineWidth   = 2;
    ctx.strokeRect(0, 0, w, w);
    ctx.restore();
}

// ------------------------------------------------------------------
// Obstacle cells (red tint over grid cell)
// ------------------------------------------------------------------

function _drawObstacleCells(ctx, obstacles) {
    if (!obstacles.length) return;
    const s = _scale();
    ctx.save();
    ctx.fillStyle   = "rgba(220, 38, 38, 0.35)";
    ctx.strokeStyle = "rgba(220, 38, 38, 0.8)";
    ctx.lineWidth   = 1;
    for (const obs of obstacles) {
        if (obs.col == null || obs.row == null) continue;
        const px = obs.col * GRID_CELL_CM * s;
        const py = (ARENA_CM - (obs.row + 1) * GRID_CELL_CM) * s;
        const sz = GRID_CELL_CM * s;
        ctx.fillRect(px, py, sz, sz);
        ctx.strokeRect(px, py, sz, sz);
        ctx.fillStyle = "rgba(220, 38, 38, 0.9)";
        ctx.font = `bold ${Math.round(s * 6)}px sans-serif`;
        ctx.fillText(`T${obs.tag_id}`, px + 2, py + sz - 2);
        ctx.fillStyle = "rgba(220, 38, 38, 0.35)";
    }
    ctx.restore();
}

// ------------------------------------------------------------------
// Goal markers (green diamond with tag ID)
// ------------------------------------------------------------------

function _drawGoalMarkers(ctx, goals) {
    if (!goals.length) return;
    const s = _scale();
    ctx.save();
    for (const g of goals) {
        let cx, cy;
        if (g.x_cm != null && g.y_cm != null) {
            [cx, cy] = _toCanvas(g.x_cm, g.y_cm);
        } else if (g.col != null && g.row != null) {
            const xc = g.col * GRID_CELL_CM + GRID_CELL_CM / 2;
            const yc = g.row * GRID_CELL_CM + GRID_CELL_CM / 2;
            [cx, cy] = _toCanvas(xc, yc);
        } else {
            continue;
        }

        const r = GRID_CELL_CM * s * 0.45;
        ctx.beginPath();
        ctx.moveTo(cx, cy - r);
        ctx.lineTo(cx + r, cy);
        ctx.lineTo(cx, cy + r);
        ctx.lineTo(cx - r, cy);
        ctx.closePath();
        ctx.fillStyle   = "rgba(5, 150, 105, 0.4)";
        ctx.strokeStyle = "#10b981";
        ctx.lineWidth   = 2;
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = "#6ee7b7";
        ctx.font = `bold ${Math.round(s * 7)}px sans-serif`;
        ctx.textAlign = "center";
        ctx.fillText(`G${g.tag_id}`, cx, cy + Math.round(s * 3));
        ctx.textAlign = "left";
    }
    ctx.restore();
}

// ------------------------------------------------------------------
// Planned A* nav path (dashed line + numbered circles)
// ------------------------------------------------------------------

function _drawNavPath(ctx, waypoints, color) {
    if (!waypoints || waypoints.length === 0) return;
    const s = _scale();
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth   = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.globalAlpha = 0.6;

    ctx.beginPath();
    for (let i = 0; i < waypoints.length; i++) {
        const [px, py] = _toCanvas(waypoints[i].x_cm, waypoints[i].y_cm);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // Waypoint number circles
    for (let i = 0; i < waypoints.length; i++) {
        const [px, py] = _toCanvas(waypoints[i].x_cm, waypoints[i].y_cm);
        ctx.beginPath();
        ctx.arc(px, py, 4, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.8;
        ctx.fill();
        ctx.fillStyle = "#000";
        ctx.font = `bold ${Math.round(s * 5)}px sans-serif`;
        ctx.textAlign = "center";
        ctx.fillText(i + 1, px, py + Math.round(s * 2));
        ctx.textAlign = "left";
    }
    ctx.restore();
}

// ------------------------------------------------------------------
// Robot glyph
// ------------------------------------------------------------------

function _drawRobot(ctx, robotId, state, hist, color) {
    const x     = state.x_cm;
    const y     = state.y_cm;
    const theta = state.theta_deg;
    if (x == null || y == null) return;

    // Trail
    if (hist.trail_x.length > 1) {
        ctx.save();
        ctx.strokeStyle  = color;
        ctx.lineWidth    = 1.5;
        ctx.globalAlpha  = 0.4;
        ctx.beginPath();
        const [tx0, ty0] = _toCanvas(hist.trail_x[0], hist.trail_y[0]);
        ctx.moveTo(tx0, ty0);
        for (let i = 1; i < hist.trail_x.length; i++) {
            const [tx, ty] = _toCanvas(hist.trail_x[i], hist.trail_y[i]);
            ctx.lineTo(tx, ty);
        }
        ctx.stroke();
        ctx.restore();
    }

    // Echo clouds
    _drawEchoCloud(ctx, hist.front_echo, color, 0.7);
    _drawEchoCloud(ctx, hist.left_echo,  color, 0.5);

    const [cx, cy] = _toCanvas(x, y);
    const s = _scale();

    if (theta == null) {
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        _drawLabel(ctx, robotId, cx, cy, color, s);
        return;
    }

    const theta_rad = theta * Math.PI / 180;

    // Ultrasonic ray (front only, dashed)
    if (hist.last_front_cm != null) {
        _drawRay(ctx, x, y, theta, hist.last_front_cm, FRONT_FWD_CM, FRONT_LAT_CM, FRONT_ANG_DEG, color);
    }

    // Rotated safety box
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-theta_rad);
    const half = SAFETY_BOX_HALF_CM * s;
    ctx.strokeStyle  = color;
    ctx.lineWidth    = 1.5;
    ctx.globalAlpha  = 0.75;
    ctx.strokeRect(-half, -half, half * 2, half * 2);
    ctx.restore();

    // Heading arrow
    const arrowLen = ARROW_LEN_CM * s;
    const headLen  = ARROW_HEAD_CM * s;
    const dx =  arrowLen * Math.cos(theta_rad);
    const dy = -arrowLen * Math.sin(theta_rad);
    const tip_x = cx + dx;
    const tip_y = cy + dy;
    const ang   = Math.atan2(dy, dx);
    const ha    = Math.PI / 6;

    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle   = color;
    ctx.lineWidth   = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tip_x, tip_y);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(tip_x, tip_y);
    ctx.lineTo(tip_x - headLen * Math.cos(ang - ha), tip_y - headLen * Math.sin(ang - ha));
    ctx.lineTo(tip_x - headLen * Math.cos(ang + ha), tip_y - headLen * Math.sin(ang + ha));
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // Robot centre dot
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();

    // Fused pose dot (vision-corrected), dashed circle
    const fx = state.fused_x, fy = state.fused_y;
    if (fx != null && fy != null) {
        const [fcx, fcy] = _toCanvas(fx, fy);
        ctx.save();
        ctx.beginPath();
        ctx.arc(fcx, fcy, 4, 0, Math.PI * 2);
        ctx.strokeStyle = color;
        ctx.lineWidth   = 2;
        ctx.setLineDash([2, 2]);
        ctx.stroke();
        ctx.restore();
    }

    _drawLabel(ctx, robotId, cx, cy, color, s);
}

function _drawLabel(ctx, label, cx, cy, color, s) {
    ctx.fillStyle = color;
    ctx.font = `bold ${Math.round(s * 8)}px sans-serif`;
    ctx.fillText(label, cx + 7, cy - 7);
}

function _drawEchoCloud(ctx, points, color, alpha) {
    if (!points.length) return;
    ctx.save();
    ctx.fillStyle   = color;
    ctx.globalAlpha = alpha;
    for (const p of points) {
        const [px, py] = _toCanvas(p.x, p.y);
        ctx.beginPath();
        ctx.arc(px, py, 2, 0, Math.PI * 2);
        ctx.fill();
    }
    ctx.restore();
}

function _drawRay(ctx, x, y, theta_deg, dist, fwd, lat, ang_offset, color) {
    if (dist < MIN_ULTRASONIC_CM || dist > MAX_ULTRASONIC_CM) return;
    const [sx, sy] = _sensorWorldPos(x, y, theta_deg, fwd, lat);
    const r  = (theta_deg + ang_offset) * Math.PI / 180;
    const ex = sx + dist * Math.cos(r);
    const ey = sy + dist * Math.sin(r);
    const [csx, csy] = _toCanvas(sx, sy);
    const [cex, cey] = _toCanvas(ex, ey);

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth   = 1;
    ctx.globalAlpha = 0.55;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(csx, csy);
    ctx.lineTo(cex, cey);
    ctx.stroke();
    ctx.restore();
}

// ------------------------------------------------------------------
// Click-to-goal
// ------------------------------------------------------------------

function _onCanvasClick(evt) {
    if (!_clickCb) return;
    const rect = _canvas.getBoundingClientRect();
    const px = (evt.clientX - rect.left) * (_canvas.width  / rect.width);
    const py = (evt.clientY - rect.top)  * (_canvas.height / rect.height);
    const [x_cm, y_cm] = _fromCanvas(px, py);
    _clickCb(
        Math.round(Math.max(0, Math.min(ARENA_CM, x_cm)) * 10) / 10,
        Math.round(Math.max(0, Math.min(ARENA_CM, y_cm)) * 10) / 10,
    );
}
