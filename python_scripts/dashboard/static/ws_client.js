/**
 * WebSocket client — connects to /ws, reconnects on drop.
 * Dispatches robot_snapshot events to registered handlers.
 *
 * Message shape from server:
 *   {
 *     type: "robot_snapshot",
 *     robots:    { [robot_id]: { x_cm, y_cm, theta_deg, state, gripper_open, ... } },
 *     obstacles: [ { tag_id, row, col, x_cm, y_cm }, ... ],
 *     goals:     [ { tag_id, row, col, x_cm, y_cm }, ... ],
 *   }
 */

const RECONNECT_DELAY_MS = 2000;

let _socket = null;
const _handlers = [];

/** Register a callback that receives the full snapshot object. */
export function onSnapshot(fn) {
    _handlers.push(fn);
}

function _dispatch(snapshot) {
    for (const fn of _handlers) fn(snapshot);
}

function _connect() {
    const url = `ws://${location.host}/ws`;
    _socket = new WebSocket(url);

    _socket.addEventListener("message", (ev) => {
        try {
            const msg = JSON.parse(ev.data);
            if (msg.type === "robot_snapshot") {
                _dispatch({
                    robots:    msg.robots    ?? {},
                    obstacles: msg.obstacles ?? [],
                    goals:     msg.goals     ?? [],
                });
            }
        } catch {
            // ignore malformed frames
        }
    });

    _socket.addEventListener("close", () => {
        setTimeout(_connect, RECONNECT_DELAY_MS);
    });

    _socket.addEventListener("error", () => {
        _socket.close();
    });
}

_connect();
