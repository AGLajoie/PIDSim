"""
main.py  –  Flask + GEKKO closed-loop PID simulation backend.

Includes all GEKKO-based process model definitions (originally process_models.py).

Signal convention (engineering units throughout):
  PV   [eng]      process variable
  SP   [eng]      setpoint
  err  [eng]      SP - PV
  ierr [eng·s]    integrator state  ∫err dt
  CO   [0-100%]   controller output

PID formula (ISA, derivative on measurement):
  u_raw = Kp*err + Ki*ierr - Kd*dPV/dt

Anti-windup – conditional integration (clamping):
  Integrate only when NOT saturated or when integrating would move output
  back inside limits.

Bumpless MAN→AUTO:
  SP  ← current PV  (zero initial error)
  ierr← (co_man - Kp*err) / Ki  (first CO output ≈ last manual CO)

historian.xml is written at every simulation step. It is kept compact
(no in-memory list) to stay well inside the 512 MB storage limit for
10+ min of logging at DT=0.05 s (~12 000 entries/min).
"""

from __future__ import annotations
import io, math, os, threading, xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from collections import deque
from datetime import datetime

from flask import Flask, request, jsonify, Response, render_template, send_file
from flask_cors import CORS
from gekko import GEKKO


# ══════════════════════════════════════════════════════════════════════════════
#  SIMULATION CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════

DT     = 0.05      # timestep [s]
PV_MIN = 20.0
PV_MAX = 150.0
CO_MIN =  0.0
CO_MAX = 100.0


# ══════════════════════════════════════════════════════════════════════════════
#  GEKKO ODE HELPERS  (one small GEKKO instance per call, cleaned up after)
# ══════════════════════════════════════════════════════════════════════════════

def _gekko_fo(pv0: float, u: float, tau: float, kgain: float) -> float:
    """First-order ODE: tau * dy/dt = -y + K*u"""
    m = GEKKO(remote=False)
    m.time = [0.0, DT]
    y = m.Var(value=pv0)
    m.Equation(tau * y.dt() == -y + kgain * u)
    m.options.IMODE = 4
    m.options.NODES = 2
    m.solve(disp=False, debug=False)
    result = float(y.value[-1])
    m.cleanup()
    return result


def _gekko_so(pv0: float, dpv0: float,
              u: float, tau: float, zeta: float, kgain: float
              ) -> tuple[float, float]:
    """Second-order ODE: tau^2*y'' + 2*zeta*tau*y' + y = K*u"""
    m = GEKKO(remote=False)
    m.time = [0.0, DT]
    y1 = m.Var(value=pv0)
    y2 = m.Var(value=dpv0)
    u_ = m.Param(value=u)
    m.Equation(y1.dt() == y2)
    m.Equation(tau**2 * y2.dt() == kgain * u_ - y1 - 2 * zeta * tau * y2)
    m.options.IMODE = 4
    m.options.NODES = 2
    m.solve(disp=False, debug=False)
    r1 = float(y1.value[-1])
    r2 = float(y2.value[-1])
    m.cleanup()
    return r1, r2


def _gekko_integrator(pv0: float, u: float, kgain: float) -> float:
    """Pure integrator: dy/dt = K*u"""
    m = GEKKO(remote=False)
    m.time = [0.0, DT]
    y = m.Var(value=pv0)
    m.Equation(y.dt() == kgain * u)
    m.options.IMODE = 4
    m.options.NODES = 2
    m.solve(disp=False, debug=False)
    result = float(y.value[-1])
    m.cleanup()
    return result


def _delay_buffer(state: dict, key: str,
                  value: float, delay_steps: int) -> tuple[float, list]:
    """
    Ring-buffer delay line.
    Returns (delayed_value, updated_buffer_as_list).
    Buffer stored as list (JSON-serialisable).
    """
    buf = deque(state.get(key, [value] * max(delay_steps, 1)),
                maxlen=max(delay_steps, 1))
    delayed = buf[0]
    buf.append(value)
    return delayed, list(buf)


# ══════════════════════════════════════════════════════════════════════════════
#  BASE PROCESS MODEL
# ══════════════════════════════════════════════════════════════════════════════

class BaseProcessModel(ABC):
    name:   str  = ""
    params: list = []          # list of param spec dicts

    @abstractmethod
    def step(self, pv: float, co: float,
             model_state: dict, params: dict) -> tuple[float, dict]:
        """Advance one DT. Returns (new_pv, new_model_state)."""

    def initial_state(self, params: dict) -> dict:
        return {}

    def initial_pv(self) -> float:
        return PV_MIN


# ══════════════════════════════════════════════════════════════════════════════
#  CONCRETE PROCESS MODELS
# ══════════════════════════════════════════════════════════════════════════════

class IntegratorProcess(BaseProcessModel):
    """dy/dt = K * u"""
    name   = "Integrator"
    params = [
        {"key": "kgain", "label": "K (gain)", "min": -10, "max": 10,
         "default": 1.0, "step": 0.1},
    ]

    def step(self, pv, co, model_state, params):
        kgain  = params.get("kgain", 1.0)
        pv_new = _gekko_integrator(pv, co, kgain)
        return pv_new, {}


class FirstOrderProcess(BaseProcessModel):
    """tau * dy/dt = -y + K * u"""
    name   = "1st Order"
    params = [
        {"key": "tau",   "label": "τ (s)",    "min": 0.1, "max": 30,
         "default": 1.0, "step": 0.1},
        {"key": "kgain", "label": "K (gain)", "min": -10, "max": 10,
         "default": 1.0, "step": 0.1},
    ]

    def step(self, pv, co, model_state, params):
        tau    = params.get("tau",   1.0)
        kgain  = params.get("kgain", 1.0)
        pv_new = _gekko_fo(pv, co, tau, kgain)
        return pv_new, {}


class SecondOrderProcess(BaseProcessModel):
    """tau^2*y'' + 2*zeta*tau*y' + y = K*u"""
    name   = "2nd Order"
    params = [
        {"key": "tau",   "label": "τ (s)",    "min": 0.1, "max": 20,
         "default": 2.0, "step": 0.1},
        {"key": "zeta",  "label": "ζ (damp)", "min": 0.1, "max": 2,
         "default": 0.7, "step": 0.05},
        {"key": "kgain", "label": "K (gain)", "min": -10, "max": 10,
         "default": 1.0, "step": 0.1},
    ]

    def initial_state(self, params):
        return {"dpv": 0.0}

    def step(self, pv, co, model_state, params):
        tau   = params.get("tau",   2.0)
        zeta  = params.get("zeta",  0.7)
        kgain = params.get("kgain", 1.0)
        dpv   = model_state.get("dpv", 0.0)
        pv_new, dpv_new = _gekko_so(pv, dpv, co, tau, zeta, kgain)
        return pv_new, {"dpv": dpv_new}


class IntegratorDelayProcess(BaseProcessModel):
    """dy/dt = K * u_delayed,  dead time = theta"""
    name   = "Integrator + Delay"
    params = [
        {"key": "kgain", "label": "K (gain)",    "min": -10, "max": 10,
         "default": 1.0, "step": 0.1},
        {"key": "theta", "label": "θ delay (s)", "min": 0,   "max": 20,
         "default": 1.0, "step": 0.05},
    ]

    def initial_state(self, params):
        steps   = max(1, round(params.get("theta", 1.0) / DT))
        init_co = (CO_MIN + CO_MAX) / 2
        return {"delay_buf": [init_co] * steps}

    def step(self, pv, co, model_state, params):
        kgain       = params.get("kgain", 1.0)
        theta       = params.get("theta", 1.0)
        delay_steps = max(1, round(theta / DT))
        co_delayed, buf = _delay_buffer(model_state, "delay_buf", co, delay_steps)
        pv_new = _gekko_integrator(pv, co_delayed, kgain)
        return pv_new, {"delay_buf": buf}


class FirstOrderDelayProcess(BaseProcessModel):
    """tau*dy/dt = -y + K*u_delayed,  dead time = theta"""
    name   = "1st Order + Delay"
    params = [
        {"key": "tau",   "label": "τ (s)",       "min": 0.1, "max": 30,
         "default": 1.0, "step": 0.1},
        {"key": "kgain", "label": "K (gain)",    "min": -10, "max": 10,
         "default": 1.0, "step": 0.1},
        {"key": "theta", "label": "θ delay (s)", "min": 0,   "max": 20,
         "default": 1.0, "step": 0.05},
    ]

    def initial_state(self, params):
        steps   = max(1, round(params.get("theta", 1.0) / DT))
        init_co = (CO_MIN + CO_MAX) / 2
        return {"delay_buf": [init_co] * steps}

    def step(self, pv, co, model_state, params):
        tau         = params.get("tau",   1.0)
        kgain       = params.get("kgain", 1.0)
        theta       = params.get("theta", 1.0)
        delay_steps = max(1, round(theta / DT))
        co_delayed, buf = _delay_buffer(model_state, "delay_buf", co, delay_steps)
        pv_new = _gekko_fo(pv, co_delayed, tau, kgain)
        return pv_new, {"delay_buf": buf}


class SecondOrderDelayProcess(BaseProcessModel):
    """tau^2*y''+2*zeta*tau*y'+y = K*u_delayed,  dead time = theta"""
    name   = "2nd Order + Delay"
    params = [
        {"key": "tau",   "label": "τ (s)",       "min": 0.1, "max": 20,
         "default": 2.0, "step": 0.1},
        {"key": "zeta",  "label": "ζ (damp)",    "min": 0.1, "max": 2,
         "default": 0.7, "step": 0.05},
        {"key": "kgain", "label": "K (gain)",    "min": -10, "max": 10,
         "default": 1.0, "step": 0.1},
        {"key": "theta", "label": "θ delay (s)", "min": 0,   "max": 20,
         "default": 1.0, "step": 0.05},
    ]

    def initial_state(self, params):
        steps   = max(1, round(params.get("theta", 1.0) / DT))
        init_co = (CO_MIN + CO_MAX) / 2
        return {"delay_buf": [init_co] * steps, "dpv": 0.0}

    def step(self, pv, co, model_state, params):
        tau         = params.get("tau",   2.0)
        zeta        = params.get("zeta",  0.7)
        kgain       = params.get("kgain", 1.0)
        theta       = params.get("theta", 1.0)
        delay_steps = max(1, round(theta / DT))
        dpv         = model_state.get("dpv", 0.0)
        co_delayed, buf = _delay_buffer(model_state, "delay_buf", co, delay_steps)
        pv_new, dpv_new = _gekko_so(pv, dpv, co_delayed, tau, zeta, kgain)
        return pv_new, {"delay_buf": buf, "dpv": dpv_new}


# ── Model registry  –  add new models here ───────────────────────────────────

PROCESS_MODELS: dict[str, BaseProcessModel] = {
    "Integrator":          IntegratorProcess(),
    "1st Order":           FirstOrderProcess(),
    "2nd Order":           SecondOrderProcess(),
    "Integrator + Delay":  IntegratorDelayProcess(),
    "1st Order + Delay":   FirstOrderDelayProcess(),
    "2nd Order + Delay":   SecondOrderDelayProcess(),
}


# ══════════════════════════════════════════════════════════════════════════════
#  FLASK APP
# ══════════════════════════════════════════════════════════════════════════════

app = Flask(__name__, template_folder='templates')
CORS(app)

# ── Historian file path ───────────────────────────────────────────────────────
HISTORIAN_PATH = os.path.join(os.path.dirname(__file__), 'historian.xml')

# ── PID controller param spec (shown in UI) ───────────────────────────────────
PID_PARAMS = [
    {"key": "kp", "label": "Kp", "min": 0,   "max": 20, "default": 1.0, "step": 0.1},
    {"key": "ki", "label": "Ki", "min": 0,   "max": 10, "default": 1.0, "step": 0.05},
    {"key": "kd", "label": "Kd", "min": 0,   "max": 5,  "default": 0.0, "step": 0.01},
]


# ══════════════════════════════════════════════════════════════════════════════
#  HISTORIAN
# ══════════════════════════════════════════════════════════════════════════════

_hist_lock = threading.Lock()


def _historian_init():
    """Create or overwrite historian.xml with an empty root."""
    with open(HISTORIAN_PATH, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="utf-8"?>\n<historian>\n</historian>\n')


def _historian_append(entry: dict):
    """
    Append one <entry .../> line to historian.xml.
    Uses a fast seek-to-closing-tag strategy to avoid re-parsing the whole file.
    """
    attrs = ' '.join(f'{k}="{v}"' for k, v in entry.items())
    line  = f'  <entry {attrs}/>\n'

    with _hist_lock:
        with open(HISTORIAN_PATH, 'r+b') as f:
            f.seek(0, 2)                    # EOF
            size = f.tell()
            chunk_size = min(size, 256)
            f.seek(size - chunk_size)
            tail = f.read().decode('utf-8')
            close_tag = '</historian>'
            idx = tail.rfind(close_tag)
            if idx == -1:
                _historian_init()
                f.seek(0, 2)
                size = f.tell()
                f.seek(size - len(close_tag) - 1)
                idx = 0
            pos = size - chunk_size + idx
            f.seek(pos)
            f.write((line + close_tag + '\n').encode('utf-8'))
            f.truncate()


def _historian_read_last(n: int = 300) -> list[dict]:
    """
    Parse historian.xml and return up to the last n entries as a list of dicts.
    Used by the /historian endpoint to feed the frontend.
    """
    try:
        tree = ET.parse(HISTORIAN_PATH)
        root = tree.getroot()
        entries = root.findall('entry')
        return [e.attrib for e in entries[-n:]]
    except Exception:
        return []


def _historian_size_kb() -> float:
    try:
        return os.path.getsize(HISTORIAN_PATH) / 1024
    except Exception:
        return 0.0


# ══════════════════════════════════════════════════════════════════════════════
#  PID CONTROLLER
# ══════════════════════════════════════════════════════════════════════════════

def pid_step(pv, sp, pv_prev, ierr, Kp, Ki, Kd,
             co_min=CO_MIN, co_max=CO_MAX):
    """
    One discrete PID step with conditional-integration anti-windup.
    Derivative on PV (no kick on SP steps).
    Returns (co, ierr_new).
    """
    err    = sp - pv
    dpv_dt = (pv - pv_prev) / DT
    u_raw  = Kp * err + Ki * ierr - Kd * dpv_dt

    co = max(co_min, min(co_max, u_raw))

    sat_hi    = u_raw > co_max
    sat_lo    = u_raw < co_min
    integrate = (not sat_hi or err < 0.0) and (not sat_lo or err > 0.0)
    if integrate:
        ierr += err * DT

    return co, ierr


def pid_preload(co_man, pv, sp, Kp, Ki):
    """Pre-load integrator for bumpless MAN→AUTO."""
    err  = sp - pv
    ierr = (co_man - Kp * err) / Ki if Ki > 1e-9 else 0.0
    return {"ierr": ierr, "pv_prev": pv}


# ══════════════════════════════════════════════════════════════════════════════
#  SIMULATION STATE
# ══════════════════════════════════════════════════════════════════════════════

lock = threading.Lock()


def _fresh_state(model_key='1st Order', model_params=None):
    model       = PROCESS_MODELS[model_key]
    init_params = model_params or {p["key"]: p["default"] for p in model.params}
    _historian_init()
    return dict(
        pv          = PV_MIN,
        ctrl_state  = {"ierr": 0.0, "pv_prev": PV_MIN},
        model_state = model.initial_state(init_params),
        t           = 0.0,
        last_co     = 0.0,
        mode        = "manual",
        model_key   = model_key,
        kp          = 1.0,
        ki          = 1.0,
        kd          = 0.0,
        sp          = PV_MIN,
    )


state = _fresh_state()


# ══════════════════════════════════════════════════════════════════════════════
#  FLASK ROUTES
# ══════════════════════════════════════════════════════════════════════════════

@app.route("/ping", methods=["GET"])
def ping():
    return jsonify({"ok": True})


@app.route("/config", methods=["GET"])
def config():
    return jsonify({
        "pv_min":         PV_MIN,
        "pv_max":         PV_MAX,
        "co_min":         CO_MIN,
        "co_max":         CO_MAX,
        "pid_params":     PID_PARAMS,
        "process_models": {
            k: {"name": v.name, "params": v.params}
            for k, v in PROCESS_MODELS.items()
        },
        "default_model":  "1st Order",
    })


@app.route("/step", methods=["POST"])
def step_route():
    body      = request.get_json(force=True)
    mode      = body.get("mode",          "manual")
    co_man    = float(body.get("co_man",  0.0))
    sp_eng    = float(body.get("sp_eng",  PV_MIN))
    model_key = body.get("process_model", "1st Order")
    Kp = float(body.get("kp", 1.0))
    Ki = float(body.get("ki", 1.0))
    Kd = float(body.get("kd", 0.0))

    model        = PROCESS_MODELS.get(model_key, PROCESS_MODELS["1st Order"])
    model_params = {p["key"]: float(body.get(p["key"], p["default"]))
                    for p in model.params}

    with lock:
        pv          = state["pv"]
        ctrl_state  = state["ctrl_state"]
        model_state = state["model_state"]
        t           = state["t"]
        prev_model  = state.get("model_key", model_key)

        if model_key != prev_model:
            model_state = model.initial_state(model_params)

        sp = max(PV_MIN, min(PV_MAX, sp_eng))

        if mode == "manual":
            co         = max(CO_MIN, min(CO_MAX, co_man))
            ctrl_state = pid_preload(co, pv, sp, Kp, Ki)
        else:
            ierr    = ctrl_state.get("ierr",    0.0)
            pv_prev = ctrl_state.get("pv_prev", pv)
            co, ierr = pid_step(pv, sp, pv_prev, ierr, Kp, Ki, Kd)
            ctrl_state = {"ierr": ierr, "pv_prev": pv}

        pv_new, model_state = model.step(pv, co, model_state, model_params)
        pv_new = max(PV_MIN - 50.0, min(PV_MAX + 50.0, pv_new))
        t_new  = round(t + DT, 6)

        err = sp - pv_new if mode == "auto" else 0.0

        state["pv"]          = pv_new
        state["ctrl_state"]  = ctrl_state
        state["model_state"] = model_state
        state["t"]           = t_new
        state["last_co"]     = co
        state["mode"]        = mode
        state["model_key"]   = model_key
        state["kp"]          = Kp
        state["ki"]          = Ki
        state["kd"]          = Kd
        state["sp"]          = sp

        entry = {
            "t":      f"{t_new:.4f}",
            "pv":     f"{pv_new:.4f}",
            "sp":     f"{sp:.4f}" if mode == "auto" else "",
            "co":     f"{co:.4f}",
            "error":  f"{err:.4f}" if mode == "auto" else "",
            "kp":     f"{Kp:.4f}",
            "ki":     f"{Ki:.4f}",
            "kd":     f"{Kd:.4f}",
            "mode":   mode,
            "model":  model_key,
            "params": ";".join(f"{k}={v:.4f}" for k, v in model_params.items()),
        }
        _historian_append(entry)

    return jsonify({
        "t":       t_new,
        "pv":      pv_new,
        "sp":      sp if mode == "auto" else None,
        "co":      co,
        "last_co": co,
        "error":   err,
        "kp":      Kp,
        "ki":      Ki,
        "kd":      Kd,
        "mode":    mode,
        "model":   model_key,
    })


@app.route("/historian", methods=["GET"])
def historian_data():
    """Return last N entries from historian.xml as JSON for the UI."""
    n = int(request.args.get("n", 300))
    n = min(n, 3000)
    entries = _historian_read_last(n)
    size_kb = _historian_size_kb()
    return jsonify({"entries": entries, "size_kb": round(size_kb, 1)})


@app.route("/export_xml", methods=["GET"])
def export_xml():
    """Download the full historian.xml."""
    return send_file(HISTORIAN_PATH, mimetype="application/xml",
                     as_attachment=True, download_name="historian.xml")


@app.route("/export_csv", methods=["GET"])
def export_csv():
    """Convert historian.xml to CSV and serve as download."""
    entries = _historian_read_last(999999)
    if not entries:
        return Response("no data", mimetype="text/plain")
    fields = ["t", "pv", "sp", "co", "error", "kp", "ki", "kd", "mode", "model", "params"]
    lines  = [",".join(fields)]
    for e in entries:
        lines.append(",".join(e.get(f, "") for f in fields))
    csv_text = "\n".join(lines)
    return Response(csv_text, mimetype="text/csv",
                    headers={"Content-Disposition": "attachment; filename=pidsim_log.csv"})


@app.route("/transfer_sp", methods=["GET"])
def transfer_sp():
    with lock:
        return jsonify({"sp_eng": state["pv"], "last_co": state["last_co"]})


@app.route("/reset", methods=["POST"])
def reset():
    body      = request.get_json(force=True, silent=True) or {}
    model_key = body.get("process_model", "1st Order")
    model     = PROCESS_MODELS.get(model_key, PROCESS_MODELS["1st Order"])
    model_params = {p["key"]: float(body.get(p["key"], p["default"]))
                    for p in model.params}
    with lock:
        state.update(_fresh_state(model_key, model_params))
    return jsonify({"ok": True})


@app.route("/status", methods=["GET"])
def status():
    with lock:
        return jsonify({
            "t": state["t"], "pv": state["pv"], "last_co": state["last_co"],
        })


@app.route("/")
def index():
    return render_template("index.html")


# ── PythonAnywhere WSGI entry point ───────────────────────────────────────────
application = app

if __name__ == "__main__":
    print("PIDSim  →  http://localhost:5050")
    app.run(host="0.0.0.0", port=5050, threaded=False)
