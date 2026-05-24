"""
pid_server.py  –  Flask + GEKKO closed-loop PID simulation backend.

Signal convention (engineering units throughout):
  PV   [eng]      process variable
  SP   [eng]      setpoint
  err  [eng]      SP - PV
  ierr [eng·s]    integrator state  ∫err dt
  CO   [0-100%]   controller output

PID formula (ISA, derivative on measurement):
  u_raw = Kp*err + Ki*ierr - Kd*dPV/dt

Anti-windup – conditional integration (clamping):
  The integrator only accumulates when the output is NOT saturated,
  OR when integrating would move the output back inside limits.
  Condition:  integrate iff  (co == u_raw)  or  sign(err) != sign(u_raw - co)
  This is unambiguous, requires no tuning constant, and guarantees
  the integrator never winds up past the point where saturation ends.

Bumpless MAN→AUTO:
  SP  ← current PV  (zero initial error)
  ierr← (co_man - Kp*err) / Ki  (first CO output ≈ last manual CO)
  pv_prev ← current PV  (zero initial derivative)

Bumpless AUTO→MAN:
  last_co returned every tick; frontend seeds CO slider to it.

Dead-time models:
  Implemented as a ring-buffer delay line (no GEKKO needed for delay).
  The ODE is stepped with GEKKO; the delay is applied to the CO signal
  before it enters the ODE.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from collections import deque
import csv, io, math, threading

from flask import Flask, request, jsonify, Response
from flask_cors import CORS
from gekko import GEKKO

app = Flask(__name__)
CORS(app)

# ── Global constants ──────────────────────────────────────────────────────────
DT     = 0.05      # simulation timestep [s]
PV_MIN = 20.0
PV_MAX = 150.0
CO_MIN =  0.0
CO_MAX = 100.0


# ═══════════════════════════════════════════════════════════════════════════════
#  PID CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

# Controller params spec (shown in UI)
PID_PARAMS = [
    {"key":"kp","label":"Kp","min": 0,  "max":20, "default":1.0, "step":0.1},
    {"key":"ki","label":"Ki","min": 0,  "max":10, "default":1.0, "step":0.05},
    {"key":"kd","label":"Kd","min": 0,  "max":5,  "default":0.0, "step":0.01},
]


def pid_step(
    pv:      float,
    sp:      float,
    pv_prev: float,
    ierr:    float,
    Kp:      float,
    Ki:      float,
    Kd:      float,
    co_min:  float = CO_MIN,
    co_max:  float = CO_MAX,
) -> tuple[float, float]:
    """
    One discrete PID step with conditional-integration anti-windup.

    Returns
    -------
    co   : clamped output [%]
    ierr : updated integrator state [eng·s]

    All signals in engineering units.
    Derivative is on PV (not error) → no derivative kick on SP steps.

    Anti-windup (conditional integration / clamping):
      Integrate only when NOT saturated, or when error would pull output
      back inside limits.  Formally:
        saturated_hi = u_raw > co_max
        saturated_lo = u_raw < co_min
        integrate = (not saturated_hi or err < 0) and
                    (not saturated_lo or err > 0)
      This is equivalent to back-calculation with Tt→0 (hard clamp) but
      without any tuning parameter and with guaranteed windup prevention.
    """
    err    = sp - pv
    dpv_dt = (pv - pv_prev) / DT
    u_raw  = Kp * err + Ki * ierr - Kd * dpv_dt

    # Clamp
    co = max(co_min, min(co_max, u_raw))

    # Conditional integration anti-windup
    sat_hi = u_raw > co_max
    sat_lo = u_raw < co_min
    integrate = (not sat_hi or err < 0.0) and (not sat_lo or err > 0.0)
    if integrate:
        ierr += err * DT

    return co, ierr


def pid_preload(co_man: float, pv: float, sp: float,
                Kp: float, Ki: float) -> dict:
    """
    Pre-load integrator for bumpless MAN→AUTO.
    Solves:  co_man = Kp*(sp-pv) + Ki*ierr  →  ierr = (co_man - Kp*err)/Ki
    """
    err  = sp - pv
    ierr = (co_man - Kp * err) / Ki if Ki > 1e-9 else 0.0
    return {"ierr": ierr, "pv_prev": pv}


# ═══════════════════════════════════════════════════════════════════════════════
#  PROCESS MODELS
# ═══════════════════════════════════════════════════════════════════════════════

class BaseProcessModel(ABC):
    """
    Subclass and register in PROCESS_MODELS to add a new model.
    step() receives (pv, co, model_state, params) and returns (new_pv, new_state).
    model_state is a dict managed by the server; models may use it freely.
    """
    name:   str  = ""
    params: list = []

    @abstractmethod
    def step(self, pv: float, co: float,
             model_state: dict, params: dict) -> tuple[float, dict]:
        """Advance one DT. Returns (new_pv, new_model_state)."""

    def initial_state(self, params: dict) -> dict:
        return {}

    def initial_pv(self) -> float:
        return PV_MIN


# ── Helpers ───────────────────────────────────────────────────────────────────

def _gekko_fo(pv0: float, u: float, tau: float, kgain: float) -> float:
    """First-order ODE step via GEKKO: tau*dy/dt = -y + K*u"""
    m = GEKKO(remote=True)
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
    """
    Second-order ODE step via GEKKO:
      tau^2 * y'' + 2*zeta*tau * y' + y = K*u
    State: (y, y')
    """
    m = GEKKO(remote=True)
    m.time = [0.0, DT]
    y1 = m.Var(value=pv0)
    y2 = m.Var(value=dpv0)
    u_ = m.Param(value=u)
    m.Equation(y1.dt() == y2)
    m.Equation(tau**2 * y2.dt() == kgain * u_ - y1 - 2*zeta*tau*y2)
    m.options.IMODE = 4
    m.options.NODES = 2
    m.solve(disp=False, debug=False)
    r1 = float(y1.value[-1])
    r2 = float(y2.value[-1])
    m.cleanup()
    return r1, r2


def _gekko_integrator(pv0: float, u: float, kgain: float) -> float:
    """Pure integrator: dy/dt = K*u"""
    m = GEKKO(remote=True)
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
                  value: float, delay_steps: int) -> tuple[float, deque]:
    """
    Ring-buffer delay line.  Returns (delayed_value, updated_buffer).
    Buffer is stored as a list in state[key] (deque not JSON-serialisable).
    """
    buf = deque(state.get(key, [value] * max(delay_steps, 1)),
                maxlen=max(delay_steps, 1))
    delayed = buf[0]       # oldest = most delayed
    buf.append(value)      # newest
    return delayed, list(buf)


# ── Process model classes ─────────────────────────────────────────────────────

class IntegratorProcess(BaseProcessModel):
    """dy/dt = K * u"""
    name   = "Integrator"
    params = [
        {"key":"kgain","label":"K (gain)","min":-10,"max":10,"default":1.0,"step":0.1},
    ]
    def step(self, pv, co, model_state, params):
        kgain = params.get("kgain", 1.0)
        pv_new = _gekko_integrator(pv, co, kgain)
        return pv_new, {}


class FirstOrderProcess(BaseProcessModel):
    """tau * dy/dt = -y + K * u"""
    name   = "1st Order"
    params = [
        {"key":"tau",  "label":"τ (s)",   "min":0.1,"max":30, "default":1.0,"step":0.1},
        {"key":"kgain","label":"K (gain)","min":-10,"max":10,  "default":1.0,"step":0.1},
    ]
    def step(self, pv, co, model_state, params):
        tau   = params.get("tau",   1.0)
        kgain = params.get("kgain", 1.0)
        pv_new = _gekko_fo(pv, co, tau, kgain)
        return pv_new, {}


class SecondOrderProcess(BaseProcessModel):
    """tau^2*y'' + 2*zeta*tau*y' + y = K*u"""
    name   = "2nd Order"
    params = [
        {"key":"tau",  "label":"τ (s)",   "min":0.1,"max":20,"default":2.0,"step":0.1},
        {"key":"zeta", "label":"ζ (damp)","min":0.1,"max":2, "default":0.7,"step":0.05},
        {"key":"kgain","label":"K (gain)","min":-10,"max":10, "default":1.0,"step":0.1},
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
        {"key":"kgain", "label":"K (gain)",   "min":-10,"max":10, "default":1.0,"step":0.1},
        {"key":"theta", "label":"θ delay (s)","min":0,  "max":20, "default":1.0,"step":0.05},
    ]
    def initial_state(self, params):
        steps = max(1, round(params.get("theta", 1.0) / DT))
        init_co = (CO_MIN + CO_MAX) / 2
        return {"delay_buf": [init_co] * steps}

    def step(self, pv, co, model_state, params):
        kgain = params.get("kgain", 1.0)
        theta = params.get("theta", 1.0)
        delay_steps = max(1, round(theta / DT))
        co_delayed, buf = _delay_buffer(model_state, "delay_buf", co, delay_steps)
        pv_new = _gekko_integrator(pv, co_delayed, kgain)
        return pv_new, {"delay_buf": buf}


class FirstOrderDelayProcess(BaseProcessModel):
    """tau*dy/dt = -y + K*u_delayed,  dead time = theta"""
    name   = "1st Order + Delay"
    params = [
        {"key":"tau",   "label":"τ (s)",      "min":0.1,"max":30, "default":1.0,"step":0.1},
        {"key":"kgain", "label":"K (gain)",   "min":-10,"max":10,  "default":1.0,"step":0.1},
        {"key":"theta", "label":"θ delay (s)","min":0,  "max":20, "default":1.0,"step":0.05},
    ]
    def initial_state(self, params):
        steps = max(1, round(params.get("theta", 1.0) / DT))
        init_co = (CO_MIN + CO_MAX) / 2
        return {"delay_buf": [init_co] * steps}

    def step(self, pv, co, model_state, params):
        tau   = params.get("tau",   1.0)
        kgain = params.get("kgain", 1.0)
        theta = params.get("theta", 1.0)
        delay_steps = max(1, round(theta / DT))
        co_delayed, buf = _delay_buffer(model_state, "delay_buf", co, delay_steps)
        pv_new = _gekko_fo(pv, co_delayed, tau, kgain)
        return pv_new, {"delay_buf": buf}


class SecondOrderDelayProcess(BaseProcessModel):
    """tau^2*y''+2*zeta*tau*y'+y = K*u_delayed,  dead time = theta"""
    name   = "2nd Order + Delay"
    params = [
        {"key":"tau",   "label":"τ (s)",      "min":0.1,"max":20,"default":2.0,"step":0.1},
        {"key":"zeta",  "label":"ζ (damp)",   "min":0.1,"max":2, "default":0.7,"step":0.05},
        {"key":"kgain", "label":"K (gain)",   "min":-10,"max":10, "default":1.0,"step":0.1},
        {"key":"theta", "label":"θ delay (s)","min":0,  "max":20,"default":1.0,"step":0.05},
    ]
    def initial_state(self, params):
        steps = max(1, round(params.get("theta", 1.0) / DT))
        init_co = (CO_MIN + CO_MAX) / 2
        return {"delay_buf": [init_co] * steps, "dpv": 0.0}

    def step(self, pv, co, model_state, params):
        tau   = params.get("tau",   2.0)
        zeta  = params.get("zeta",  0.7)
        kgain = params.get("kgain", 1.0)
        theta = params.get("theta", 1.0)
        delay_steps = max(1, round(theta / DT))
        dpv   = model_state.get("dpv", 0.0)
        co_delayed, buf = _delay_buffer(model_state, "delay_buf", co, delay_steps)
        pv_new, dpv_new = _gekko_so(pv, dpv, co_delayed, tau, zeta, kgain)
        return pv_new, {"delay_buf": buf, "dpv": dpv_new}


# ── Process model registry ────────────────────────────────────────────────────
PROCESS_MODELS: dict[str, BaseProcessModel] = {
    "Integrator":          IntegratorProcess(),
    "1st Order":           FirstOrderProcess(),
    "2nd Order":           SecondOrderProcess(),
    "Integrator + Delay":  IntegratorDelayProcess(),
    "1st Order + Delay":   FirstOrderDelayProcess(),
    "2nd Order + Delay":   SecondOrderDelayProcess(),
}


# ═══════════════════════════════════════════════════════════════════════════════
#  SIMULATION STATE
# ═══════════════════════════════════════════════════════════════════════════════

lock = threading.Lock()

def _fresh_state(model_key: str = "1st Order",
                 model_params: dict | None = None) -> dict:
    model       = PROCESS_MODELS[model_key]
    init_params = model_params or {p["key"]: p["default"] for p in model.params}
    return dict(
        pv          = PV_MIN,
        ctrl_state  = {"ierr": 0.0, "pv_prev": PV_MIN},
        model_state = model.initial_state(init_params),
        t           = 0.0,
        last_co     = 0.0,
        mode        = "manual",
        model_key   = model_key,
    )

state = _fresh_state()
log: list[dict] = []


# ═══════════════════════════════════════════════════════════════════════════════
#  FLASK ROUTES
# ═══════════════════════════════════════════════════════════════════════════════

@app.route("/ping", methods=["GET"])
def ping():
    return jsonify({"ok": True})


@app.route("/config", methods=["GET"])
def config():
    return jsonify({
        "pv_min": PV_MIN,
        "pv_max": PV_MAX,
        "co_min": CO_MIN,
        "co_max": CO_MAX,
        "pid_params": PID_PARAMS,
        "process_models": {
            k: {"name": v.name, "params": v.params}
            for k, v in PROCESS_MODELS.items()
        },
        "default_model": "1st Order",
    })


@app.route("/step", methods=["POST"])
def step_route():
    body      = request.get_json(force=True)
    mode      = body.get("mode",          "manual")
    co_man    = float(body.get("co_man",  0.0))
    sp_eng    = float(body.get("sp_eng",  PV_MIN))  # SP in engineering units
    model_key = body.get("process_model", "1st Order")

    # PID gains
    Kp = float(body.get("kp", 1.0))
    Ki = float(body.get("ki", 1.0))
    Kd = float(body.get("kd", 0.0))

    model = PROCESS_MODELS.get(model_key, PROCESS_MODELS["1st Order"])
    model_params = {p["key"]: float(body.get(p["key"], p["default"]))
                    for p in model.params}

    with lock:
        pv          = state["pv"]
        ctrl_state  = state["ctrl_state"]
        model_state = state["model_state"]
        t           = state["t"]
        prev_model  = state.get("model_key", model_key)

        # Reset model state if model changed
        if model_key != prev_model:
            model_state = model.initial_state(model_params)

        sp = max(PV_MIN, min(PV_MAX, sp_eng))  # clamp SP to PV range

        if mode == "manual":
            co = max(CO_MIN, min(CO_MAX, co_man))
            # Continuously pre-load for bumpless MAN→AUTO
            ctrl_state = pid_preload(co, pv, sp, Kp, Ki)

        else:  # AUTO
            ierr    = ctrl_state.get("ierr",    0.0)
            pv_prev = ctrl_state.get("pv_prev", pv)
            co, ierr = pid_step(pv, sp, pv_prev, ierr, Kp, Ki, Kd)
            ctrl_state = {"ierr": ierr, "pv_prev": pv}

        # Advance process model
        pv_new, model_state = model.step(pv, co, model_state, model_params)

        # Clamp PV to a reasonable range (prevents runaway display)
        pv_new = max(PV_MIN - 50.0, min(PV_MAX + 50.0, pv_new))
        t_new  = round(t + DT, 6)

        state["pv"]          = pv_new
        state["ctrl_state"]  = ctrl_state
        state["model_state"] = model_state
        state["t"]           = t_new
        state["last_co"]     = co
        state["mode"]        = mode
        state["model_key"]   = model_key

        log.append({
            "t":    round(t_new, 4),
            "pv":   round(pv_new, 4),
            "sp":   round(sp, 4) if mode == "auto" else "",
            "co":   round(co, 4),
            "mode": mode,
        })

    return jsonify({
        "t":       t_new,
        "pv":      pv_new,
        "sp":      sp if mode == "auto" else None,
        "co":      co,
        "last_co": co,
    })


@app.route("/transfer_sp", methods=["GET"])
def transfer_sp():
    """Return current PV (eng units) to use as SP on MAN→AUTO."""
    with lock:
        return jsonify({"sp_eng": state["pv"], "last_co": state["last_co"]})


@app.route("/export", methods=["GET"])
def export_csv():
    with lock:
        snapshot = list(log)
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=["t", "pv", "sp", "co", "mode"])
    writer.writeheader()
    writer.writerows(snapshot)
    return Response(buf.getvalue(), mimetype="text/csv",
        headers={"Content-Disposition": "attachment; filename=pid_log.csv"})


@app.route("/reset", methods=["POST"])
def reset():
    body      = request.get_json(force=True, silent=True) or {}
    model_key = body.get("process_model", "1st Order")
    model     = PROCESS_MODELS.get(model_key, PROCESS_MODELS["1st Order"])
    model_params = {p["key"]: float(body.get(p["key"], p["default"]))
                    for p in model.params}
    with lock:
        state.update(_fresh_state(model_key, model_params))
        log.clear()
    return jsonify({"ok": True})


@app.route("/status", methods=["GET"])
def status():
    with lock:
        return jsonify({
            "t": state["t"], "pv": state["pv"],
            "last_co": state["last_co"],
        })


# ── PythonAnywhere WSGI entry point ───────────────────────────────────────────
# PythonAnywhere serves the app via WSGI; the `application` variable is what
# the WSGI server (uWSGI / gunicorn) looks for automatically.
# The app is also runnable directly for local testing:
#   python main.py
application = app   # <── required by PythonAnywhere WSGI

if __name__ == "__main__":
    # Local dev only — not used on PythonAnywhere
    print("PID Server  →  http://localhost:5050")
    app.run(host="0.0.0.0", port=5050, threaded=False)
