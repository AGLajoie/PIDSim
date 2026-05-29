# PIDSim
General multi-purpose dynamically controlled process simulator.

In BASH (https://www.pythonanywhere.com/user/AGLajoie/consoles/46877070/)
(in case of mistake : rm -r directory_to_remove)

—— Step 1 ——————————————————————————————————————

git clone https://github.com/AGLajoie/PIDSim.git

—— Step 2 ——————————————————————————————————————

cd PIDSim

—— Step 3 ——————————————————————————————————————

mkvirtualenv --python=python3.10 pidsim-env

—— Step 4 ——————————————————————————————————————

pip install -r requirements.txt

—— Step 5 ——————————————————————————————————————

python main.py

—— Step 6 ——————————————————————————————————————

WSGI configuration file:
import sys
sys.path.insert(0, '/home/AGLajoie/PIDSim')

from main import application

BaSH :

cd ~

git clone https://github.com/AGLajoie/PIDSim.git

cd PIDSim

mkvirtualenv --python=python3.10 pidsim-env

pip install -r requirements.txt

python main.py

—— Step 7 ——————————————————————————————————————

git status

git pull origin

# Flowchart
The issue is the special Unicode characters (·, τ, ζ, ∫, ẏ, ÿ) and emoji breaking the parser. Here's a cleaned-up version:

```mermaid
graph TB
    subgraph Browser["Browser - index.html"]
        UI["User Interface\nCharts, PID sliders, Mode toggle"]
    end

    subgraph Flask["Flask Server - main.py"]
        direction TB
        Routes["HTTP Routes\n/control, /control_state\n/historian, /config\n/reset, /export_csv, /export_xml"]
        SimLock["sim_lock\nthreading.Lock"]
        CtrlIntent["ctrl_intent - Shared State\nmode, SP, CO, Kp, Ki, Kd, model"]
        SimState["sim_state - Live State\npv, t, last_co, ctrl_state"]

        subgraph SimLoop["Background Thread - sim_loop DT=100ms"]
            DoStep["_do_sim_step()"]
            PID["PID Controller\nu = Kp*err + Ki*ierr - Kd*dPV\nAnti-windup + Bumpless transfer"]
            Models["Process Model\nGEKKO ODE Solver"]
        end

        subgraph ProcessModels["Process Models"]
            FO["1st Order"]
            SO["2nd Order"]
            INT["Integrator"]
            DELAY["Delay variants"]
        end
    end

    subgraph Historian["historian.xml - Disk"]
        XML["XML Entries\nt, pv, sp, co, error\nkp, ki, kd, mode, model"]
    end

    UI -- "POST /control" --> Routes
    UI -- "GET /historian, GET /control_state" --> Routes

    Routes <--> SimLock
    SimLock <--> CtrlIntent
    SimLock <--> SimState

    SimLoop -- "reads" --> CtrlIntent
    SimLoop -- "updates" --> SimState
    DoStep --> PID
    PID -- "CO" --> Models
    Models -- "PV" --> DoStep

    Models --> FO
    Models --> SO
    Models --> INT
    Models --> DELAY

    DoStep -- "_historian_append()" --> XML
    XML -- "_historian_read_last(n)" --> Routes
    Routes -- "JSON entries" --> UI
```

The main fixes: removed all Unicode math symbols, emoji, `·` separators, and replaced them with plain ASCII commas and dashes.
