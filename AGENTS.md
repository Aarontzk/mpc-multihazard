# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

MPC RL Disaster — combines Model Predictive Control (MPC) and Reinforcement Learning (RL) for disaster management/response scenarios.

## File Roles

| File | Purpose |
|------|---------|
| `structure.py` | Data structures, state representations, environment/scenario models |
| `disaster.py` | Core disaster simulation or environment logic |
| `workflow.py` | Orchestration — training loops, control pipelines, episode runners |
| `metric.py` | Evaluation metrics — reward functions, KPIs, performance measurement |

## Running

```bash
python workflow.py   # main entry point (orchestration)
python disaster.py   # run disaster environment standalone
```

## Architecture

Intended layering:
1. `structure.py` — define state/action spaces, data classes
2. `disaster.py` — environment dynamics (Gym-compatible or custom)
3. `metric.py` — reward shaping and evaluation metrics
4. `workflow.py` — training/control loop that wires everything together

MPC component should compute optimal control actions over a receding horizon. RL component should learn a policy that approximates or augments MPC decisions under uncertainty.
