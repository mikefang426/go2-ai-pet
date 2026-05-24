# 🐕 Go2 AI Pet

A home robotics project combining:

- 🤖 Quadruped robot (Unitree Go2 or simulation)
- 🧠 AI (LLM / agent-based control)
- 🎤 Voice interaction (speech-to-text / text-to-speech)
- 👁 Vision (face recognition & person tracking)

---

## 🚀 Features

- Simulation-first development (no hardware required)
- Modular architecture (AI / behavior / robot / vision / voice)
- Command-based robot control
- Built-in behaviors like greet, beg, shake hand, sit, stand, follow, and patrol
- Extensible to real Go2 robot

---

## Patrol Behavior

`patrol` starts an autonomous low-speed perimeter patrol in the background so the command loop remains interruptible. The default route walks forward, pauses, turns in place, pauses again, then repeats. Use `stop`, `exit`, or another behavior command to preempt it safely.

Environment knobs:

- `PATROL_VX` forward speed, default `0.24`
- `PATROL_YAW` turn-in-place rate, default `0.72`
- `PATROL_STRAIGHT_SEC` straight segment duration, default `3.2`
- `PATROL_TURN_SEC` turn segment duration, default `1.9`
- `PATROL_PAUSE_SEC` pause between segments, default `0.35`
- `PATROL_STEP_SEC` command refresh interval, default `0.1`
- `PATROL_LOOPS` finite route count; unset or `0` means patrol until interrupted

---

## 🧩 Architecture
