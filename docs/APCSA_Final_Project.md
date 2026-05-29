---
layout: default
title: AP CSA Presentation Guide
---

# AP CSA Final Project — Presentation Guide
## Vortex: Autonomous Motion & Odometry for VEX Robotics (Team 56S-Override)

**Student:** Jonah Chang
**Course:** AP Computer Science A  
**Project type:** Real-world robotics software (C++ on VEX V5)

Use this document as **speaker notes** and a **slide outline**. Put images on slides; keep bullets short and explain out loud.

---

## Slide 1 — Introduction

**Who am I?**
- [Your Name], AP CSA student and member of **VEX robotics team 56S-Override**
- I program our competition robot’s “brain” (the V5 Brain) in **C++** using the **PROS** framework

**What is this project?**
- **Vortex** — a motion-control library I built for our robot
- It lets the robot **know where it is on the field** (odometry) and **drive itself** to points and paths during autonomous mode
- Think of it as the robot’s navigation + driver for competition autons

**Suggested images:** Photo of you with the robot, team logo, competition field diagram

---

## Slide 2 — Describe the Project & Why I Chose It

**What it does**
- Tracks robot position: **x, y, heading** (inches and degrees)
- Runs **autonomous routines**: drive forward, turn, go to a coordinate, follow a path
- Supports **driver control** (arcade drive) and tools to **tune** control on the robot without recompiling every time

**Why this topic?**
- Robotics is where I actually use programming outside of class
- Our team needed reliable autons; commercial libraries (LemLib, EZ-Template) were hard to adapt, so I wanted something **our team owns and understands**
- Perfect bridge between **AP CSA concepts** (classes, algorithms, testing) and **real hardware**

**What I hoped to accomplish**
- A **clean, reusable** library teammates can configure without reading 500 lines of motion math
- Odometry accurate enough to hit scoring positions consistently
- Motion commands simple enough to write autons in minutes, not hours

**Suggested images:** Robot on field, screenshot of PROS terminal showing pose (x, y, h), diagram of tracking wheel + IMU

---

## Slide 3 — How I Learned on My Own

**Resources I used**
- [PROS documentation](https://pros.cs.purdue.edu/) — API for motors, IMU, tasks
- VEX / community concepts: tracking wheels, PID control, pure pursuit path following
- Studied how **LemLib** and **EZ-Template** structure odometry and chassis APIs (design inspiration, not copy-paste)
- **Doxygen** to document my own code so I could remember what each class does

**Skills I had to learn beyond AP CSA**
- **C++** on embedded systems (pointers, headers, compilation with `pros make`)
- Basic **control theory**: PID loops, error, settling conditions
- **Coordinate geometry**: converting wheel movement + heading into field x/y
- **Concurrency**: background PROS tasks so odometry updates while the robot moves

**What I learned (big ideas)**
- **Object-oriented design**: separate concerns (sensors → odometry → chassis → auton)
- **Abstraction**: hide motor encoder math behind a `DistanceSensor` interface
- **Iteration**: software for robots is never “done on the first try”—you measure, tune, repeat
- **API design**: good function names (`init()`, `reset()`, `drive_to_point()`) matter as much as algorithms

---

## Slide 4 — My Process (How I Built It)

**Phase 1 — Research & plan**
- Listed what autons need: pose, turns, point moves, paths
- Sketched architecture: `Config` → `Odometry` → `Chassis` → `main.cpp` autons

**Phase 2 — Core math & sensors**
- Implemented pose tracking from IMU + tracking wheel
- Built `Wheel` and config tables so ports live in one file

**Phase 3 — Motion control**
- PID controllers for drive and turn
- High-level moves: `drive_distance`, `turn_to_heading`, `drive_to_point`, `follow_path`

**Phase 4 — Usability & tools**
- `odom.init()` and `chassis.init()` — one-call startup
- **Auton selector** — pick routines from the controller
- **PID tuner** — adjust gains on the field

**Phase 5 — Test & refine**
- Test autons on tile, read terminal pose, adjust PID and timeouts
- Simplified API after realizing auton code was too verbose

**Suggested images:** Simple architecture diagram (see below), photo of robot during testing

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│  Sensors    │ ──► │  Odometry    │ ──► │  Chassis    │
│ IMU, wheels │     │  (x, y, θ)   │     │  PID moves  │
└─────────────┘     └──────────────┘     └─────────────┘
                           ▲                    │
                           │                    ▼
                    ┌──────────────┐     ┌─────────────┐
                    │  Config.cpp  │     │  Autons /   │
                    │  port tables │     │  Opcontrol  │
                    └──────────────┘     └─────────────┘
```

---

## Slide 5 — Final Project & Expectations

**Did I meet my goals?**
- **Yes, mostly:** Team can configure hardware in one place and write short auton routines
- **Odometry works** and prints live pose in driver mode
- **Motion commands run** end-to-end (drive test + path test autons)
- **Still improving:** Full competition autons, horizontal tracking wheel when mounted, more field testing

**What the audience will see live**
1. **Driver mode** — class drives with sticks; terminal shows **x, y, heading**
2. **Auton selector** — pick “Drive test” or “Path test” from controller
3. **Autonomous run** — robot moves on its own using odometry feedback

**Do NOT walk through all code on slides.** If you show code, use **one short, readable snippet** only.

**Example snippet — simple auton (readable, not the whole project):**

```cpp
void drive_test_auton() {
  odom.reset();
  chassis.drive_distance(24.0, 3000);
  chassis.turn_to_heading(90.0, 2000);
  chassis.drive_to_point(24.0, 24.0, 4000);
}
```

**Example snippet — config in one place:**

```cpp
namespace Robot {
  const Vortex::DrivetrainConfig drivetrain{{4, -5, -6}, {1, -2, 3}, 11.5, 3.25};
  const Vortex::ImuConfig imu{8};
  const Vortex::TrackerConfig vertical_tracker{-18, 2.0, 0.0};
}
```

**AP CSA connections to mention verbally**
| Concept | In my project |
|--------|----------------|
| Classes & objects | `Odometry`, `Chassis`, `Wheel`, `PID` |
| Interfaces / abstraction | `DistanceSensor` — different encoders, same methods |
| Arrays / collections | `Path` of waypoints, motor port lists |
| Algorithms | PID control, path smoothing, pure pursuit |
| Testing & iteration | Pose logging, auton retries, PID tuner |

---

## Slide 6 — Obstacles & How I Overcame Them

| Obstacle | What I did |
|----------|------------|
| **Odometry drift** — robot position wrong after several moves | Recalibrated IMU, reset pose at start of each auton, tuned wheel diameter |
| **PID overshoot / oscillation** | Used on-robot PID tuner; reduced kP, added settle time |
| **Messy, hard-to-read auton code** | Wrapped motion into blocking helpers (`drive_to_point` instead of long parameter structs) |
| **Too many files to configure** | Centralized ports in `Robot::` namespace in `Config.cpp` |
| **AI-generated code hard to integrate** | AI suggested pieces that didn’t match PROS APIs; I used it for **ideas and structure**, then wrote/tested **my own** integration |
| **C++ build errors** | Read compiler errors, fixed includes, built incrementally (odom first, then chassis) |

**Talking point:** AI is a tool, not a substitute for understanding—when suggestions didn’t compile or match our robot, I had to debug myself.

---

## Slide 7 — Verification & Testing

**How I verified correctness**

1. **Unit-style checks (on robot)**
   - Drive **24 inches** forward — measure with tape vs odometry delta
   - Turn **90°** — compare IMU heading to expected
   - Print pose every 250 ms in opcontrol — watch values respond to pushing the robot

2. **Auton testing**
   - Run **Drive test** auton repeatedly; note where it ends vs intended point
   - Run **Path test** — watch path following and final pose

3. **User control testing**
   - Arcade drive: full range sticks, no jitter at center
   - Auton selector: switch routines, confirm correct one runs
   - PID tuner: change a gain, re-run move, observe behavior change

4. **Safety**
   - Timeouts on every motion so the robot stops if it never “settles”
   - `cancelMotion` / brake when disabled

5. **Iteration loop**
   - Hypothesis → run auton → read terminal pose → adjust PID or coordinates → repeat

**Suggested images:** Terminal screenshot of `x: y: h:` output, photo of measured drive test

---

## Slide 8 — Reflection

**If I had more time, I would…**
- Add full **competition autons** for this season’s game
- Enable **horizontal tracking wheel** for better strafe accuracy
- Log pose to SD card for post-match analysis
- Auto-tune PID from logged error data

**If I started over…**
- Design the **simple API first** (`init`, `reset`, `drive_to_point`) before async/LemLib-style methods
- Write **test checklist** on day one (drive 24", turn 90°, etc.)
- Keep config tables separate from logic from the start (I got there eventually)

**What I’m proud of**
- Real software on real hardware—not just a classroom console program
- Teammates can change ports and write autons without touching PID math

---

## Slide 9 — References & AI Use

**References**
- PROS — https://pros.cs.purdue.edu/
- VEX V5 hardware documentation
- Community discussions of tracking-wheel odometry and PID tuning
- LemLib / EZ-Template (API design inspiration for familiar naming)

**AI use (be honest — your teacher asked for this)**

| Tool | How I used it |
|------|----------------|
| **Cursor / Claude** | Explaining PROS patterns, refactoring odometry/chassis APIs, Doxygen comments, debugging build issues |
| **ChatGPT / Gemini** | (Add if you used — e.g. “explained PID in plain English”) |

**First prompt (example — replace with your actual first prompt):**

> “I’m building VEX V5 robot code in PROS C++. I need odometry with an IMU and one vertical tracking wheel. How should I structure classes for pose tracking and where should the update loop run?”

**Representative prompt (example — replace with yours):**

> “Can you update my odometry so it can be called and set up simpler like LemLib or EZ-Template, with a single init() and config tables for ports?”

**Important:** I reviewed, tested, and modified all AI suggestions. Final design and working code are mine.

**People**
- [Teammates / mentor names] — hardware setup, testing on field, competition strategy

---

## Slide 10 — Live Demo Plan (“Run it!”)

**Before class**
- Charge robot & controller; upload latest code (`pros mu`)
- Pre-select an auton; know your port numbers match the built robot

**Demo script (≈3–5 minutes)**

| Step | What you do | What class sees |
|------|-------------|-----------------|
| 1 | Explain robot + Vortex in 30 sec | Context |
| 2 | **Opcontrol** — drive with sticks | Human control |
| 3 | Show terminal / brain screen **pose updating** | Odometry working |
| 4 | Let a classmate **drive** briefly (optional) | User control |
| 5 | **Auton** — run Drive test or Path test | Autonomous motion |
| 6 | Q&A | — |

**If something fails**
- Have a backup video of a successful run
- Explain what *should* happen and one debugging step you’d take

---

## Slideshow Tips (from checklist)

- **Images:** robot, field, architecture diagram, terminal output
- **Bullets:** 3–5 per slide max; you explain details verbally
- **Talk to the audience**, not the slides
- **Practice** with a timer (aim for 8–12 minutes + demo)
- **Do not** scroll through hundreds of lines of code

---

## Quick Checklist — Did I Cover Everything?

| Requirement | Section |
|-------------|---------|
| Introduce yourself & project | Slide 1 |
| Describe project & why chosen | Slide 2 |
| Hopes / goals | Slide 2 |
| Self-directed learning | Slide 3 |
| What you learned | Slide 3 |
| Process | Slide 4 |
| Final project & expectations | Slide 5 |
| Obstacles | Slide 6 |
| Verification & testing | Slide 7 |
| Reflection | Slide 8 |
| References & AI prompts | Slide 9 |
| Live demo | Slide 10 |

---

*Fill in [Your Name], teammate/mentor names, and your real AI prompts before presenting.*
