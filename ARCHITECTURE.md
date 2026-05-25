# GO2 AI Pet Architecture

## Project Overview

GO2 AI Pet is a robotics + AI agent platform built around:

- modular robot behaviors
- simulation-first development
- real robot compatibility
- extensible AI orchestration
- safe and maintainable robotics architecture

The project supports both:

- PyBullet simulation
- Unitree GO2 hardware execution

through a shared abstraction layer.

---

## High-Level System Architecture

```text
Voice Input / User Commands
            ↓
      Intent Parser
            ↓
 Planner / Orchestrator
            ↓
     Behavior System
            ↓
     RobotInterface
        ↙      ↘
 Simulation   Hardware
```

---

## Core Architectural Principles

### 1. Simulation First

All major robot behaviors should:

- support simulation mode
- be testable without hardware
- avoid hardware-only implementations

Simulation is the primary development environment.

---

### 2. Hardware Abstraction

Robot hardware access should only occur through:

- RobotInterface
- robot/ layer

High-level behaviors should never directly control hardware APIs.

---

### 3. Modular Behaviors

Behaviors should:

- be self-contained
- be interruptible
- avoid shared mutable state
- avoid direct coupling to other behaviors

Examples:

- greet
- follow_user
- sit
- patrol

---

### 4. Centralized Orchestration

Behavior arbitration should be centralized.

The planner/orchestrator is responsible for:

- behavior priority
- interruption handling
- scheduling
- conflict resolution
- future autonomous behavior coordination

Behaviors should not directly control or interrupt each other.

---

### 5. Safety First

Robot safety takes priority over creativity or autonomy.

The system should:

- avoid unsafe motions
- avoid blocking control loops
- validate commands before execution
- support emergency interruption
- validate AI-generated actions

---

## Core Module Responsibilities

### behavior/

High-level robot behaviors.

Responsibilities:

- behavior execution logic
- behavior lifecycle management
- interruptibility support
- orchestration integration

Behaviors should remain independent and modular.

---

### robot/

Low-level robot control layer.

Responsibilities:

- robot hardware APIs
- motor commands
- hardware state access
- hardware communication

This layer isolates hardware-specific logic.

---

### simulation/

PyBullet simulation layer.

Responsibilities:

- simulated robot execution
- simulation testing
- animation and motion validation
- simulation-specific utilities

Simulation should remain independently runnable.

---

### interfaces/

Shared abstraction contracts.

Responsibilities:

- RobotInterface definitions
- behavior interfaces
- shared protocol definitions
- abstraction boundaries

This layer reduces coupling between systems.

---

### voice/

Voice interaction system.

Responsibilities:

- speech recognition
- wake word detection
- text-to-speech
- command ingestion

Voice systems should not directly control robot hardware.

---

### vision/

Perception and sensor processing.

Future responsibilities:

- object detection
- person tracking
- environmental awareness
- visual context processing

---

## Behavior Lifecycle

All behaviors should follow a predictable lifecycle.

```text
registered
    ↓
started
    ↓
running
    ↓
completed / interrupted / failed
```

Behaviors should:

- expose explicit lifecycle transitions
- support interruption
- avoid hidden state transitions

---

## State Management Philosophy

The system should avoid uncontrolled shared mutable state.

Guidelines:

- centralize important robot state
- prefer explicit state transitions
- avoid hidden side effects
- minimize cross-module dependencies
- prefer event-driven coordination

Future systems may introduce:

- state manager
- event bus
- shared memory layer

---

## Event System Philosophy

Subsystems should communicate primarily through events.

Examples:

- behavior_started
- behavior_completed
- wake_word_detected
- motion_interrupted
- planner_task_created

Guidelines:

- avoid direct subsystem dependencies
- prefer event-driven coordination
- events should be explicit and traceable
- avoid hidden side effects from event handlers

---

## Concurrency Model

The system will eventually support concurrent subsystems:

- voice processing
- behavior execution
- planner logic
- memory systems
- perception systems
- autonomous scheduling

Concurrency rules:

- avoid blocking the main control loop
- long-running tasks should be asynchronous
- protect shared state consistency
- behaviors must remain interruptible

---

## Behavior Priority Philosophy

Behaviors may have different priorities.

Examples:

- emergency_stop > all behaviors
- obstacle_avoidance > movement behaviors
- user interaction > idle behaviors

The orchestrator should:

- resolve conflicts centrally
- support interruption
- support preemption
- avoid simultaneous conflicting motions

---

## Robot Control Ownership

Only one active motion controller should command robot movement at a time.

The planner/orchestrator is responsible for:

- granting control ownership
- revoking ownership
- handling interruptions
- resolving motion conflicts

Subsystems should not bypass orchestration control.

---

## Timing Philosophy

The system should avoid assuming perfectly synchronous execution.

Guidelines:

- behaviors should tolerate delayed events
- timeouts should be explicit
- long-running operations should remain interruptible
- avoid relying on precise timing guarantees

---

## Resource Ownership Philosophy

Shared resources should have clear ownership boundaries.

Examples:

- motion control
- sensor access
- planner execution
- camera streams

Avoid uncontrolled concurrent access to shared resources.

---

## AI / LLM Integration

LLMs should assist with:

- planning
- intent interpretation
- conversational interaction
- high-level behavior selection

LLM outputs must NOT directly control hardware actions.

All AI-generated actions should pass through:

- validation
- orchestration
- safety checks

---

## Failure Recovery Philosophy

The system should fail safely.

Guidelines:

- robot motions should stop safely on failure
- subsystems should recover gracefully when possible
- avoid cascading failures between modules
- timeouts should be explicit and handled
- interruption and recovery paths should be tested

---

## Dependency Rules

Preferred dependency direction:

```text
voice/ and vision/
        ↓
planner/
        ↓
behavior/
        ↓
interfaces/
      ↙      ↘
robot/   simulation/
```

Guidelines:

- behavior/ should depend on interfaces/, not robot/ or simulation/
- robot/ should implement hardware-specific interfaces
- simulation/ should implement simulation-specific interfaces
- high-level systems should not depend on low-level implementation details
- avoid circular dependencies between subsystems

---

## Planner Responsibilities

The planner/orchestrator is responsible for:

- behavior arbitration
- priority resolution
- interruption handling
- task scheduling
- future autonomous decision-making
- coordinating subsystem interactions

The planner should remain:

- centralized
- observable
- interruptible
- extensible

---

## Planner Constraints

The planner should coordinate systems, not implement subsystem logic directly.

Avoid:

- embedding behavior logic inside the planner
- direct hardware control from planner
- excessive subsystem coupling

---

## Observability Philosophy

Important system events should be observable and traceable.

Examples:

- behavior transitions
- interruptions
- planner decisions
- motion execution
- subsystem failures

Guidelines:

- prefer structured logging
- avoid silent failures
- make important state transitions explicit
- prioritize debuggability over hidden automation

---

## Simulation Fidelity Philosophy

Simulation behavior should remain as close as possible to real robot behavior.

Avoid:

- simulation-only shortcuts
- hardware-only logic paths
- inconsistent behavior timing

Simulation should be considered a primary validation environment.

---

## Future System Expansion

The architecture is expected to evolve toward:

- planner/orchestrator system
- emotion/state system
- memory system
- autonomous scheduling
- event bus architecture
- multi-modal interaction
- long-term behavior learning

New systems should prioritize:

- modularity
- extensibility
- safety
- low coupling

---

## Anti-Patterns

Avoid:

- giant files
- god objects
- direct hardware access from behaviors
- deeply shared mutable state
- blocking while-loops
- duplicated robot logic
- tightly coupled behaviors
- mixing simulation and hardware logic

---

## Development Philosophy

Prioritize:

- readability
- maintainability
- safety
- extensibility
- architecture consistency

Avoid:

- premature optimization
- unnecessary rewrites
- overengineering
- hidden architectural shortcuts
