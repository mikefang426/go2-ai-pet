# GO2 AI Pet Project

This is a robotics + AI agent project.

## Architecture

- modular behavior system
- simulation + real robot support
- RobotInterface abstraction
- behavior-driven design

## Project Goals

- maintainable robotics architecture
- safe robot behavior execution
- extensible AI behavior system
- simulation-first development

## Rules

- prioritize robot safety
- avoid blocking loops
- keep simulation independent from hardware
- use type hints
- avoid giant files
- prefer modular behaviors
- avoid hidden shared state
- avoid tight coupling between behaviors

## Behavior Guidelines

- behaviors should be interruptible
- behaviors should avoid direct hardware access
- behaviors should use RobotInterface abstraction
- behaviors should not manage global state

## Coding Preferences

- prioritize readability
- avoid overengineering
- keep functions focused
- avoid giant classes
- prefer composition over inheritance

## Debugging Rules

- identify root cause first
- prefer minimal fixes
- avoid rewriting unrelated code
- preserve architecture consistency

## Folder Responsibilities

- behavior/: high-level robot behaviors
- robot/: low-level robot hardware control
- simulation/: PyBullet simulation implementation
- interfaces/: abstraction contracts
- voice/: speech input/output and wake word handling
- vision/: perception and future sensor processing

## Safety Constraints

- never generate unsafe robot motion
- avoid abrupt movement transitions
- avoid infinite control loops
- simulation validation before hardware execution
- robot actions must be interruptible

## Future Architecture Direction

The project will eventually support:
- memory system
- planner/orchestrator
- emotion/state system
- multi-modal interaction
- autonomous behavior scheduling

Design new code with extensibility in mind.
Avoid tightly coupling behaviors together.

## Anti-Patterns

Avoid:
- giant files
- god objects
- blocking while-loops
- hidden global state
- duplicated robot logic
- mixing simulation and hardware code
- direct hardware access from behaviors

## State Management Rules

- avoid shared mutable state
- centralize robot state management
- behaviors should not directly modify global system state
- state transitions should be explicit and traceable
- avoid hidden side effects between behaviors
- prefer event-driven coordination over direct coupling

## Concurrency Rules

- avoid blocking the main control loop
- long-running tasks should be asynchronous
- robot motion execution should be interruptible
- avoid unsafe shared mutable data
- protect robot state consistency

## Testing Philosophy

- validate behavior in simulation before hardware
- add regression tests for bug fixes
- verify interruptibility of behaviors
- prefer deterministic behavior testing
- avoid hardware-only validation

## Orchestration Principles

- only one high-priority motion behavior should control the robot at a time
- behavior arbitration should be centralized
- behaviors should communicate through events or orchestration layers
- avoid direct behavior-to-behavior control
- planner/orchestrator should manage behavior priority and interruption

## Data Flow Philosophy

- prefer explicit data flow
- avoid deeply shared object graphs
- pass minimal required context to behaviors
- avoid hidden cross-module dependencies
- prefer immutable or controlled state updates when possible

## LLM Safety Rules

- LLM outputs should not directly control hardware actions
- validate AI-generated commands before execution
- planner decisions should pass through safety checks
- avoid autonomous unsafe behavior generation
