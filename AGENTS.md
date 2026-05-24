# Agent Workflow

When implementing a new behavior:

1. Add intent parsing
2. Register behavior
3. Add simulation support
4. Add RobotInterface support
5. Update README
6. Add tests
7. Validate interruptibility

Before large refactors:
- analyze architecture impact
- identify coupling risks
- preserve simulation compatibility

## Before Writing Code

- inspect existing architecture first
- avoid duplicate abstractions
- reuse RobotInterface when possible
- prefer extending existing systems over creating parallel systems

## Before Refactoring

- identify coupling risks
- preserve backward compatibility
- avoid unnecessary rewrites
- minimize architectural disruptions

## Implementation Strategy

When implementing new systems:

1. analyze existing architecture
2. identify reusable abstractions
3. design minimal extension points
4. implement simulation support first
5. validate interruptibility
6. update documentation
7. review future extensibility impact

## Implementation Constraints

Avoid:

- bypassing RobotInterface abstractions
- introducing hidden shared state
- creating tightly coupled behaviors
- implementing hardware-specific logic inside behaviors
- adding parallel systems that duplicate existing abstractions
- large-scale rewrites without architectural justification

## Before Submitting Changes

- verify architecture consistency
- check for dependency boundary violations
- ensure simulation compatibility
- review interruptibility behavior
- verify no hidden blocking operations were introduced

## Code Review Priorities

Prioritize reviewing:

- architecture boundary violations
- unsafe robot control paths
- hidden state mutations
- blocking operations
- planner complexity growth
- simulation/hardware leakage

## Incremental Development Philosophy

Prefer:

- small focused changes
- incremental improvements
- isolated refactors
- architecture-preserving evolution

Avoid:

- massive rewrites
- broad speculative abstractions
- unnecessary system replacement

## Testing Priorities

Prioritize testing:

- interruptibility
- planner coordination
- simulation consistency
- failure recovery
- behavior transitions

## Documentation Expectations

When introducing new systems:

- document architectural intent
- explain subsystem responsibilities
- describe important constraints
- update architecture docs if boundaries change
