<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections
Removed sections: N/A
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending review
- README.md ⚠ pending review
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Embodied Intelligence First
Every robotic system must be designed with embodied cognition in mind; Intelligence emerges from the interaction between the robot's physical form, sensors, actuators, and environment; All algorithms must consider the robot's physical embodiment as a core constraint and opportunity, not an afterthought.

### II. Simulation-to-Reality Pipeline
All development must follow a simulation-first approach; Code must be tested in simulation before hardware deployment; Real-world testing requires safety protocols and gradual progression from simulated to physical environments; Models trained in simulation must demonstrate transfer learning capability to real hardware.

### III. Test-First for Safety-Critical Systems (NON-NEGOTIABLE)
TDD mandatory for all robotics code: Safety tests written → Verified in simulation → Tests pass → Then deploy to hardware; Red-Green-Refactor cycle strictly enforced with emphasis on safety validation; All motion planning and control algorithms must have safety boundary tests.

### IV. ROS 2 Architecture Patterns
All communication must use ROS 2 middleware; Nodes must be modular and independently deployable; Use standard message types and interfaces; Follow DDS communication patterns for distributed systems; All Python code must adhere to PEP 8 standards.

### V. Sensor Fusion and State Estimation
Multi-sensor integration required for robust perception; Uncertainty quantification mandatory for all sensor readings; Kalman filters, particle filters, or equivalent probabilistic methods must be employed for state estimation; Sensor data must be timestamped and synchronized.

### VI. Human-Robot Interaction Design
All humanoid robots must prioritize safe and intuitive human interaction; Interface design must consider human factors and ergonomics; Emergency stop mechanisms must be accessible and responsive; Social robotics principles must guide behavioral design.

## Technical Standards
<!-- Code quality, technology stack, and compliance requirements -->

All ROS 2 packages must follow the Robot Operating System 2 (ROS 2) style guide; Python code must comply with PEP 8 standards with maximum line length of 88 characters; Use Docusaurus for documentation generation; C++ code must follow Google C++ Style Guide; All code must include unit tests with minimum 80% coverage; Type hints required for all Python function signatures.

## Development Workflow
<!-- Code review, testing, and deployment processes -->

All pull requests require dual review: one technical expert and one safety reviewer; Pre-commit hooks enforce code formatting and static analysis; Continuous integration must pass simulation tests before hardware deployment; Hardware testing requires safety checklist completion; Documentation updates required for all public interfaces; Code must be compatible with Ubuntu 22.04 LTS and ROS 2 Humble Hawksbill.

## Governance
<!-- Amendment procedures and compliance oversight -->

This constitution governs all development activities for the Physical AI & Humanoid Robotics course; Amendments require instructor approval and student consensus; All PRs must verify compliance with embodied intelligence principles; Code complexity must be justified with reference to robot performance gains; Use this constitution as the primary reference for development decisions.

**Version**: 1.0.0 | **Ratified**: 2025-01-07 | **Last Amended**: 2025-01-07
