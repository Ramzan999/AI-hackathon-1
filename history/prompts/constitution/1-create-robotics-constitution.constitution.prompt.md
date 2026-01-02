---
id: 1
title: Create Robotics Constitution
stage: constitution
date: 2025-01-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Administrator
command: /sp.constitution
labels: [constitution, robotics, ai, humanoid]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

Act as a Senior Robotics Engineer and Technical Writer. The goal is to create a high-quality Docusaurus book for the 'Physical AI & Humanoid Robotics' course. The tone must be academic yet practical, focusing on 'Embodied Intelligence.' Use clear Markdown, Docusaurus callouts (admonitions), and ensure all ROS 2/Python code snippets follow PEP 8 standards.

## Response snapshot

Created a comprehensive constitution for the Physical AI & Humanoid Robotics course project, focusing on embodied intelligence principles. The constitution includes six core principles: Embodied Intelligence First, Simulation-to-Reality Pipeline, Test-First for Safety-Critical Systems, ROS 2 Architecture Patterns, Sensor Fusion and State Estimation, and Human-Robot Interaction Design. It also defines technical standards, development workflow, and governance procedures.

## Outcome

- ✅ Impact: Created foundational project constitution for robotics course
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with robotics-focused principles
- 🔁 Next prompts: Continue with spec, plan, or task creation for course materials
- 🧠 Reflection: Constitution establishes clear guidelines for embodied intelligence and ROS 2 development

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating course specifications based on constitution