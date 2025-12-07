---
id: 1
title: Update project constitution with MCP directives
stage: constitution
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: Ahsan
command: /sp.constitution
labels: ["constitution", "governance", "principles", "mcp"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - none
---

## Prompt

```
CRITICAL DIRECTIVES FOR EXECUTION:

NO HALLUCINATION OF SYNTAX: You are strictly FORBIDDEN from writing configuration files (docusaurus.config.js, package.json, schema.prisma) based on training memory alone. You MUST use context7, Google Search,playwright or available browser tools to fetch the current documentation for:
Docusaurus v3.9.2 (Installation & Configuration)
OpenAI Agents SDK (Python/Node implementations)
BetterAuth (Setup & Schema)
Neon (Serverless Postgres connection)
Qdrant (Cloud setup)

AGENT SKILL CREATION FIRST: Before writing application code, you must create "Agent Skills" (reference markdown files in docs/skills/) for every major tool. You must read the docs, summarize the installation/config patterns, and save them. You will reference these files for all subsequent coding.
STRICT DEPENDENCY MANAGEMENT: Do not use npm install blindly. You must verify package names and versions. If an install fails, stop immediately, read the error, search for the error, and fix the package.json before retrying.

ATOMIC STEPS: Do not scaffold the whole app at once.
Step 1: Install Docusaurus -> Verify it runs locally.
Step 2: Add Tailwind -> Verify it runs.
Step 3: Add BetterAuth -> Verify database connection.
If a step fails, roll back and fix.
PROFESIONAL AESTHETIC: When generating UI components or assets later, adhere to the "Modern and Professional Style" guideline (Better color themes, local context), but focus on technical correctness first.
```

## Response snapshot

Updated the project constitution to include the critical directives as core principles:
- No Hallucination of Syntax
- Agent Skill Creation First
- Strict Dependency Management
- Atomic Steps
- Professional Aesthetic
- MCP Server Utilization

Also updated the template files to align with these principles:
- Modified plan-template.md to include compliance gates
- Modified tasks-template.md to include agent skills creation task
- Modified spec-template.md to include documentation requirements

## Outcome

- ✅ Impact: Constitution updated with 6 core principles reflecting user's critical directives and MCP server utilization
- 🧪 Tests: none
- 📁 Files: 4 files updated (.specify/memory/constitution.md, .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md)
- 🔁 Next prompts: Continue with project implementation following the new constitution
- 🧠 Reflection: MCP servers were confirmed to be working and incorporated into the constitution as a principle

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Begin implementing features following the new constitution