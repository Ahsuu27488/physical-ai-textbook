<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles:
- Principle 1: [PRINCIPLE_1_NAME] → No Hallucination of Syntax
- Principle 2: [PRINCIPLE_2_NAME] → Agent Skill Creation First
- Principle 3: [PRINCIPLE_3_NAME] → Strict Dependency Management
- Principle 4: [PRINCIPLE_4_NAME] → Atomic Steps
- Principle 5: [PRINCIPLE_5_NAME] → Professional Aesthetic
- Principle 6: [PRINCIPLE_6_NAME] → MCP Server Utilization
Added sections: None
Removed sections: None
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Physical AI Textbook Constitution

## Core Principles

### No Hallucination of Syntax
You are strictly FORBIDDEN from writing configuration files (docusaurus.config.js, package.json, schema.prisma) based on training memory alone. You MUST use context7, Google Search, playwright or available browser tools to fetch the current documentation for: Docusaurus v3.9.2 (Installation & Configuration), OpenAI Agents SDK (Python/Node implementations), BetterAuth (Setup & Schema), Neon (Serverless Postgres connection), Qdrant (Cloud setup).

### Agent Skill Creation First
Before writing application code, you must create "Agent Skills" (reference markdown files in docs/skills/) for every major tool. You must read the docs, summarize the installation/config patterns, and save them. You will reference these files for all subsequent coding.

### Strict Dependency Management
Do not use npm install blindly. You must verify package names and versions. If an install fails, stop immediately, read the error, search for the error, and fix the package.json before retrying.

### Atomic Steps
Do not scaffold the whole app at once. Step 1: Install Docusaurus -> Verify it runs locally. Step 2: Add Tailwind -> Verify it runs. Step 3: Add BetterAuth -> Verify database connection. If a step fails, roll back and fix.

### Professional Aesthetic
When generating UI components or assets later, adhere to the "Modern and Professional Style" guideline (Better color themes, local context), but focus on technical correctness first.

### MCP Server Utilization
Always leverage the available MCP servers (Context7, GitHub, Playwright) when appropriate for your project needs. Use Context7 for documentation and library information, GitHub for repository management, and Playwright for browser automation when needed.

## Additional Constraints

Technology stack requirements: Docusaurus v3.9.2, Tailwind CSS, BetterAuth, Neon PostgreSQL, Qdrant vector database, OpenAI Agents SDK. All implementations must follow the atomic step approach with proper verification at each stage.

## Development Workflow

All development must follow the critical directives: verify syntax with documentation sources, create agent skills before coding, manage dependencies strictly, implement in atomic steps, and maintain professional aesthetic standards. Each change must be validated before proceeding to the next step.

## Governance

All development practices must adhere to these principles. Changes to this constitution require documentation, approval, and appropriate migration planning. All code reviews must verify compliance with these principles.

**Version**: 1.1.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06