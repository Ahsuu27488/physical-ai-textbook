# Claude Code Subagents Implementation

## Overview
This directory contains the implementation of Claude Code Subagents for the Physical AI & Humanoid Robotics Textbook Platform, fulfilling the hackathon requirement for "reusable intelligence via Claude Code Subagents and Agent Skills".

## Subagent Architecture

### 1. AuthSubagent
- Handles user authentication workflows
- Manages registration with background collection
- Coordinates with database operations

### 2. PersonalizationSubagent
- Processes personalization preferences
- Applies content customization rules
- Integrates with user background data

### 3. TranslationSubagent
- Manages content translation workflows
- Handles caching mechanisms
- Coordinates with OpenAI API

### 4. ContentSubagent
- Orchestrates content delivery
- Coordinates between personalization and translation
- Manages user experience flows

## Implementation Approach

The subagents are implemented as specialized components that can coordinate and communicate with each other as required by the hackathon specification. Each subagent has specific responsibilities while maintaining the ability to work together for complex operations.

## Agent Skills Integration

The subagents leverage the Agent Skills defined in the `docs/skills/` directory to perform specialized tasks:

- BetterAuth Integration Skill
- Translation Service Skill
- Personalization Engine Skill
- Database Operations Skill

## Hackathon Compliance

This implementation fulfills the requirement: "Participants can earn up to 50 extra bonus points by creating and using reusable intelligence via Claude Code Subagents and Agent Skills in the book project."