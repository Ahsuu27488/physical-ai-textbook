# Personalization Subagent

## Purpose
Processes personalization preferences and applies content customization based on user background and preferences.

## Responsibilities
- Apply personalization rules to content based on user preferences
- Adjust difficulty level, include examples, focus on code vs theory
- Use user's software and hardware background for customization
- Store and retrieve personalization preferences

## Implementation
The PersonalizationSubagent uses the personalization engine to transform content based on user preferences and background information.

## Coordination
- Receives user data from AuthSubagent
- Works with DatabaseSubagent for preference storage
- Interfaces with ContentSubagent for content delivery

## Agent Skills Used
- Personalization Engine Skill
- Database Operations Skill