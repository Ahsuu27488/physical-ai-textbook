# Auth Subagent

## Purpose
Handles user authentication workflows including registration, login, and profile management with background information collection.

## Responsibilities
- Process user registration requests with software/hardware background
- Authenticate users and manage sessions
- Update user profile information
- Coordinate with database operations

## Implementation
The AuthSubagent uses the BetterAuth integration service to handle authentication while maintaining compatibility with our custom background collection requirements.

## Coordination
- Works with DatabaseSubagent for user data persistence
- Communicates with PersonalizationSubagent to apply user preferences
- Interfaces with ContentSubagent for protected content access

## Agent Skills Used
- BetterAuth Integration Skill
- Database Operations Skill