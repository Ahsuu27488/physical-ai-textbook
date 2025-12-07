# Translation Subagent

## Purpose
Manages content translation workflows, particularly to Urdu, with caching for performance optimization.

## Responsibilities
- Translate content to specified languages (default Urdu)
- Implement caching mechanism to avoid repeated API calls
- Handle translation errors gracefully
- Support for multiple target languages

## Implementation
The TranslationSubagent uses the translation service to convert content while implementing caching for performance.

## Coordination
- Works with DatabaseSubagent for translation caching
- Interfaces with ContentSubagent for content processing
- May receive user preferences from PersonalizationSubagent for language preferences

## Agent Skills Used
- Translation Service Skill
- Database Operations Skill