# Data Model: AI Subagents, User Authentication, Personalization & Urdu Translation

## User Entity
- **id**: string (UUID) - Unique identifier for the user
- **email**: string - User's email address (unique, validated)
- **name**: string - User's full name
- **password_hash**: string - Bcrypt hash of user password
- **created_at**: datetime - Account creation timestamp
- **updated_at**: datetime - Last update timestamp
- **software_background**: string - User's software experience level (beginner/intermediate/advanced)
- **hardware_background**: string - User's hardware experience level (beginner/intermediate/advanced)
- **is_active**: boolean - Account status flag

## Personalization Preference Entity
- **id**: string (UUID) - Unique identifier for the preference
- **user_id**: string - Reference to the user (foreign key)
- **chapter_id**: string - Reference to the chapter (foreign key)
- **preferences**: JSON - Personalization settings for this chapter
  - difficulty_level: string (beginner/intermediate/advanced)
  - include_examples: boolean
  - focus_on_code: boolean
  - focus_on_theory: boolean
  - software_background: string (beginner/intermediate/advanced)
  - hardware_background: string (beginner/intermediate/advanced)
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp

## Translation Cache Entity
- **id**: string (UUID) - Unique identifier for the cache entry
- **original_content_hash**: string - Hash of original content (unique, for quick lookup)
- **original_content**: text - Original content that was translated
- **translated_content**: text - Content translated to Urdu
- **source_language**: string - Source language (default: "en")
- **target_language**: string - Target language (default: "ur" for Urdu)
- **created_at**: datetime - Cache entry creation timestamp

## Agent Skill Entity
- **id**: string (UUID) - Unique identifier for the agent skill
- **name**: string - Name of the agent skill
- **description**: text - Description of what the skill does
- **functionality**: JSON - Configuration for the agent skill
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp

## Chapter Entity
- **id**: string (UUID) - Unique identifier for the chapter
- **title**: string - Chapter title
- **content**: text - Chapter content in markdown format
- **content_urdu**: text - Chapter content translated to Urdu (optional, cached)
- **slug**: string - URL-friendly identifier for the chapter
- **order**: integer - Chapter sequence in the textbook
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp
- **metadata**: JSON - Additional chapter metadata (tags, keywords, etc.)

## Relationships
- User has many Personalization Preferences (one-to-many)
- User has many Translation Cache entries (one-to-many, via content creation)
- Chapter has many Personalization Preferences (one-to-many)
- Agent Skill can be associated with multiple user interactions (many-to-many, through usage logs)

## Validation Rules
- User email must be unique and properly formatted
- Chapter slug must be unique across all chapters
- Personalization preferences require authenticated user
- Translation cache entries are immutable once created (to ensure consistency)
- Content fields support markdown formatting
- Timestamps are automatically managed by the system
- Agent skills must have unique names within the system