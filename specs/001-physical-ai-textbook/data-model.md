# Data Model: Physical AI & Humanoid Robotics Textbook Platform

## User Entity
- **id**: string (UUID) - Unique identifier for the user
- **email**: string - User's email address (unique, validated)
- **name**: string - User's full name
- **password_hash**: string - Bcrypt hash of user password
- **created_at**: datetime - Account creation timestamp
- **updated_at**: datetime - Last update timestamp
- **preferences**: JSON - User preferences including personalization settings
- **is_active**: boolean - Account status flag

## Textbook Chapter Entity
- **id**: string (UUID) - Unique identifier for the chapter
- **title**: string - Chapter title
- **content**: text - Chapter content in markdown format
- **content_urdu**: text - Chapter content translated to Urdu (optional)
- **slug**: string - URL-friendly identifier for the chapter
- **order**: integer - Chapter sequence in the textbook
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp
- **metadata**: JSON - Additional chapter metadata (tags, keywords, etc.)

## Personalization Preference Entity
- **id**: string (UUID) - Unique identifier for the preference
- **user_id**: string - Reference to the user (foreign key)
- **chapter_id**: string - Reference to the chapter (foreign key)
- **preferences**: JSON - Personalization settings for this chapter
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp

## Chatbot Session Entity
- **id**: string (UUID) - Unique identifier for the session
- **user_id**: string - Reference to the user (foreign key, optional for anonymous users)
- **session_token**: string - Session identifier for anonymous users
- **created_at**: datetime - Session start timestamp
- **updated_at**: datetime - Last interaction timestamp
- **conversation_history**: JSON - Array of conversation turns

## Chat Message Entity
- **id**: string (UUID) - Unique identifier for the message
- **session_id**: string - Reference to the chat session (foreign key)
- **user_id**: string - Reference to the user (foreign key, optional)
- **role**: string - 'user' or 'assistant'
- **content**: text - Message content
- **timestamp**: datetime - Message creation timestamp
- **context_used**: JSON - Vector references used for RAG response (optional)

## Relationships
- User has many Personalization Preferences (one-to-many)
- User has many Chatbot Sessions (one-to-many)
- Textbook Chapter has many Personalization Preferences (one-to-many)
- Chatbot Session has many Chat Messages (one-to-many)

## Validation Rules
- User email must be unique and properly formatted
- Chapter slug must be unique across all chapters
- Personalization preferences require either authenticated user or valid session
- Content fields support markdown formatting
- Timestamps are automatically managed by the system