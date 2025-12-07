# Integration Instructions: Personalization and Translation Features

This document provides instructions for integrating the personalization and translation features into the Physical AI & Humanoid Robotics Textbook Platform.

## Overview

The platform includes two key interactive features:
1. **Personalize Chapter Button**: Allows users to customize textbook content based on their learning preferences
2. **Translate to Urdu Button**: Translates textbook content to Urdu for multilingual support

## Frontend Components

### PersonalizeButton Component
- **Location**: `frontend/src/components/PersonalizeButton.js`
- **Purpose**: Provides UI for users to customize textbook content based on preferences
- **Features**:
  - Difficulty level selection (beginner/advanced)
  - Option to include more examples
  - Focus area selection (code vs. theory)
  - Background information customization

### TranslateButton Component
- **Location**: `frontend/src/components/TranslateButton.js`
- **Purpose**: Translates textbook content to Urdu or other supported languages
- **Features**:
  - Translation to Urdu by default
  - Support for other languages
  - Toggle to show/hide translated content

### Auth Hook
- **Location**: `frontend/src/hooks/useAuth.js`
- **Purpose**: Manages user authentication state in Docusaurus
- **Features**:
  - User login/logout functionality
  - Token management
  - User state persistence

## Integration Instructions

### Adding Buttons to Textbook Chapters

To add the personalization and translation buttons to any Docusaurus page, you need to ensure the AuthProvider is available. The recommended approach is to wrap the buttons with the AuthProvider:

```md
import React from 'react';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';
import { AuthProvider } from '@site/src/hooks/useAuth';

<div style={{marginTop: '2rem', padding: '1rem', backgroundColor: '#f8f9fa', borderRadius: '8px', border: '1px solid #e9ecef'}}>
  <AuthProvider>
    <PersonalizeButton
      chapterId="unique-chapter-identifier"
      content={`Paste the full chapter content here as a template string`}
    />

    <TranslateButton
      content={`Paste the full chapter content here as a template string`}
      targetLanguage="ur"
    />
  </AuthProvider>
</div>
```

### Chapter ID Guidelines
- Use a unique identifier for each chapter
- Recommended format: `module-{number}-{lesson-name}` (e.g., `module-1-lesson-1-ros2-architecture`)
- Keep IDs URL-friendly (use hyphens instead of spaces)

### Content Template Guidelines
- Wrap the entire chapter content in a template string (backticks)
- Escape any existing backticks in the content with `\`
- Include all content that should be personalized or translated

## Backend API Endpoints

### Authentication Endpoints
- `POST /api/v1/auth/register` - User registration
- `POST /api/v1/auth/token` - User login
- `GET /api/v1/auth/me` - Get current user info
- `PUT /api/v1/auth/profile` - Update user profile

### Personalization Endpoints
- `GET /api/v1/personalization/preference/{chapter_id}` - Get preferences for a chapter
- `POST /api/v1/personalization/preference` - Create/update preferences
- `GET /api/v1/personalization/preferences` - Get all user preferences
- `DELETE /api/v1/personalization/preference/{chapter_id}` - Delete preferences
- `POST /api/v1/personalization/apply/{chapter_id}` - Apply personalization to content

### Translation Endpoints
- `POST /api/v1/translation/translate` - Translate content to any language
- `GET /api/v1/translation/supported-languages` - Get supported languages
- `POST /api/v1/translation/translate-chapter` - Translate entire chapter
- `POST /api/v1/translation/translate-to-urdu` - Translate to Urdu

## Environment Variables

Make sure the following environment variables are set in your `.env` file:

```env
OPENAI_API_KEY=your_openai_api_key
SECRET_KEY=your_secret_key_for_jwt
ACCESS_TOKEN_EXPIRE_MINUTES=30
NEON_DATABASE_URL=your_database_url  # or leave empty to use SQLite for testing
```

## Running the Applications

### Frontend (Docusaurus)
```bash
cd frontend
npm install  # if dependencies haven't been installed
npm run start
```
The frontend will be available at `http://localhost:3001/physical-ai-textbook/`

### Backend (FastAPI)
```bash
cd backend
source venv/bin/activate  # if using virtual environment
pip install -r requirements.txt  # if dependencies haven't been installed
uvicorn app.main:app --reload
```
The backend API will be available at `http://localhost:8000`

## Testing the Features

1. **Authentication**: Test user registration and login functionality
2. **Personalization**: Add preferences and verify content customization
3. **Translation**: Verify content translation to Urdu works correctly
4. **Integration**: Test that frontend components communicate properly with backend APIs

## Troubleshooting

### Common Issues

1. **Buttons not appearing**: Ensure the component imports are correct and placed in the right location in your markdown file
2. **API connection errors**: Verify that the backend server is running and the frontend can reach it
3. **Authentication errors**: Check that the JWT token is being properly handled
4. **Translation not working**: Verify that the OpenAI API key is properly configured

### API Documentation
- Backend API documentation is available at `http://localhost:8000/docs` when the server is running
- Use the Swagger UI to test API endpoints directly

## Security Considerations

- JWT tokens are used for authentication
- Passwords are hashed using bcrypt
- API endpoints are protected with proper authentication
- Environment variables should not be committed to version control