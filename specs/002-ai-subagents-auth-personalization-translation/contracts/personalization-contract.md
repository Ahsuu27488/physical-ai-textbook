# Personalization API Contract

## Get Chapter Preferences
- **Endpoint**: `GET /api/v1/personalization/preference/{chapter_id}`
- **Description**: Get personalization preferences for a specific chapter
- **Headers**: `Authorization: Bearer {token}`
- **Path Parameters**: `chapter_id` - ID of the chapter
- **Response**:
  ```json
  {
    "id": "string (preference UUID)",
    "user_id": "string (user UUID)",
    "chapter_id": "string (chapter ID)",
    "preferences": {
      "difficulty_level": "string (beginner/intermediate/advanced)",
      "include_examples": "boolean",
      "focus_on_code": "boolean",
      "focus_on_theory": "boolean",
      "software_background": "string",
      "hardware_background": "string"
    },
    "created_at": "datetime",
    "updated_at": "datetime"
  }
  ```
- **Status Codes**:
  - 200: Preferences retrieved
  - 401: Unauthorized
  - 404: Preferences not found

## Create/Update Chapter Preferences
- **Endpoint**: `POST /api/v1/personalization/preference`
- **Description**: Create or update personalization preferences for a chapter
- **Headers**: `Authorization: Bearer {token}`
- **Request Body**:
  ```json
  {
    "chapter_id": "string (chapter ID)",
    "preferences": {
      "difficulty_level": "string (beginner/intermediate/advanced)",
      "include_examples": "boolean",
      "focus_on_code": "boolean",
      "focus_on_theory": "boolean",
      "software_background": "string",
      "hardware_background": "string"
    }
  }
  ```
- **Response**: Same as Get Chapter Preferences
- **Status Codes**:
  - 200: Preferences created/updated
  - 401: Unauthorized

## Get All User Preferences
- **Endpoint**: `GET /api/v1/personalization/preferences`
- **Description**: Get all personalization preferences for the current user
- **Headers**: `Authorization: Bearer {token}`
- **Response**:
  ```json
  [
    {
      "id": "string (preference UUID)",
      "user_id": "string (user UUID)",
      "chapter_id": "string (chapter ID)",
      "preferences": {
        "difficulty_level": "string (beginner/intermediate/advanced)",
        "include_examples": "boolean",
        "focus_on_code": "boolean",
        "focus_on_theory": "boolean",
        "software_background": "string",
        "hardware_background": "string"
      },
      "created_at": "datetime",
      "updated_at": "datetime"
    }
  ]
  ```
- **Status Codes**:
  - 200: Preferences retrieved
  - 401: Unauthorized

## Apply Personalization to Content
- **Endpoint**: `POST /api/v1/personalization/apply/{chapter_id}`
- **Description**: Apply personalization to content based on user preferences
- **Headers**: `Authorization: Bearer {token}`
- **Path Parameters**: `chapter_id` - ID of the chapter
- **Request Body**:
  ```json
  {
    "content": "string (content to personalize)",
    "chapter_id": "string (chapter ID)"
  }
  ```
- **Response**:
  ```json
  {
    "original_content": "string (original content)",
    "personalized_content": "string (personalized content)",
    "preferences_applied": "object (preferences that were applied)"
  }
  ```
- **Status Codes**:
  - 200: Content personalized
  - 401: Unauthorized