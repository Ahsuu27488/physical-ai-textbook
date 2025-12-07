# Translation API Contract

## Translate Content
- **Endpoint**: `POST /api/v1/translation/translate`
- **Description**: Translate content to specified language (default Urdu)
- **Request Body**:
  ```json
  {
    "content": "string (content to translate)",
    "target_language": "string (default: ur for Urdu)"
  }
  ```
- **Response**:
  ```json
  {
    "original_content": "string (original content)",
    "translated_content": "string (translated content)",
    "target_language": "string (language code)"
  }
  ```
- **Status Codes**:
  - 200: Translation successful
  - 400: Invalid content or target language

## Translate Chapter Content (Authenticated)
- **Endpoint**: `POST /api/v1/translation/translate-chapter`
- **Description**: Translate entire chapter content (requires authentication for caching)
- **Headers**: `Authorization: Bearer {token}`
- **Request Body**:
  ```json
  {
    "content": "string (chapter content to translate)",
    "target_language": "string (default: ur for Urdu)"
  }
  ```
- **Response**:
  ```json
  {
    "original_content": "string (original content)",
    "translated_content": "string (translated content)",
    "target_language": "string (language code)"
  }
  ```
- **Status Codes**:
  - 200: Translation successful
  - 400: Invalid content or target language
  - 401: Unauthorized

## Translate to Urdu
- **Endpoint**: `POST /api/v1/translation/translate-to-urdu`
- **Description**: Convenience endpoint to translate content specifically to Urdu
- **Request Body**:
  ```json
  {
    "content": "string (content to translate)"
  }
  ```
- **Response**:
  ```json
  {
    "original_content": "string (original content)",
    "translated_content": "string (urdu translated content)",
    "target_language": "string (ur)"
  }
  ```
- **Status Codes**:
  - 200: Translation successful
  - 400: Invalid content

## Get Supported Languages
- **Endpoint**: `GET /api/v1/translation/supported-languages`
- **Description**: Get list of supported translation languages
- **Response**:
  ```json
  {
    "ur": "Urdu",
    "es": "Spanish",
    "fr": "French",
    "de": "German",
    "zh": "Chinese",
    "ja": "Japanese",
    "ar": "Arabic"
  }
  ```
- **Status Codes**:
  - 200: Languages list retrieved successfully

## Get Translation Cache
- **Endpoint**: `GET /api/v1/translation/cache`
- **Description**: Get cached translations (requires authentication)
- **Headers**: `Authorization: Bearer {token}`
- **Response**:
  ```json
  [
    {
      "id": "string (cache entry UUID)",
      "original_content_hash": "string (hash of original content)",
      "source_language": "string (default: en)",
      "target_language": "string (default: ur)",
      "created_at": "datetime"
    }
  ]
  ```
- **Status Codes**:
  - 200: Cache entries retrieved
  - 401: Unauthorized