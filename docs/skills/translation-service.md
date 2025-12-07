# Translation Service Skill

## Purpose
This skill handles content translation, particularly to Urdu, with caching for performance optimization.

## Functionality
- Translate content to specified languages (default Urdu)
- Implement caching mechanism to avoid repeated API calls
- Handle translation errors gracefully
- Support for multiple target languages

## Implementation Pattern
```python
import openai
from typing import Dict, Optional
import hashlib

class TranslationService:
    def __init__(self, openai_client):
        self.client = openai_client
        self.cache = {}  # In production, use Redis or database

    def translate_content(self, content: str, target_language: str = "ur") -> str:
        # Create cache key
        cache_key = hashlib.md5(f"{content}_{target_language}".encode()).hexdigest()

        # Check cache first
        if cache_key in self.cache:
            return self.cache[cache_key]

        # Perform translation
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": f"You are a professional translator. Translate the following content to {target_language}."},
                {"role": "user", "content": content}
            ]
        )

        translated_content = response.choices[0].message.content

        # Cache the result
        self.cache[cache_key] = translated_content

        return translated_content

    def get_supported_languages(self) -> Dict[str, str]:
        return {
            "ur": "Urdu",
            "es": "Spanish",
            "fr": "French",
            "de": "German",
            "zh": "Chinese",
            "ja": "Japanese",
            "ar": "Arabic"
        }
```

## Usage Examples
1. Initialize translation service with OpenAI client
2. Call translate_content for content translation
3. Use caching to improve performance
4. Handle multiple language support