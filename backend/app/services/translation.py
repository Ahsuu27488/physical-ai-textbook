from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from typing import Dict, Any
from ..database.models import TranslationCache as CacheModel
from ..models import TranslationRequest, TranslationResponse
from hashlib import md5
import openai
from openai import OpenAI
import os
from dotenv import load_dotenv
from ..auth.utils import generate_translation_cache_id
import json

# Load environment variables
load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


class TranslationService:
    """Service class for handling translations, particularly to Urdu."""

    def __init__(self):
        self.supported_languages = {
            "ur": "Urdu",
            "es": "Spanish",
            "fr": "French",
            "de": "German",
            "zh": "Chinese",
            "ja": "Japanese",
            "ar": "Arabic",
        }

    def translate_content(self, db: Session, translation_request: TranslationRequest) -> TranslationResponse:
        """Translate content to the target language using OpenAI API."""
        target_language = translation_request.target_language

        # Validate target language
        if target_language not in self.supported_languages:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Translation to '{target_language}' is not supported. Supported languages: {list(self.supported_languages.keys())}"
            )

        # Create a hash of the original content to use for caching
        content_hash = md5(translation_request.content.encode()).hexdigest()

        # Check if translation is already cached
        cached_translation = db.query(CacheModel).filter(
            CacheModel.original_content_hash == content_hash,
            CacheModel.target_language == target_language
        ).first()

        if cached_translation:
            return TranslationResponse(
                original_content=cached_translation.original_content,
                translated_content=cached_translation.translated_content,
                target_language=target_language
            )

        # If not cached, perform translation using OpenAI
        try:
            # Construct the translation prompt
            language_name = self.supported_languages[target_language]
            prompt = f"""Translate the following text to {language_name}.
            Preserve the technical terminology and meaning as accurately as possible.
            If it contains code, formulas, or technical terms, keep them as they are or provide appropriate transliterations.

            Text to translate:
            {translation_request.content}"""

            response = client.chat.completions.create(
                model="gpt-4o-mini",  # Using gpt-4o-mini as per previous configuration
                messages=[
                    {"role": "system", "content": f"You are a professional translator specialized in technical content. Translate accurately to {language_name}."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=2000,
                temperature=0.3  # Lower temperature for more consistent translations
            )

            translated_content = response.choices[0].message.content

            # Cache the translation
            cache_entry = CacheModel(
                id=generate_translation_cache_id(),
                original_content_hash=content_hash,
                original_content=translation_request.content,
                translated_content=translated_content,
                target_language=target_language
            )

            db.add(cache_entry)
            db.commit()

            return TranslationResponse(
                original_content=translation_request.content,
                translated_content=translated_content,
                target_language=target_language
            )

        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_ERROR,
                detail=f"Translation failed: {str(e)}"
            )

    def get_supported_languages(self) -> Dict[str, str]:
        """Get a list of supported languages for translation."""
        return self.supported_languages

    def translate_chapter(self, db: Session, chapter_content: str, target_language: str = "ur") -> str:
        """Translate an entire chapter to the target language."""
        translation_request = TranslationRequest(
            content=chapter_content,
            target_language=target_language
        )

        response = self.translate_content(db, translation_request)
        return response.translated_content