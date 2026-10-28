"""Translate a lesson page with the chat model (Gemini by default), cached per text and language."""
from hashlib import md5
from typing import Dict, List

from fastapi import HTTPException, status
from sqlalchemy.orm import Session

from ..auth.utils import generate_translation_cache_id
from ..database.models import TranslationCache as CacheModel
from ..llm import complete
from ..models import TranslationRequest, TranslationResponse

# code: (English name, name in that language, right-to-left)
LANGUAGES: Dict[str, tuple] = {
    "ur": ("Urdu", "اردو", True),
    "hi": ("Hindi", "हिन्दी", False),
    "bn": ("Bengali", "বাংলা", False),
    "ar": ("Arabic", "العربية", True),
    "fa": ("Persian", "فارسی", True),
    "tr": ("Turkish", "Türkçe", False),
    "zh": ("Chinese (Simplified)", "简体中文", False),
    "ja": ("Japanese", "日本語", False),
    "ko": ("Korean", "한국어", False),
    "id": ("Indonesian", "Bahasa Indonesia", False),
    "vi": ("Vietnamese", "Tiếng Việt", False),
    "ru": ("Russian", "Русский", False),
    "es": ("Spanish", "Español", False),
    "pt": ("Portuguese (Brazil)", "Português", False),
    "fr": ("French", "Français", False),
    "de": ("German", "Deutsch", False),
    "it": ("Italian", "Italiano", False),
}

SYSTEM = """You translate lessons of Kinemata, a course in robotics and Physical AI, for learners who read {name} \
better than English. Rules:
- Translate everything a reader reads into natural, clear {name}; keep the meaning exact and the tone friendly.
- Keep Markdown structure as it is: headings, lists, tables, bold, links (translate link text, never URLs).
- Never translate or change code, commands, file names, package names, ROS topic names, API names, inline code or
  math ($...$, $$...$$). Keep established technical terms (ROS 2, URDF, LiDAR, PID, Kalman filter, SLAM...) in
  English; the first time one appears you may add a short {name} gloss in parentheses.
- Reply with the translation only: no preface, no notes."""


def supported_languages() -> List[dict]:
    return [{"code": c, "name": n, "native": native, "rtl": rtl} for c, (n, native, rtl) in LANGUAGES.items()]


class TranslationService:
    def translate_content(self, db: Session, request: TranslationRequest) -> TranslationResponse:
        lang = request.target_language
        if lang not in LANGUAGES:
            raise HTTPException(status.HTTP_400_BAD_REQUEST, f"'{lang}' is not supported. Supported: {', '.join(LANGUAGES)}")
        content_hash = md5(request.content.encode()).hexdigest()
        cached = (
            db.query(CacheModel)
            .filter(CacheModel.original_content_hash == content_hash, CacheModel.target_language == lang)
            .first()
        )
        if cached:
            return TranslationResponse(original_content=cached.original_content,
                                       translated_content=cached.translated_content, target_language=lang)

        name = LANGUAGES[lang][0]
        try:
            response = complete(lambda model: dict(
                model=model,
                messages=[
                    {"role": "system", "content": SYSTEM.format(name=name)},
                    {"role": "user", "content": request.content},
                ],
                max_tokens=8192,
                temperature=0.2,
            ))
            translated = (response.choices[0].message.content or "").strip()
        except HTTPException:
            raise
        except Exception as exc:
            code = getattr(exc, "status_code", None)
            if code == 429:
                raise HTTPException(429, "The translation service is busy or out of free quota. Try again in a minute.")
            raise HTTPException(502, "The translation service failed. Try again.")
        if not translated:
            raise HTTPException(502, "The translation came back empty. Try again.")

        db.add(CacheModel(id=generate_translation_cache_id(), original_content_hash=content_hash,
                          original_content=request.content, translated_content=translated, target_language=lang))
        db.commit()
        return TranslationResponse(original_content=request.content, translated_content=translated, target_language=lang)
