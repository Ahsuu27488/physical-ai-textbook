"""Re-explain a lesson page for one learner's background with the chat model (Gemini), cached per level pair."""
from hashlib import md5

from fastapi import HTTPException
from sqlalchemy.orm import Session

from ..auth.utils import generate_translation_cache_id
from ..config import get_settings
from ..database.models import TranslationCache as CacheModel
from ..llm import get_openai_client

LEVELS = {"beginner", "intermediate", "advanced"}

SYSTEM = """You are Rivet, the tutor of Kinemata, a course in robotics and Physical AI. Re-explain the lesson page the \
learner sends so it fits them. Their software background: {software}. Their hardware/electronics background: {hardware}.
- Beginner in an area: plain words, an everyday analogy, define each term on first use, smaller steps.
- Intermediate: keep the precision, skip the very basics, connect ideas to things they likely know.
- Advanced: be concise and dense, add the deeper "why", trade-offs and pitfalls an expert would care about.
- Keep every fact correct and every code block and formula unchanged (you may add short comments to code).
- Keep the page's structure (headings, lists) so it can be read side by side with the original.
- Start with one line "**Tuned for you:** ..." naming what you adjusted. Use Markdown. No other preface."""


def _level(value: str) -> str:
    value = (value or "").strip().lower()
    return value if value in LEVELS else "beginner"


def rewrite(db: Session, content: str, page_title: str, software: str, hardware: str) -> str:
    software, hardware = _level(software), _level(hardware)
    key = f"personal:{software}/{hardware}"
    content_hash = md5(content.encode()).hexdigest()
    cached = db.query(CacheModel).filter(CacheModel.original_content_hash == content_hash, CacheModel.target_language == key).first()
    if cached:
        return cached.translated_content
    try:
        response = get_openai_client().chat.completions.create(
            model=get_settings().chat_model,
            messages=[
                {"role": "system", "content": SYSTEM.format(software=software, hardware=hardware)},
                {"role": "user", "content": (f"# {page_title}\n\n" if page_title else "") + content},
            ],
            max_tokens=8192,
            temperature=0.4,
        )
        text = (response.choices[0].message.content or "").strip()
    except Exception as exc:
        if getattr(exc, "status_code", None) == 429:
            raise HTTPException(429, "The AI service is busy or out of free quota. Try again in a minute.")
        raise HTTPException(502, "Personalizing failed. Try again.")
    if not text:
        raise HTTPException(502, "The rewrite came back empty. Try again.")
    db.add(CacheModel(id=generate_translation_cache_id(), original_content_hash=content_hash,
                      original_content=content, translated_content=text, target_language=key))
    db.commit()
    return text
