"""Translate a lesson page on demand: signed-in learners only, since it spends the same free model quota as Rivet."""
from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session

from ...api.chat import signed_in_learner
from ...database.database import get_db
from ...llm import _require_key
from ...models import TranslationRequest, TranslationResponse
from ...services.translation import TranslationService, supported_languages

router = APIRouter()
translation_service = TranslationService()


@router.get("/supported-languages")
async def get_supported_languages():
    """[{code, name, native, rtl}] in display order."""
    return supported_languages()


@router.post("/translate", response_model=TranslationResponse, dependencies=[Depends(signed_in_learner)])
def translate_content(request: TranslationRequest, db: Session = Depends(get_db)):
    """Translate page text (Markdown) into one of the supported languages; cached per text and language."""
    _require_key()  # 503 when GEMINI_API_KEY is not set
    return translation_service.translate_content(db, request)
