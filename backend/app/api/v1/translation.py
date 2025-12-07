from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict
from ...database.database import get_db
from ...models import TranslationRequest, TranslationResponse
from ...services.translation import TranslationService
from ...auth.utils import verify_token
from fastapi.security import OAuth2PasswordBearer

# Create router
router = APIRouter()

# OAuth2 scheme for token
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/v1/auth/token")

# Initialize service
translation_service = TranslationService()


def get_current_user_id_optional(token: str = Depends(oauth2_scheme)) -> str:
    """Optional dependency to get the current user ID from the token."""
    try:
        credentials_exception = HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

        token_data = verify_token(token)
        if token_data is None:
            raise credentials_exception

        user_id = token_data.get("user_id")
        if user_id is None:
            raise credentials_exception

        return user_id
    except:
        # If token is invalid, return None but allow public translation
        return None


@router.post("/translate", response_model=TranslationResponse)
async def translate_content(translation_request: TranslationRequest, db: Session = Depends(get_db)):
    """Translate content to the target language (default: Urdu)."""
    return translation_service.translate_content(db, translation_request)


@router.get("/supported-languages")
async def get_supported_languages():
    """Get a list of supported languages for translation."""
    return translation_service.get_supported_languages()


@router.post("/translate-chapter")
async def translate_chapter(content: str, target_language: str = "ur", current_user_id: str = Depends(get_current_user_id_optional), db: Session = Depends(get_db)):
    """Translate an entire chapter to the target language."""
    # Note: We're not using the user ID here, but we could in the future to track translation usage
    translated_content = translation_service.translate_chapter(db, content, target_language)

    return TranslationResponse(
        original_content=content,
        translated_content=translated_content,
        target_language=target_language
    )


@router.post("/translate-to-urdu")
async def translate_to_urdu(content: str, db: Session = Depends(get_db)):
    """Translate content specifically to Urdu."""
    translation_request = TranslationRequest(
        content=content,
        target_language="ur"
    )

    return translation_service.translate_content(db, translation_request)