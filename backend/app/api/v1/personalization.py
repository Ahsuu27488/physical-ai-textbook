from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List
from ...database.database import get_db
from ...models import PersonalizationPreference, PersonalizationPreferenceCreate
from ...services.personalization import PersonalizationService
from ...auth.service import AuthService
from ...auth.utils import verify_token
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel, Field

from ...api.chat import signed_in_learner
from ...database.models import User as UserModel
from ...llm import _require_key

# Create router
router = APIRouter()

# OAuth2 scheme for token
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/v1/auth/token")

# Initialize services
personalization_service = PersonalizationService()
auth_service = AuthService()


def get_current_user_id(token: str = Depends(oauth2_scheme)) -> str:
    """Dependency to get the current user ID from the token."""
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


@router.get("/preference/{chapter_id}", response_model=PersonalizationPreference)
async def get_chapter_preference(chapter_id: str, current_user_id: str = Depends(get_current_user_id), db: Session = Depends(get_db)):
    """Get personalization preferences for a specific chapter."""
    return personalization_service.get_preference(db, current_user_id, chapter_id)


@router.get("/preferences", response_model=List[PersonalizationPreference])
async def get_all_preferences(current_user_id: str = Depends(get_current_user_id), db: Session = Depends(get_db)):
    """Get all personalization preferences for the current user."""
    return personalization_service.get_all_preferences(db, current_user_id)


@router.post("/preference", response_model=PersonalizationPreference)
async def create_or_update_preference(preference_create: PersonalizationPreferenceCreate, current_user_id: str = Depends(get_current_user_id), db: Session = Depends(get_db)):
    """Create or update personalization preferences for a chapter."""
    return personalization_service.create_preference(db, current_user_id, preference_create)


@router.delete("/preference/{chapter_id}")
async def delete_preference(chapter_id: str, current_user_id: str = Depends(get_current_user_id), db: Session = Depends(get_db)):
    """Delete personalization preferences for a chapter."""
    success = personalization_service.delete_preference(db, current_user_id, chapter_id)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Preference not found"
        )
    return {"message": "Preference deleted successfully"}


class RewriteRequest(BaseModel):
    content: str = Field(..., min_length=1, max_length=12000)
    page_title: str = Field("", max_length=300)


@router.post("/rewrite", dependencies=[Depends(signed_in_learner)])
def rewrite_for_learner(request: RewriteRequest, current_user_id: str = Depends(get_current_user_id), db: Session = Depends(get_db)):
    """Re-explain a lesson page for this learner's software and hardware background (Gemini), cached per level."""
    from ...services.personalize_ai import rewrite

    _require_key()
    user = db.query(UserModel).filter(UserModel.id == current_user_id).first()
    software = (user.software_background if user else "") or "beginner"
    hardware = (user.hardware_background if user else "") or "beginner"
    text = rewrite(db, request.content, request.page_title, software, hardware)
    return {"personalized_content": text, "software_background": software, "hardware_background": hardware}
