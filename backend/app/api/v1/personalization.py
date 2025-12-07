from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List
from ...database.database import get_db
from ...models import PersonalizationPreference, PersonalizationPreferenceCreate
from ...services.personalization import PersonalizationService
from ...auth.service import AuthService
from ...auth.utils import verify_token
from fastapi.security import OAuth2PasswordBearer

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


@router.post("/apply/{chapter_id}")
async def apply_personalization(content: str, chapter_id: str, current_user_id: str = Depends(get_current_user_id), db: Session = Depends(get_db)):
    """Apply personalization to content based on user preferences."""
    # Get user preferences for this chapter
    try:
        preference = personalization_service.get_preference(db, current_user_id, chapter_id)
        preferences = preference.preferences
    except HTTPException:
        # If no preference exists, use default empty preferences
        preferences = {}

    # Apply personalization to the content
    personalized_content = personalization_service.apply_personalization(content, preferences)

    return {
        "original_content": content,
        "personalized_content": personalized_content,
        "preferences_applied": preferences
    }