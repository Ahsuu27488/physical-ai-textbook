from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from typing import Dict, Any, List
from ..database.models import PersonalizationPreference as PreferenceModel
from ..models import PersonalizationPreference, PersonalizationPreferenceCreate
from ..auth.utils import generate_preference_id
import json


class PersonalizationService:
    """Service class for handling personalization preferences."""

    def get_preference(self, db: Session, user_id: str, chapter_id: str) -> PersonalizationPreference:
        """Get personalization preferences for a user and chapter."""
        preference = db.query(PreferenceModel).filter(
            PreferenceModel.user_id == user_id,
            PreferenceModel.chapter_id == chapter_id
        ).first()

        if not preference:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Personalization preference not found"
            )

        return PersonalizationPreference(
            id=preference.id,
            user_id=preference.user_id,
            chapter_id=preference.chapter_id,
            preferences=preference.preferences,
            created_at=preference.created_at,
            updated_at=preference.updated_at
        )

    def get_all_preferences(self, db: Session, user_id: str) -> List[PersonalizationPreference]:
        """Get all personalization preferences for a user."""
        preferences = db.query(PreferenceModel).filter(
            PreferenceModel.user_id == user_id
        ).all()

        return [
            PersonalizationPreference(
                id=p.id,
                user_id=p.user_id,
                chapter_id=p.chapter_id,
                preferences=p.preferences,
                created_at=p.created_at,
                updated_at=p.updated_at
            ) for p in preferences
        ]

    def create_preference(self, db: Session, user_id: str, preference_create: PersonalizationPreferenceCreate) -> PersonalizationPreference:
        """Create or update personalization preferences for a user and chapter."""
        # Check if preference already exists for this user and chapter
        existing_preference = db.query(PreferenceModel).filter(
            PreferenceModel.user_id == user_id,
            PreferenceModel.chapter_id == preference_create.chapter_id
        ).first()

        preference_id = generate_preference_id()

        if existing_preference:
            # Update existing preference
            existing_preference.preferences = preference_create.preferences
            db.commit()
            db.refresh(existing_preference)

            return PersonalizationPreference(
                id=existing_preference.id,
                user_id=existing_preference.user_id,
                chapter_id=existing_preference.chapter_id,
                preferences=existing_preference.preferences,
                created_at=existing_preference.created_at,
                updated_at=existing_preference.updated_at
            )
        else:
            # Create new preference
            db_preference = PreferenceModel(
                id=preference_id,
                user_id=user_id,
                chapter_id=preference_create.chapter_id,
                preferences=preference_create.preferences
            )

            db.add(db_preference)
            db.commit()
            db.refresh(db_preference)

            return PersonalizationPreference(
                id=db_preference.id,
                user_id=db_preference.user_id,
                chapter_id=db_preference.chapter_id,
                preferences=db_preference.preferences,
                created_at=db_preference.created_at,
                updated_at=db_preference.updated_at
            )

    def delete_preference(self, db: Session, user_id: str, chapter_id: str) -> bool:
        """Delete personalization preferences for a user and chapter."""
        preference = db.query(PreferenceModel).filter(
            PreferenceModel.user_id == user_id,
            PreferenceModel.chapter_id == chapter_id
        ).first()

        if not preference:
            return False

        db.delete(preference)
        db.commit()
        return True
