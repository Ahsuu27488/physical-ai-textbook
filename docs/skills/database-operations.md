# Database Operations Skill

## Purpose
This skill handles all database operations including user management, preferences, and translation caching.

## Functionality
- User creation, retrieval, and updates with background information
- Personalization preferences storage and retrieval
- Translation caching for performance
- Database connection management

## Implementation Pattern
```python
from sqlalchemy.orm import Session
from app.database.models import User, PersonalizationPreference, TranslationCache
from app.models import UserCreate, UserUpdate, PersonalizationPreferenceCreate
import uuid
from datetime import datetime
import hashlib

class DatabaseOperations:
    def __init__(self, db_session: Session):
        self.db = db_session

    def create_user(self, user_create: UserCreate) -> User:
        # Hash the password
        hashed_password = get_password_hash(user_create.password)

        # Create new user with background information
        db_user = User(
            id=str(uuid.uuid4()),
            email=user_create.email,
            name=user_create.name,
            hashed_password=hashed_password,
            software_background=user_create.software_background,
            hardware_background=user_create.hardware_background,
            is_active=True
        )

        self.db.add(db_user)
        self.db.commit()
        self.db.refresh(db_user)

        return db_user

    def get_user_by_email(self, email: str) -> User:
        return self.db.query(User).filter(User.email == email).first()

    def create_personalization_preference(self, user_id: str, chapter_id: str, preferences: dict):
        # Check if preference already exists for this user and chapter
        existing_pref = self.db.query(PersonalizationPreference).filter(
            PersonalizationPreference.user_id == user_id,
            PersonalizationPreference.chapter_id == chapter_id
        ).first()

        if existing_pref:
            # Update existing preference
            existing_pref.preferences = preferences
            existing_pref.updated_at = datetime.now()
            self.db.commit()
            return existing_pref
        else:
            # Create new preference
            pref = PersonalizationPreference(
                id=str(uuid.uuid4()),
                user_id=user_id,
                chapter_id=chapter_id,
                preferences=preferences
            )
            self.db.add(pref)
            self.db.commit()
            self.db.refresh(pref)
            return pref

    def get_personalization_preference(self, user_id: str, chapter_id: str):
        return self.db.query(PersonalizationPreference).filter(
            PersonalizationPreference.user_id == user_id,
            PersonalizationPreference.chapter_id == chapter_id
        ).first()

    def create_translation_cache(self, original_content: str, translated_content: str, target_language: str = "ur"):
        # Create hash of original content for caching
        content_hash = hashlib.md5(original_content.encode()).hexdigest()

        # Check if translation already exists in cache
        existing_cache = self.db.query(TranslationCache).filter(
            TranslationCache.original_content_hash == content_hash,
            TranslationCache.target_language == target_language
        ).first()

        if existing_cache:
            return existing_cache

        # Create new cache entry
        cache_entry = TranslationCache(
            id=str(uuid.uuid4()),
            original_content_hash=content_hash,
            original_content=original_content,
            translated_content=translated_content,
            source_language="en",
            target_language=target_language
        )

        self.db.add(cache_entry)
        self.db.commit()
        self.db.refresh(cache_entry)

        return cache_entry

    def get_cached_translation(self, original_content: str, target_language: str = "ur"):
        content_hash = hashlib.md5(original_content.encode()).hexdigest()
        return self.db.query(TranslationCache).filter(
            TranslationCache.original_content_hash == content_hash,
            TranslationCache.target_language == target_language
        ).first()
```

## Usage Examples
1. Initialize database operations with session
2. Create users with background information
3. Store and retrieve personalization preferences
4. Cache translations for performance