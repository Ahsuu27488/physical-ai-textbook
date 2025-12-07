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

    def apply_personalization(self, content: str, preferences: Dict[str, Any]) -> str:
        """Apply personalization preferences to content."""
        # This is a simplified implementation - in a real application, this would be much more complex
        # For now, we'll implement basic personalization based on user preferences

        personalized_content = content

        # Apply personalization based on preferences
        if preferences:
            # Adjust content based on difficulty level preference
            difficulty_level = preferences.get("difficulty_level")
            if difficulty_level == "beginner":
                # For beginners, add explanations and simplify complex concepts
                personalized_content = self._simplify_for_beginners(personalized_content)

            # Add more examples if requested
            if preferences.get("include_examples", False):
                personalized_content = self._add_examples(personalized_content)

            # Focus on code or theory based on preferences
            if preferences.get("focus_on_code", False):
                personalized_content = self._emphasize_code(personalized_content)
            elif preferences.get("focus_on_theory", False):
                personalized_content = self._emphasize_theory(personalized_content)

            # Consider user's background
            software_background = preferences.get("software_background")
            hardware_background = preferences.get("hardware_background")
            if software_background or hardware_background:
                personalized_content = self._adjust_for_background(personalized_content, software_background, hardware_background)

        return personalized_content

    def _simplify_for_beginners(self, content: str) -> str:
        """Add explanations and simplify content for beginners."""
        # Add more descriptive headers and explanations
        simplified_content = content.replace("#", "##")  # Make main headers smaller to add more context
        simplified_content += "\n\n> 💡 **Beginner Tip:** This concept might seem complex at first, but it's fundamental to understanding robotics. Try to think of it as connecting different parts of a robot so they can communicate effectively."
        return simplified_content

    def _add_examples(self, content: str) -> str:
        """Add more examples to the content."""
        # Add practical examples throughout the content
        examples_added_content = content + "\n\n### Practical Example:\nHere's a practical example to illustrate this concept:\n```python\n# Example code demonstrating the concept\nexample_code = 'implementation'\nprint(f'Result: {example_code}')\n```\n"
        return examples_added_content

    def _emphasize_code(self, content: str) -> str:
        """Emphasize code implementation aspects."""
        # Add more code-focused content
        code_emphasized_content = content + "\n\n### Implementation Details:\nWhen implementing this in code, consider the following:\n- Performance implications\n- Error handling\n- Memory management\n"
        return code_emphasized_content

    def _emphasize_theory(self, content: str) -> str:
        """Emphasize theoretical aspects."""
        # Add more theoretical context
        theory_emphasized_content = content + "\n\n### Theoretical Background:\nThis concept has its roots in [relevant theory], which was developed to solve [specific problem]. The mathematical foundation involves [brief explanation]."
        return theory_emphasized_content

    def _adjust_for_background(self, content: str, software_background: str, hardware_background: str) -> str:
        """Adjust content based on user's software/hardware background."""
        adjusted_content = content
        if software_background:
            adjusted_content += f"\n\n> 🧠 **Software Perspective:** As someone with a {software_background} software background, consider how this concept applies to software architecture and design patterns."
        if hardware_background:
            adjusted_content += f"\n\n> ⚙️ **Hardware Perspective:** As someone with a {hardware_background} hardware background, consider the physical implementation challenges and constraints."
        return adjusted_content