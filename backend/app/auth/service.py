from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from typing import Optional
from ..database.models import User as UserModel
from ..models import UserCreate, UserUpdate, UserInDB, User
from .utils import get_password_hash, verify_password, generate_user_id
import uuid


class AuthService:
    """Service class for handling authentication-related operations."""

    def authenticate_user(self, db: Session, email: str, password: str) -> Optional[UserInDB]:
        """Authenticate a user by email and password."""
        user = self.get_user_by_email(db, email)
        if not user:
            return None
        if not verify_password(password, user.hashed_password):
            return None
        return user

    def get_user_by_email(self, db: Session, email: str) -> Optional[UserInDB]:
        """Get a user by email."""
        user = db.query(UserModel).filter(UserModel.email == email).first()
        if user:
            return UserInDB(
                id=user.id,
                email=user.email,
                name=user.name,
                hashed_password=user.hashed_password,
                is_active=user.is_active,
                created_at=user.created_at,
                updated_at=user.updated_at
            )
        return None

    def get_user_by_id(self, db: Session, user_id: str) -> Optional[User]:
        """Get a user by ID."""
        user = db.query(UserModel).filter(UserModel.id == user_id).first()
        if user:
            return User(
                id=user.id,
                email=user.email,
                name=user.name,
                is_active=user.is_active,
                created_at=user.created_at,
                updated_at=user.updated_at
            )
        return None

    def create_user(self, db: Session, user_create: UserCreate) -> User:
        """Create a new user."""
        # Check if user already exists
        existing_user = self.get_user_by_email(db, user_create.email)
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

        # Hash the password
        hashed_password = get_password_hash(user_create.password)

        # Generate a new user ID
        user_id = generate_user_id()

        # Create new user
        db_user = UserModel(
            id=user_id,
            email=user_create.email,
            name=user_create.name,
            hashed_password=hashed_password,
            software_background=user_create.software_background,
            hardware_background=user_create.hardware_background
        )

        db.add(db_user)
        db.commit()
        db.refresh(db_user)

        # Return the user without sensitive data
        return User(
            id=db_user.id,
            email=db_user.email,
            name=db_user.name,
            is_active=db_user.is_active,
            created_at=db_user.created_at,
            updated_at=db_user.updated_at
        )

    def update_user(self, db: Session, user_id: str, user_update: UserUpdate) -> User:
        """Update user information."""
        db_user = db.query(UserModel).filter(UserModel.id == user_id).first()
        if not db_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        # Update fields if they are provided
        if user_update.name is not None:
            db_user.name = user_update.name
        if user_update.software_background is not None:
            db_user.software_background = user_update.software_background
        if user_update.hardware_background is not None:
            db_user.hardware_background = user_update.hardware_background

        db.commit()
        db.refresh(db_user)

        return User(
            id=db_user.id,
            email=db_user.email,
            name=db_user.name,
            is_active=db_user.is_active,
            created_at=db_user.created_at,
            updated_at=db_user.updated_at
        )

    def deactivate_user(self, db: Session, user_id: str) -> bool:
        """Deactivate a user account."""
        db_user = db.query(UserModel).filter(UserModel.id == user_id).first()
        if not db_user:
            return False

        db_user.is_active = False
        db.commit()
        return True