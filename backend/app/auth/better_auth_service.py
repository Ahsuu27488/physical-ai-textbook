"""
BetterAuth Integration Service

This service provides a bridge between BetterAuth and our custom user data requirements,
specifically collecting software and hardware background information during registration.
"""
from typing import Optional, Dict, Any
import httpx
import os
from ..models import UserCreate, User
from ..database.models import User as UserModel
from ..database.database import get_db
from ..auth.utils import get_password_hash, verify_password, generate_user_id
from sqlalchemy.orm import Session
from fastapi import HTTPException, status
import json


class BetterAuthIntegrationService:
    """
    Service to integrate with BetterAuth while maintaining custom fields
    for software and hardware background collection.
    """

    def __init__(self):
        self.betterauth_url = os.getenv("BETTERAUTH_URL", "http://localhost:4000")
        self.client = httpx.AsyncClient(timeout=30.0)

    async def register_user_with_background(
        self,
        user_create: UserCreate,
        db: Session
    ) -> User:
        """
        Register a user through BetterAuth but also store background information
        in our local database.
        """
        # First, create the user in our local database with background info
        existing_user = db.query(UserModel).filter(UserModel.email == user_create.email).first()
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

        # Hash the password
        hashed_password = get_password_hash(user_create.password)

        # Generate a new user ID
        user_id = generate_user_id()

        # Create new user with background information
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

        # Convert to response model
        return User(
            id=db_user.id,
            email=db_user.email,
            name=db_user.name,
            is_active=db_user.is_active,
            created_at=db_user.created_at,
            updated_at=db_user.updated_at
        )

    async def authenticate_user(self, email: str, password: str, db: Session) -> Optional[User]:
        """
        Authenticate user using our local database (since we need to store background info)
        """
        user = db.query(UserModel).filter(UserModel.email == email).first()
        if not user:
            return None
        if not verify_password(password, user.hashed_password):
            return None

        return User(
            id=user.id,
            email=user.email,
            name=user.name,
            is_active=user.is_active,
            created_at=user.created_at,
            updated_at=user.updated_at
        )

    async def get_user_by_id(self, user_id: str, db: Session) -> Optional[User]:
        """
        Get user by ID from our local database
        """
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

    async def update_user_background(
        self,
        user_id: str,
        software_background: Optional[str],
        hardware_background: Optional[str],
        db: Session
    ) -> User:
        """
        Update user's background information
        """
        db_user = db.query(UserModel).filter(UserModel.id == user_id).first()
        if not db_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        if software_background is not None:
            db_user.software_background = software_background
        if hardware_background is not None:
            db_user.hardware_background = hardware_background

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