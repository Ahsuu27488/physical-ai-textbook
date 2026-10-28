from pydantic import BaseModel, EmailStr, Field
from typing import Optional, List, Dict, Any
from datetime import datetime


# Authentication and User Models
class UserBase(BaseModel):
    """Base model for user data."""
    email: EmailStr
    name: Optional[str] = None


class UserCreate(UserBase):
    """Model for user creation request."""
    password: str
    # Background information for personalization
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class UserUpdate(BaseModel):
    """Model for user update request."""
    name: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class UserInDB(UserBase):
    """Model for user data in database (includes hashed password)."""
    id: str
    hashed_password: str
    is_active: bool = True
    created_at: datetime = datetime.now()
    updated_at: Optional[datetime] = None


class User(UserBase):
    """Model for user response (excludes sensitive data)."""
    id: str
    is_active: bool = True
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
    created_at: datetime
    updated_at: Optional[datetime] = None


class UserLogin(BaseModel):
    """Model for user login request."""
    email: EmailStr
    password: str


class Token(BaseModel):
    """Model for authentication token."""
    access_token: str
    token_type: str = "bearer"


class TokenData(BaseModel):
    """Model for token data."""
    user_id: str
    email: EmailStr


class TranslationRequest(BaseModel):
    """Model for translation request."""
    content: str = Field(..., min_length=1, max_length=12000)
    target_language: str = "ur"  # Default to Urdu


class TranslationResponse(BaseModel):
    """Model for translation response."""
    original_content: str
    translated_content: str
    target_language: str