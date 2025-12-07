from pydantic import BaseModel, EmailStr
from typing import Optional, List, Dict, Any
from datetime import datetime
from enum import Enum


class DocumentChunk(BaseModel):
    """Model for a chunk of a document in the RAG system."""
    id: str
    title: str
    content: str
    source_path: str
    chunk_index: int
    total_chunks: int
    metadata: Optional[Dict[str, Any]] = None
    created_at: datetime = datetime.now()


class ChatMessageRole(str, Enum):
    """Role of a message in a chat conversation."""
    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"


class ChatMessage(BaseModel):
    """Model for a chat message."""
    role: ChatMessageRole
    content: str
    timestamp: datetime = datetime.now()


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    message: str
    conversation_id: Optional[str] = None
    user_id: Optional[str] = None
    context_window: Optional[int] = 5  # Number of previous messages to include


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    response: str
    conversation_id: str
    sources: Optional[List[str]] = None
    timestamp: datetime = datetime.now()


class DocumentSearchRequest(BaseModel):
    """Request model for document search."""
    query: str
    top_k: int = 5
    filters: Optional[Dict[str, Any]] = None


class DocumentSearchResult(BaseModel):
    """Result model for document search."""
    id: str
    title: str
    content: str
    source_path: str
    score: float
    metadata: Optional[Dict[str, Any]] = None


class DocumentSearchResponse(BaseModel):
    """Response model for document search."""
    query: str
    results: List[DocumentSearchResult]
    timestamp: datetime = datetime.now()


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


class PersonalizationPreference(BaseModel):
    """Model for personalization preferences."""
    id: str
    user_id: str
    chapter_id: str
    preferences: Dict[str, Any]  # Store personalization settings as JSON
    created_at: datetime = datetime.now()
    updated_at: Optional[datetime] = None


class PersonalizationPreferenceCreate(BaseModel):
    """Model for creating personalization preferences."""
    chapter_id: str
    preferences: Dict[str, Any]


class TranslationRequest(BaseModel):
    """Model for translation request."""
    content: str
    target_language: str = "ur"  # Default to Urdu


class TranslationResponse(BaseModel):
    """Model for translation response."""
    original_content: str
    translated_content: str
    target_language: str