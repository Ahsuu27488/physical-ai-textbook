from passlib.context import CryptContext
from datetime import datetime, timedelta
from typing import Optional
import jwt
from jwt.exceptions import ExpiredSignatureError, InvalidTokenError
from fastapi import HTTPException, status
from dotenv import load_dotenv
import os
import uuid

# Load environment variables
load_dotenv()

# Initialize password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT configuration
SECRET_KEY = os.getenv("SECRET_KEY", "fallback-secret-key-for-development")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against a hashed password."""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """Hash a plain password."""
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token."""
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> Optional[dict]:
    """Verify a JWT token and return the payload."""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except ExpiredSignatureError:
        return None
    except InvalidTokenError:
        return None


def generate_user_id() -> str:
    """Generate a unique user ID."""
    return str(uuid.uuid4())


def generate_preference_id() -> str:
    """Generate a unique preference ID."""
    return str(uuid.uuid4())


def generate_translation_cache_id() -> str:
    """Generate a unique translation cache ID."""
    return str(uuid.uuid4())