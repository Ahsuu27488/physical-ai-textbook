from fastapi import APIRouter, Body, Depends, HTTPException, status
from pydantic import BaseModel, Field, field_validator
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from sqlalchemy.orm import Session
from datetime import timedelta
from typing import Optional
from ...database.database import get_db
from ...models import UserCreate, User, UserLogin, Token, TokenData
from ...auth.better_auth_service import BetterAuthIntegrationService
from ...auth.utils import create_access_token, verify_token, verify_password, get_password_hash
from ...database.models import User as UserModel, UserProgress, QuizScore, PersonalizationPreference
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Create router
router = APIRouter()

# OAuth2 scheme for token
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

# Initialize auth service
auth_service = BetterAuthIntegrationService()

# Token expiration time (in minutes)
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))


async def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)) -> User:
    """Dependency to get the current user from the token."""
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

    user = await auth_service.get_user_by_id(user_id, db)
    if user is None:
        raise credentials_exception

    return user


@router.post("/register", response_model=User)
async def register(user_create: UserCreate, db: Session = Depends(get_db)):
    """Register a new user with background information."""
    try:
        user = await auth_service.register_user_with_background(user_create, db)
        return user
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Registration failed: {str(e)}"
        )


@router.post("/token", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends(), db: Session = Depends(get_db)):
    """Login endpoint that returns an access token."""
    user = await auth_service.authenticate_user(form_data.username, form_data.password, db)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Inactive user"
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"user_id": user.id, "email": user.email},
        expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}


@router.post("/login", response_model=Token)
async def login_legacy(login_data: UserLogin, db: Session = Depends(get_db)):
    """Legacy login endpoint for direct email/password authentication."""
    user = await auth_service.authenticate_user(login_data.email, login_data.password, db)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Inactive user"
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"user_id": user.id, "email": user.email},
        expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}


@router.get("/me", response_model=User)
async def read_users_me(current_user: User = Depends(get_current_user)):
    """Get current user information."""
    return current_user


BACKGROUND_LEVELS = ("beginner", "intermediate", "advanced")


def _clean_background(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    value = value.strip().lower()
    if value and value not in BACKGROUND_LEVELS:
        raise ValueError("must be one of: beginner, intermediate, advanced (or empty)")
    return value


class ProfileUpdate(BaseModel):
    """Only the fields that are sent are changed. Empty background = clear it."""
    name: Optional[str] = Field(None, max_length=100)
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None

    @field_validator("software_background", "hardware_background")
    @classmethod
    def _levels(cls, value):
        return _clean_background(value)


class ChangePassword(BaseModel):
    current_password: str
    new_password: str = Field(..., min_length=8, max_length=200)


class DeleteAccount(BaseModel):
    password: str


def _db_user(db: Session, user_id: str) -> UserModel:
    db_user = db.query(UserModel).filter(UserModel.id == user_id).first()
    if db_user is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="User not found")
    return db_user


@router.put("/profile", response_model=User)
async def update_profile(
    body: Optional[ProfileUpdate] = Body(None),
    software_background: Optional[str] = None,  # legacy: query parameters still work
    hardware_background: Optional[str] = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Update name and the software/hardware background (beginner | intermediate | advanced | empty)."""
    try:
        legacy = ProfileUpdate(software_background=software_background, hardware_background=hardware_background)
    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=str(e))
    changes = {k: getattr(legacy, k) for k in ("software_background", "hardware_background") if getattr(legacy, k) is not None}
    if body is not None:
        changes.update({k: getattr(body, k) for k in body.model_fields_set})

    db_user = _db_user(db, current_user.id)
    for field, value in changes.items():
        value = (value.strip() or None) if isinstance(value, str) else None
        setattr(db_user, field, value)
    db.commit()
    db.refresh(db_user)
    return await auth_service.get_user_by_id(db_user.id, db)


@router.post("/change-password")
async def change_password(
    body: ChangePassword,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    db_user = _db_user(db, current_user.id)
    if not verify_password(body.current_password, db_user.hashed_password):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Current password is incorrect")
    db_user.hashed_password = get_password_hash(body.new_password)
    db.commit()
    return {"message": "Password changed"}


@router.delete("/account")
async def delete_account(
    body: DeleteAccount,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """Delete the account together with its progress, quiz scores and personalization preferences."""
    db_user = _db_user(db, current_user.id)
    if not verify_password(body.password, db_user.hashed_password):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Incorrect password")
    for model in (UserProgress, QuizScore, PersonalizationPreference):
        db.query(model).filter(model.user_id == db_user.id).delete(synchronize_session=False)
    db.delete(db_user)
    db.commit()
    return {"message": "Account deleted"}
