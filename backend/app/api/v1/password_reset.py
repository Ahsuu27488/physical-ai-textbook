"""Forgot password: a one-time link by email (docs/plans/2026-10-28-password-reset.md)."""
import hashlib
import html
import logging
import secrets
from datetime import datetime, timedelta, timezone

from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy import func
from sqlalchemy.orm import Session

from ...auth.utils import get_password_hash
from ...config import get_settings
from ...database.database import get_db
from ...database.models import PasswordReset, User as UserModel
from ...mail import MailError, mail_configured, send_email
from ...ratelimit import SlidingWindowLimiter

logger = logging.getLogger(__name__)
router = APIRouter()

LINK_MINUTES = 30
EMAILS_PER_HOUR = 3  # per address
reset_limiter = SlidingWindowLimiter(60 * 60.0)

SENT = "If an account exists for that email, we've sent a link to reset the password. Check your inbox and spam folder."
BAD_LINK = "This reset link is invalid or has expired. Ask for a new one."
NOT_SET_UP = "Password reset by email isn't set up on this server yet."


class ForgotPassword(BaseModel):
    email: EmailStr


class ResetPassword(BaseModel):
    token: str = Field(..., min_length=20, max_length=200)
    new_password: str = Field(..., min_length=8, max_length=200)


def _hash(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


def _now() -> datetime:
    return datetime.now(timezone.utc)


def _aware(value: datetime) -> datetime:
    return value if value.tzinfo else value.replace(tzinfo=timezone.utc)  # SQLite hands back naive datetimes


def _message(link: str, name: str) -> tuple:
    hello = f"Hi {name}," if name else "Hi,"
    text = (
        f"{hello}\n\nSomeone (hopefully you) asked to reset the password of your Kinemata account.\n"
        f"Open this link within {LINK_MINUTES} minutes to choose a new one:\n\n{link}\n\n"
        "If it wasn't you, ignore this email: your password stays the same.\n"
    )
    safe = html.escape(link, quote=True)
    body = (
        f"<p>{html.escape(hello)}</p><p>Someone (hopefully you) asked to reset the password of your Kinemata account. "
        f"Open this link within {LINK_MINUTES} minutes to choose a new one:</p>"
        f'<p><a href="{safe}">Choose a new password</a></p><p style="color:#666">{safe}</p>'
        "<p>If it wasn't you, ignore this email: your password stays the same.</p>"
    )
    return text, body


@router.post("/forgot-password")
def forgot_password(body: ForgotPassword, db: Session = Depends(get_db)):
    """Same reply whether or not the account exists, so the form can't be used to find accounts."""
    if not mail_configured():
        raise HTTPException(status.HTTP_503_SERVICE_UNAVAILABLE, NOT_SET_UP)
    email = body.email.strip().lower()
    allowed, _ = reset_limiter.check(email, EMAILS_PER_HOUR)
    if not allowed:
        return {"message": SENT}
    user = db.query(UserModel).filter(func.lower(UserModel.email) == email).first()
    if user is None:
        return {"message": SENT}
    token = secrets.token_urlsafe(32)
    db.add(PasswordReset(user_id=user.id, token_hash=_hash(token), expires_at=_now() + timedelta(minutes=LINK_MINUTES)))
    db.commit()
    link = f"{get_settings().site_url.rstrip('/')}/reset-password?token={token}"
    text, page = _message(link, user.name or "")
    try:
        send_email(user.email, "Reset your Kinemata password", text, page)
    except MailError as exc:
        logger.warning("Password-reset email failed: %s", exc)
        raise HTTPException(status.HTTP_502_BAD_GATEWAY, "Couldn't send the email. Try again in a few minutes.")
    return {"message": SENT}


@router.post("/reset-password")
def reset_password(body: ResetPassword, db: Session = Depends(get_db)):
    row = db.query(PasswordReset).filter(PasswordReset.token_hash == _hash(body.token)).first()
    if row is None or row.used_at is not None or _aware(row.expires_at) <= _now():
        raise HTTPException(status.HTTP_400_BAD_REQUEST, BAD_LINK)
    user = db.query(UserModel).filter(UserModel.id == row.user_id).first()
    if user is None:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, BAD_LINK)
    user.hashed_password = get_password_hash(body.new_password)
    now = _now()
    for open_link in db.query(PasswordReset).filter(PasswordReset.user_id == user.id, PasswordReset.used_at.is_(None)):
        open_link.used_at = now  # this link and any other unused one stop working
    db.commit()
    return {"message": "Password changed. Sign in with the new one."}
