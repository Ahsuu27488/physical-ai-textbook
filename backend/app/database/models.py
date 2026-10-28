from sqlalchemy import Column, String, Boolean, DateTime, Text, JSON, ForeignKey, Integer, UniqueConstraint
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from datetime import datetime

Base = declarative_base()


class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    name = Column(String, nullable=True)
    hashed_password = Column(String, nullable=False)
    is_active = Column(Boolean, default=True)
    software_background = Column(Text, nullable=True)  # User's software background for personalization
    hardware_background = Column(Text, nullable=True)  # User's hardware background for personalization
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class PersonalizationPreference(Base):
    __tablename__ = "personalization_preferences"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=False)
    chapter_id = Column(String, nullable=False)  # ID of the chapter being personalized
    preferences = Column(JSON, nullable=False)  # Store personalization settings as JSON
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class TranslationCache(Base):
    __tablename__ = "translation_cache"

    id = Column(String, primary_key=True, index=True)
    original_content_hash = Column(String, unique=True, nullable=False)  # Hash of original content
    original_content = Column(Text, nullable=False)
    translated_content = Column(Text, nullable=False)
    source_language = Column(String, default="en", nullable=False)
    target_language = Column(String, default="ur", nullable=False)  # Default to Urdu
    created_at = Column(DateTime(timezone=True), server_default=func.now())

class UserProgress(Base):
    """A lesson (doc_id, e.g. 'ros2/nodes-topics-services') a user has marked complete."""

    __tablename__ = "user_progress"
    __table_args__ = (UniqueConstraint("user_id", "doc_id", name="uq_user_progress_user_doc"),)

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=False, index=True)
    doc_id = Column(String, nullable=False)
    completed_at = Column(DateTime(timezone=True), server_default=func.now())


class QuizScore(Base):
    """Best quiz result per user and lesson page."""

    __tablename__ = "quiz_scores"
    __table_args__ = (UniqueConstraint("user_id", "page_path", name="uq_quiz_scores_user_page"),)

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=False, index=True)
    page_path = Column(String, nullable=False)
    best_score = Column(Integer, nullable=False, default=0)
    total = Column(Integer, nullable=False, default=0)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())


class ChatThread(Base):
    """One Rivet conversation of a signed-in learner, kept so it can be continued on another device."""

    __tablename__ = "chat_threads"

    id = Column(String, primary_key=True)  # the conversation_id the chat API hands out
    user_id = Column(String, ForeignKey("users.id"), nullable=False, index=True)
    title = Column(String, nullable=False, default="")
    messages = Column(JSON, nullable=False)  # [{role, text, sources?, feedback?}], newest last
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), index=True)


class AnswerFeedback(Base):
    """A thumbs up (1) or down (-1) on one of Rivet's answers, with the question and answer as they were."""

    __tablename__ = "answer_feedback"
    __table_args__ = (UniqueConstraint("user_id", "conversation_id", "message_id", name="uq_feedback_answer"),)

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=False, index=True)
    conversation_id = Column(String, nullable=False)
    message_id = Column(String, nullable=False)
    rating = Column(Integer, nullable=False)
    question = Column(Text, nullable=False, default="")
    answer = Column(Text, nullable=False, default="")
    page = Column(String, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class PasswordReset(Base):
    """A one-time password-reset link; only the SHA-256 of the emailed token is stored."""

    __tablename__ = "password_resets"

    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(String, ForeignKey("users.id"), nullable=False, index=True)
    token_hash = Column(String(64), nullable=False, unique=True, index=True)
    expires_at = Column(DateTime(timezone=True), nullable=False)
    used_at = Column(DateTime(timezone=True), nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

