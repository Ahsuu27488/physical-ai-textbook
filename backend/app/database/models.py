from sqlalchemy import Column, String, Boolean, DateTime, Text, JSON, ForeignKey, Integer
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