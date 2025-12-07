#!/usr/bin/env python3
"""
Test script to verify the authentication system is working properly
"""
import os
import sys
import asyncio
from datetime import timedelta
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# Add the app directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from app.database.database import engine, get_db
from app.database.models import Base, User as UserModel
from app.models import UserCreate, User, Token, TokenData
from app.auth.service import AuthService
from app.auth.utils import create_access_token, verify_token, get_password_hash, verify_password
from app.services.translation import TranslationService
from app.services.personalization import PersonalizationService

def test_database_models():
    """Test that database models are properly defined"""
    print("Testing database models...")

    # Create tables
    Base.metadata.create_all(bind=engine)
    print("✅ Database tables created successfully")

    return True

def test_password_hashing():
    """Test password hashing and verification"""
    print("Testing password hashing...")

    password = "test_password_123"
    hashed = get_password_hash(password)

    # Verify the password
    assert verify_password(password, hashed), "Password verification failed"
    assert not verify_password("wrong_password", hashed), "Wrong password should not verify"

    print("✅ Password hashing and verification working")

    return True

def test_jwt_tokens():
    """Test JWT token creation and verification"""
    print("Testing JWT tokens...")

    # Create a token
    data = {"user_id": "test_user_123", "email": "test@example.com"}
    token = create_access_token(data=data, expires_delta=timedelta(minutes=30))

    # Verify the token
    token_data = verify_token(token)
    assert token_data is not None, "Token verification failed"
    assert token_data.get("user_id") == "test_user_123", "User ID mismatch"
    assert token_data.get("email") == "test@example.com", "Email mismatch"

    print("✅ JWT token creation and verification working")

    return True

def test_auth_service():
    """Test the authentication service"""
    print("Testing authentication service...")

    # Create a test database session
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    db = SessionLocal()

    try:
        # Initialize auth service
        auth_service = AuthService()

        # Create a test user
        user_create = UserCreate(
            email="test@example.com",
            password="test_password_123",
            name="Test User",
            software_background="intermediate",
            hardware_background="beginner"
        )

        # Create the user
        user = auth_service.create_user(db, user_create)
        assert user is not None, "User creation failed"
        assert user.email == "test@example.com", "User email mismatch"

        print("✅ User creation working")

        # Authenticate the user
        authenticated_user = auth_service.authenticate_user(db, "test@example.com", "test_password_123")
        assert authenticated_user is not None, "User authentication failed"
        assert authenticated_user.email == "test@example.com", "Authenticated user email mismatch"

        print("✅ User authentication working")

        # Test failed authentication
        failed_auth = auth_service.authenticate_user(db, "test@example.com", "wrong_password")
        assert failed_auth is None, "Authentication should fail with wrong password"

        print("✅ Failed authentication working correctly")

        # Get user by ID
        retrieved_user = auth_service.get_user_by_id(db, user.id)
        assert retrieved_user is not None, "User retrieval by ID failed"
        assert retrieved_user.email == "test@example.com", "Retrieved user email mismatch"

        print("✅ User retrieval by ID working")

    finally:
        # Clean up: delete the test user
        db.query(UserModel).filter(UserModel.email == "test@example.com").delete()
        db.commit()
        db.close()

    print("✅ Authentication service tests passed")

    return True

def test_translation_service():
    """Test the translation service"""
    print("Testing translation service...")

    translation_service = TranslationService()

    # Test supported languages
    supported_languages = translation_service.supported_languages
    assert "ur" in supported_languages, "Urdu should be supported"
    assert "es" in supported_languages, "Spanish should be supported"
    assert "fr" in supported_languages, "French should be supported"

    print("✅ Translation service supported languages working")

    # Test get_supported_languages method
    languages = translation_service.get_supported_languages()
    assert languages == supported_languages, "get_supported_languages should return the same languages"

    print("✅ Translation service get_supported_languages working")

    print("✅ Translation service tests passed")

    return True

def test_personalization_service():
    """Test the personalization service"""
    print("Testing personalization service...")

    personalization_service = PersonalizationService()

    # Test apply personalization (basic functionality)
    content = "This is a sample content about robotics and AI."
    preferences = {
        "difficulty_level": "beginner",
        "include_examples": True,
        "focus_on_code": False,
        "focus_on_theory": True
    }

    # Apply personalization (this will just return the content for now as the actual
    # personalization logic would be more complex)
    personalized_content = personalization_service.apply_personalization(content, preferences)

    # For now, we just check that it returns something
    assert personalized_content is not None, "Personalization should return content"

    print("✅ Personalization service basic functionality working")

    print("✅ Personalization service tests passed")

    return True

def main():
    """Run all tests"""
    print("🧪 Starting authentication system tests...\n")

    try:
        # Test database models
        test_database_models()
        print()

        # Test password hashing
        test_password_hashing()
        print()

        # Test JWT tokens
        test_jwt_tokens()
        print()

        # Test auth service
        test_auth_service()
        print()

        # Test translation service
        test_translation_service()
        print()

        # Test personalization service
        test_personalization_service()
        print()

        print("🎉 All tests passed! Authentication system is working correctly.")
        return True

    except Exception as e:
        print(f"❌ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    main()