from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Get database URL from environment variables
DATABASE_URL = os.getenv("NEON_DATABASE_URL", "sqlite:///./test.db")

# Create engine with connection pooling
engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool if not DATABASE_URL.startswith("sqlite") else None,
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,  # Validates connections before use
    pool_recycle=300,    # Recycle connections after 5 minutes
    connect_args={"check_same_thread": False} if DATABASE_URL.startswith("sqlite") else {}  # Needed for SQLite
)

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for models
Base = declarative_base()


# Dependency to get database session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Function to create tables
def create_tables():
    """Create all tables in the database."""
    Base.metadata.create_all(bind=engine)


# Function to get database URL
def get_database_url():
    """Return the database URL."""
    return DATABASE_URL