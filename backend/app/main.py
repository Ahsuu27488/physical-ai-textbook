from contextlib import asynccontextmanager
import logging

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Import API routers
from app.config import get_settings
from app.api.chat import router as chat_router
from app.api.v1.auth import router as auth_router
from app.api.v1.personalization import router as personalization_router
from app.api.v1.translation import router as translation_router
from app.api.v1.progress import router as progress_router
from app.api.v1.chats import router as chats_router
from app.api.v1.password_reset import router as password_reset_router
from app.database.database import create_tables

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Nothing else created the tables, so registration failed on a fresh database.
    try:
        create_tables()
    except Exception as e:  # keep /health and chat up if the database is unreachable
        logger.error(f"Could not create database tables: {e}")
    from app.rag import autoindex

    autoindex.start_in_background()  # keeps the course index in line with the deployed lessons
    yield


app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="API for Retrieval Augmented Generation for Physical AI textbook",
    version="1.0.0",
    lifespan=lifespan,
)

# CORS: the course site (SITE_URL, FRONTEND_URL, ADDITIONAL_CORS_ORIGIN) plus local dev servers
app.add_middleware(
    CORSMiddleware,
    allow_origins=get_settings().cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(auth_router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(password_reset_router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(personalization_router, prefix="/api/v1/personalization", tags=["personalization"])
app.include_router(translation_router, prefix="/api/v1/translation", tags=["translation"])
app.include_router(progress_router, prefix="/api/v1/progress", tags=["progress"])
app.include_router(chats_router, prefix="/api/v1/chats", tags=["chats"])

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics RAG API"}

@app.get("/health")
def health_check():
    """Liveness, which kind of database is in use and whether it answers (never the URL), and whether email is on."""
    from sqlalchemy import text

    from app.database.database import DATABASE_URL, engine

    kind = "sqlite (temporary)" if DATABASE_URL.startswith("sqlite") else "postgres"
    try:
        with engine.connect() as conn:
            conn.execute(text("SELECT 1"))
        reachable = True
    except Exception as exc:
        logger.warning("Database health check failed: %s", type(exc).__name__)
        reachable = False
    from app.mail import mail_configured

    return {"status": "healthy", "database": kind, "database_reachable": reachable, "email": mail_configured()}

if __name__ == "__main__":
    import uvicorn
    import os
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=port,
        reload=True
    )