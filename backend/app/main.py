from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Import API routers
from app.api.chat import router as chat_router
from app.api.v1.auth import router as auth_router
from app.api.v1.personalization import router as personalization_router
from app.api.v1.translation import router as translation_router

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="API for Retrieval Augmented Generation for Physical AI textbook",
    version="1.0.0"
)

# Add CORS middleware
frontend_url = os.getenv("FRONTEND_URL", "http://localhost:3000")  # Default for local development
additional_origin = os.getenv("ADDITIONAL_CORS_ORIGIN", "")  # For GitHub Pages or other origins

# Create list of allowed origins
allowed_origins = [frontend_url]
if additional_origin:
    allowed_origins.append(additional_origin)

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,  # Use environment variable for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(auth_router, prefix="/api/v1/auth", tags=["auth"])
app.include_router(personalization_router, prefix="/api/v1/personalization", tags=["personalization"])
app.include_router(translation_router, prefix="/api/v1/translation", tags=["translation"])

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics RAG API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

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