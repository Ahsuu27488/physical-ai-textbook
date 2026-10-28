"""Central settings for the Kinemata backend (environment variables, `.env` supported)."""
from functools import lru_cache
from typing import List, Optional
from urllib.parse import urlparse

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

GEMINI_OPENAI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai/"
LOCAL_DEV_ORIGINS = ("http://localhost:3000", "http://localhost:3123")


def _origin(value: str) -> str:
    """Reduce 'https://host/path/' to 'https://host' (a CORS origin has no path)."""
    value = value.strip()
    if not value:
        return ""
    parsed = urlparse(value if "://" in value else f"https://{value}")
    if not parsed.netloc:
        return ""
    return f"{parsed.scheme}://{parsed.netloc}"


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

    # Any OpenAI-compatible API; the default is Google Gemini's free tier (D-020).
    gemini_api_key: Optional[str] = None
    llm_api_key_override: Optional[str] = Field(None, validation_alias="LLM_API_KEY")  # for a non-Gemini LLM_BASE_URL
    llm_base_url: str = GEMINI_OPENAI_BASE_URL
    chat_model: str = "gemini-3.8-flash"
    chat_fallback_model: str = "gemini-3.5-flash-lite"  # used when the main model is out of free quota
    chat_reasoning_effort: str = "low"  # blank to leave the model's default thinking on
    embed_model: str = "gemini-embedding-2"
    embed_dim: int = 3072  # the model's native size; another value is requested via `dimensions`

    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "kinemata_course"

    rag_top_k: int = 6
    rag_score_threshold: float = 0.25

    site_url: str = "https://physical-ai-textbook-puce-two.vercel.app"
    frontend_url: str = "http://localhost:3000"
    additional_cors_origin: str = ""  # one origin, or several separated by commas

    chat_rate_limit_per_min: int = 20  # per signed-in learner
    chat_daily_limit_per_user: int = 150  # keeps one learner from using up the free quota

    @field_validator("gemini_api_key", "llm_api_key_override", "qdrant_url", "qdrant_api_key", mode="before")
    @classmethod
    def _blank_is_none(cls, value):
        if isinstance(value, str):
            value = value.strip()
            return value or None
        return value

    @property
    def llm_api_key(self) -> Optional[str]:
        return self.llm_api_key_override or self.gemini_api_key

    @property
    def cors_origins(self) -> List[str]:
        raw = [self.frontend_url, self.site_url, *self.additional_cors_origin.split(","), *LOCAL_DEV_ORIGINS]
        origins: List[str] = []
        for item in raw:
            origin = _origin(item)
            if origin and origin not in origins:
                origins.append(origin)
        return origins


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    return Settings()
