"""Central settings for the Proprio backend (environment variables, `.env` supported)."""
from functools import lru_cache
from typing import List, Optional
from urllib.parse import urlparse

from pydantic import field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

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

    openai_api_key: Optional[str] = None
    chat_model: str = "gpt-4o-mini"
    embed_model: str = "text-embedding-3-small"  # 1536 dimensions

    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "proprio_course"

    rag_top_k: int = 6
    rag_score_threshold: float = 0.25

    site_url: str = "https://physical-ai-textbook-puce-two.vercel.app"
    frontend_url: str = "http://localhost:3000"
    additional_cors_origin: str = ""  # one origin, or several separated by commas

    chat_rate_limit_per_min: int = 20

    @field_validator("openai_api_key", "qdrant_url", "qdrant_api_key", mode="before")
    @classmethod
    def _blank_is_none(cls, value):
        if isinstance(value, str):
            value = value.strip()
            return value or None
        return value

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
