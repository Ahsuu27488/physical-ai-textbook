from functools import lru_cache

from fastapi import HTTPException, status
from openai import AsyncOpenAI, OpenAI

from app.config import get_settings


@lru_cache(maxsize=1)
def _client(api_key: str) -> OpenAI:
    return OpenAI(api_key=api_key)


@lru_cache(maxsize=1)
def _async_client(api_key: str) -> AsyncOpenAI:
    return AsyncOpenAI(api_key=api_key)


def _require_key() -> str:
    api_key = get_settings().openai_api_key
    if not api_key:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="AI features are not configured (OPENAI_API_KEY is not set).",
        )
    return api_key


def openai_configured() -> bool:
    return bool(get_settings().openai_api_key)


def get_openai_client() -> OpenAI:
    """Return a shared OpenAI client, created on first use.

    Building the client at import time crashed the whole API when OPENAI_API_KEY was missing,
    taking auth and /health down with it. Now only the endpoints that need OpenAI fail, with a 503.
    """
    return _client(_require_key())


def get_async_openai_client() -> AsyncOpenAI:
    """Async twin of get_openai_client(), used for streaming chat completions."""
    return _async_client(_require_key())
