"""The chat/embedding client: the OpenAI SDK pointed at an OpenAI-compatible API (Gemini by default, D-020)."""
from functools import lru_cache

from fastapi import HTTPException, status
from openai import AsyncOpenAI, OpenAI

from app.config import get_settings

NOT_CONFIGURED = "AI features are not configured (GEMINI_API_KEY is not set)."


@lru_cache(maxsize=2)
def _client(api_key: str, base_url: str) -> OpenAI:
    return OpenAI(api_key=api_key, base_url=base_url)


@lru_cache(maxsize=2)
def _async_client(api_key: str, base_url: str) -> AsyncOpenAI:
    return AsyncOpenAI(api_key=api_key, base_url=base_url)


def _require_key() -> str:
    api_key = get_settings().llm_api_key
    if not api_key:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail=NOT_CONFIGURED)
    return api_key


def llm_configured() -> bool:
    return bool(get_settings().llm_api_key)


def get_openai_client() -> OpenAI:
    """Return a shared client, created on first use.

    Building the client at import time crashed the whole API when the key was missing,
    taking auth and /health down with it. Now only the endpoints that need the model fail, with a 503.
    """
    return _client(_require_key(), get_settings().llm_base_url)


def get_async_openai_client() -> AsyncOpenAI:
    """Async twin of get_openai_client(), used for streaming chat completions."""
    return _async_client(_require_key(), get_settings().llm_base_url)
