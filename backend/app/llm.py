from functools import lru_cache
import os

from fastapi import HTTPException, status
from openai import OpenAI


@lru_cache(maxsize=1)
def _client(api_key: str) -> OpenAI:
    return OpenAI(api_key=api_key)


def get_openai_client() -> OpenAI:
    """Return a shared OpenAI client, created on first use.

    Building the client at import time crashed the whole API when OPENAI_API_KEY was missing,
    taking auth and /health down with it. Now only the endpoints that need OpenAI fail, with a 503.
    """
    api_key = os.getenv("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="AI features are not configured (OPENAI_API_KEY is not set).",
        )
    return _client(api_key)
