"""The chat/embedding client: the OpenAI SDK pointed at an OpenAI-compatible API (Gemini by default, D-020)."""
import logging
import time
from functools import lru_cache
from typing import Any, Callable, Dict, List

from fastapi import HTTPException, status
from openai import AsyncOpenAI, OpenAI

from app.config import get_settings

logger = logging.getLogger(__name__)

NOT_CONFIGURED = "AI features are not configured (OPENAI_API_KEY or GEMINI_API_KEY is not set)."


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


# --- the model chain: CHAT_MODEL, then each CHAT_FALLBACK_MODEL in turn ---------------------------------------------

DAY_QUOTA_PAUSE = 3600  # a model that said "per day" is skipped this long (its day resets at midnight Pacific)
MINUTE_QUOTA_PAUSE = 60
_resting: Dict[str, float] = {}  # model -> time it may be tried again


def _retryable(exc: Exception) -> bool:
    code = getattr(exc, "status_code", None) or 0
    return code == 429 or code >= 500


def _rest(model: str, exc: Exception) -> None:
    if getattr(exc, "status_code", None) == 429:
        _resting[model] = time.time() + (DAY_QUOTA_PAUSE if "per day" in str(exc) else MINUTE_QUOTA_PAUSE)


def chat_models() -> List[str]:
    """The chain in order, models still out of quota left out (all of them again if every one is resting)."""
    settings = get_settings()
    chain: List[str] = []
    for name in [settings.chat_model, *settings.chat_fallback_model.split(",")]:
        name = name.strip()
        if name and name not in chain:
            chain.append(name)
    now = time.time()
    awake = [m for m in chain if _resting.get(m, 0) <= now]
    return awake or chain


def _prepare_kwargs(kwargs: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize chat completion parameters for model-specific constraints (e.g. max_completion_tokens)."""
    params = dict(kwargs)
    model = str(params.get("model", "")).lower()

    # OpenAI reasoning and newer models (GPT-6, GPT-5, o1, o3, o4) reject max_tokens and require max_completion_tokens
    if (
        model.startswith("gpt-6")
        or model.startswith("gpt-5")
        or model.startswith("o1")
        or model.startswith("o3")
        or model.startswith("o4")
    ):
        if "max_tokens" in params:
            max_tok = params.pop("max_tokens")
            if "max_completion_tokens" not in params:
                params["max_completion_tokens"] = max_tok
    elif "max_completion_tokens" in params and "max_tokens" in params:
        params.pop("max_tokens", None)

    return params


def complete(kwargs_for: Callable[[str], Dict[str, Any]]) -> Any:
    """chat.completions.create on the first model of the chain that answers; non-quota errors are raised at once."""
    client = get_openai_client()
    models = chat_models()
    for i, model in enumerate(models):
        try:
            return client.chat.completions.create(**_prepare_kwargs(kwargs_for(model)))
        except Exception as exc:
            if not _retryable(exc) or i == len(models) - 1:
                raise
            _rest(model, exc)
            logger.warning("%s answered %s; trying %s", model, getattr(exc, "status_code", None), models[i + 1])


async def complete_async(kwargs_for: Callable[[str], Dict[str, Any]]) -> Any:
    """Async twin of complete(); with stream=True in the kwargs it returns the stream of the model that answered."""
    client = get_async_openai_client()
    models = chat_models()
    for i, model in enumerate(models):
        try:
            return await client.chat.completions.create(**_prepare_kwargs(kwargs_for(model)))
        except Exception as exc:
            if not _retryable(exc) or i == len(models) - 1:
                raise
            _rest(model, exc)
            logger.warning("%s answered %s; trying %s", model, getattr(exc, "status_code", None), models[i + 1])
