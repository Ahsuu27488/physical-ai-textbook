"""Batch embeddings through the shared client. Errors are raised, never replaced by zero vectors."""
from __future__ import annotations

import logging
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

from app import llm
from app.config import get_settings

logger = logging.getLogger(__name__)

BATCH_SIZE = 50
MAX_CHARS = 24_000  # ~6k tokens, under gemini-embedding-2's 8192-token input limit
NATIVE_DIM = 3072  # gemini-embedding-2 default size
MAX_RETRIES = 6  # free-tier 429s: wait 2, 4, 8, ... up to 60 s between tries


def _uses_task_prefixes(model: str) -> bool:
    # gemini-embedding-2 has no task_type parameter; it expects the task written into the text.
    return model.startswith("gemini-embedding-2")


def as_document(text: str, title: str = "", model: Optional[str] = None) -> str:
    model = model or get_settings().embed_model
    if not _uses_task_prefixes(model):
        return text
    return f"title: {title.strip() or 'none'} | text: {text}"


def as_query(text: str, model: Optional[str] = None) -> str:
    model = model or get_settings().embed_model
    return f"task: search result | query: {text}" if _uses_task_prefixes(model) else text


def _create(client: Any, kwargs: Dict[str, Any], sleep: Callable[[float], None]) -> Any:
    for attempt in range(MAX_RETRIES + 1):
        try:
            return client.embeddings.create(**kwargs)
        except Exception as exc:
            if getattr(exc, "status_code", None) != 429 or attempt == MAX_RETRIES:
                raise
            wait = min(60.0, 2.0 ** (attempt + 1))
            logger.warning("Embedding rate-limited; retrying in %.0f s", wait)
            sleep(wait)
    raise AssertionError("unreachable")


GEMINI_NATIVE = "https://generativelanguage.googleapis.com/v1beta/models/{model}:batchEmbedContents"


class NativeGemini:
    """Gemini's own batchEmbedContents API behind the same `.embeddings.create(...)` shape as the OpenAI SDK.

    Gemini's OpenAI-compatible endpoint returned embeddings the SDK could not read (TypeError in production),
    so for Gemini models the documented native endpoint is used instead.
    """

    def __init__(self, api_key: str, dim: int) -> None:
        self.api_key, self.dim = api_key, dim
        self.embeddings = self

    def create(self, model: str, input: List[str], **_: Any) -> Any:
        import httpx
        from types import SimpleNamespace

        body = {"requests": [
            {"model": f"models/{model}", "content": {"parts": [{"text": t}]},
             **({"outputDimensionality": self.dim} if self.dim != NATIVE_DIM else {})}
            for t in input
        ]}
        r = httpx.post(GEMINI_NATIVE.format(model=model), json=body, headers={"x-goog-api-key": self.api_key}, timeout=60)
        if r.status_code >= 400:
            err = RuntimeError(f"Gemini embeddings HTTP {r.status_code}")
            err.status_code = r.status_code  # lets the 429 backoff and error reports see it
            raise err
        rows = r.json().get("embeddings") or []
        return SimpleNamespace(data=[SimpleNamespace(index=i, embedding=row.get("values") or []) for i, row in enumerate(rows)])


def _client_for(model: str) -> Any:
    settings = get_settings()
    if model.startswith("gemini-") and "generativelanguage.googleapis.com" in settings.llm_base_url:
        return NativeGemini(llm._require_key(), settings.embed_dim)
    return llm.get_openai_client()


def embed_texts(
    texts: Sequence[str],
    *,
    model: Optional[str] = None,
    batch_size: int = BATCH_SIZE,
    on_batch: Optional[Callable[[int, int], None]] = None,
    sleep: Callable[[float], None] = time.sleep,
) -> List[List[float]]:
    """Embed `texts` in order (already formatted with as_document/as_query). Raises on errors or a wrong size."""
    if not texts:
        return []
    settings = get_settings()
    model = model or settings.embed_model
    client = _client_for(model)
    extra: Dict[str, Any] = {"dimensions": settings.embed_dim} if settings.embed_dim != NATIVE_DIM else {}
    vectors: List[List[float]] = []
    for start in range(0, len(texts), batch_size):
        batch = [t[:MAX_CHARS] if t.strip() else " " for t in texts[start:start + batch_size]]
        # encoding_format="float": the SDK otherwise asks for base64, which Gemini's endpoint doesn't send back
        response = _create(client, {"model": model, "input": batch, "encoding_format": "float", **extra}, sleep)
        data = sorted(response.data, key=lambda d: d.index)
        if len(data) != len(batch):
            raise RuntimeError(f"Embedding API returned {len(data)} vectors for {len(batch)} inputs")
        for d in data:
            if len(d.embedding) != settings.embed_dim:
                raise RuntimeError(
                    f"{model} returned {len(d.embedding)}-d vectors but EMBED_DIM is {settings.embed_dim}"
                )
            vectors.append(list(d.embedding))
        if on_batch:
            on_batch(len(vectors), len(texts))
    return vectors


def embed_query(text: str, *, model: Optional[str] = None) -> List[float]:
    return embed_texts([as_query(text, model)], model=model)[0]
