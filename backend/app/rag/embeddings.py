"""Batch embeddings through the shared OpenAI client. Errors are raised, never replaced by zero vectors."""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence

from app import llm
from app.config import get_settings

BATCH_SIZE = 96
MAX_CHARS = 24_000  # ~6k tokens, safely under the 8191-token input limit


def embed_texts(
    texts: Sequence[str],
    *,
    model: Optional[str] = None,
    batch_size: int = BATCH_SIZE,
    on_batch: Optional[Callable[[int, int], None]] = None,
) -> List[List[float]]:
    """Embed `texts` in order. Raises on any API error or malformed response."""
    if not texts:
        return []
    client = llm.get_openai_client()
    model = model or get_settings().embed_model
    vectors: List[List[float]] = []
    for start in range(0, len(texts), batch_size):
        batch = [t[:MAX_CHARS] if t.strip() else " " for t in texts[start:start + batch_size]]
        response = client.embeddings.create(model=model, input=batch)
        data = sorted(response.data, key=lambda d: d.index)
        if len(data) != len(batch):
            raise RuntimeError(f"Embedding API returned {len(data)} vectors for {len(batch)} inputs")
        vectors.extend(list(d.embedding) for d in data)
        if on_batch:
            on_batch(len(vectors), len(texts))
    return vectors


def embed_query(text: str, *, model: Optional[str] = None) -> List[float]:
    return embed_texts([text], model=model)[0]
