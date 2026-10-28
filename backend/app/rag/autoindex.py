"""Index the course from inside the deployed API, so nobody needs the keys on their own machine.

The deploy copies the lessons next to the app (COURSE_DOCS_DIR, default ./course_docs). On startup a background
thread chunks them and compares a fingerprint (chunks + embedding model + size) with the one stored in a tiny
`<collection>_meta` collection: unchanged lessons cost nothing; changed ones are embedded and upserted, and points
of removed sections are deleted. Progress and failures show on /api/documents/health (never any secret).
"""
from __future__ import annotations

import hashlib
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from app.config import get_settings
from app.rag.chunking import chunk_docs

logger = logging.getLogger(__name__)

META_ID = 1
status: Dict[str, Any] = {"state": "not started"}
_lock = threading.Lock()


def docs_dir() -> Path:
    return Path(os.getenv("COURSE_DOCS_DIR", "course_docs"))


def fingerprint(chunks) -> str:
    settings = get_settings()
    h = hashlib.sha256(f"{settings.embed_model}|{settings.embed_dim}".encode())
    for c in chunks:
        h.update(b"\0" + c.url.encode() + b"\0" + c.text.encode())
    return h.hexdigest()


def _meta_name(store) -> str:
    return f"{store.collection}_meta"


def _stored_fingerprint(store) -> Optional[str]:
    client = store.client
    if not client.collection_exists(_meta_name(store)):
        return None
    points = client.retrieve(_meta_name(store), ids=[META_ID], with_payload=True)
    return (points[0].payload or {}).get("fingerprint") if points else None


def _save_fingerprint(store, value: str, count: int) -> None:
    from qdrant_client.http import models

    client = store.client
    if not client.collection_exists(_meta_name(store)):
        client.create_collection(_meta_name(store), vectors_config=models.VectorParams(size=1, distance=models.Distance.DOT))
    client.upsert(
        _meta_name(store),
        points=[models.PointStruct(id=META_ID, vector=[1.0], payload={"fingerprint": value, "points": count, "at": time.time()})],
        wait=True,
    )


def run_once(store=None, docs: Optional[Path] = None) -> Dict[str, Any]:
    """Bring the index in line with the lessons. Returns (and records) the outcome."""
    from app.rag import embeddings
    from app.rag.store import get_store

    with _lock:
        docs = docs or docs_dir()
        if not docs.is_dir():
            status.update(state="skipped", reason=f"no lessons folder at {docs}")
            return dict(status)
        if not get_settings().llm_api_key:
            status.update(state="skipped", reason="GEMINI_API_KEY is not set")
            return dict(status)
        store = store or get_store()
        if not store.configured:
            status.update(state="skipped", reason="QDRANT_URL is not set")
            return dict(status)
        try:
            chunks = chunk_docs(docs)
            fp = fingerprint(chunks)
            if store.client.collection_exists(store.collection) and _stored_fingerprint(store) == fp:
                status.update(state="up to date", chunks=len(chunks), reason=None)
                return dict(status)
            status.update(state="indexing", chunks=len(chunks), done=0, reason=None)
            vectors = embeddings.embed_texts(
                [embeddings.as_document(c.text, c.page_title) for c in chunks],
                on_batch=lambda done, total: status.update(done=done),
            )
            store.ensure_collection()
            ids = store.upsert(chunks, vectors)
            removed = store.delete_stale(ids)
            _save_fingerprint(store, fp, len(ids))
            status.update(state="indexed", chunks=len(ids), removed=removed, at=time.time())
            logger.info("Indexed %d chunks (%d stale removed)", len(ids), removed)
        except Exception as exc:  # report the kind of failure, never the message (it may echo a URL or key)
            code = getattr(exc, "status_code", None) or getattr(exc, "code", None)
            reason = f"{type(exc).__name__}" + (f" ({code})" if code else "")
            if isinstance(exc, (TypeError, ValueError, KeyError, AttributeError)):  # shape errors: the text is safe
                reason += f": {str(exc)[:160]}"
            status.update(state="failed", reason=reason)
            logger.error("Auto-index failed: %s", status["reason"])
        return dict(status)


def check_chat_model() -> None:
    """One tiny call so /api/documents/health shows whether Rivet's model answers (never the error text)."""
    from app import llm
    from app.rag.service import _chat_kwargs

    if not get_settings().llm_api_key:
        status["chat"] = "skipped: no key"
        return
    try:
        kwargs = _chat_kwargs([{"role": "user", "content": "Reply with the single word OK."}], get_settings().chat_model)
        llm.get_openai_client().chat.completions.create(**kwargs)
        status["chat"] = "ok"
    except Exception as exc:
        code = getattr(exc, "status_code", None)
        status["chat"] = f"failed: {type(exc).__name__}" + (f" ({code})" if code else "")


def _startup() -> None:
    check_chat_model()
    run_once()


def start_in_background() -> None:
    if os.getenv("AUTO_INDEX", "1").strip() in ("0", "false", "no"):
        status.update(state="disabled")
        return
    threading.Thread(target=_startup, name="course-autoindex", daemon=True).start()
