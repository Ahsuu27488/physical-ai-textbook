"""Rivet: retrieve course excerpts, build the prompt, call the chat model."""
from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Any, AsyncIterator, Dict, List, Optional, Sequence, Tuple

from app import llm
from app.config import get_settings
from app.rag import prompts
from app.rag.chunking import strip_prefix
from app.rag.embeddings import embed_query
from app.rag.store import Hit, StoreUnavailable, get_store, normalize_page_url

logger = logging.getLogger(__name__)

PAGE_TOP_K = 3
PAGE_THRESHOLD_FACTOR = 0.8  # the page the learner is on gets a slightly lower bar
MAX_TOKENS = 1500  # Gemini counts thinking tokens here too, so leave headroom
TEMPERATURE = 0.3


@dataclass
class Passage:
    title: str
    section: str
    url: str
    text: str
    score: float
    key: str = ""  # identity of the underlying chunk, used for de-duplication


def sources_of(passages: Sequence[Passage]) -> List[Dict[str, Any]]:
    return [
        {"n": n, "title": p.title, "section": p.section, "url": p.url, "score": round(float(p.score), 4)}
        for n, p in enumerate(passages, start=1)
    ]


def _passage(hit: Hit) -> Passage:
    payload = hit.payload
    return Passage(
        title=payload.get("page_title", ""),
        section=payload.get("section", ""),
        url=payload.get("url", ""),
        text=payload.get("text", ""),
        score=hit.score,
        key=hit.id,
    )


def page_url_from_path(path: Optional[str]) -> Optional[str]:
    """'/physical-ai-textbook/docs/ros2/foo/' -> '/docs/ros2/foo'; None when it is not a docs page."""
    if not path:
        return None
    path = path.split("#", 1)[0].split("?", 1)[0].strip()
    idx = path.find("/docs")
    if idx < 0:
        return None
    return normalize_page_url(path[idx:])


def _page_path(page: Any) -> Optional[str]:
    if page is None:
        return None
    return page.get("path") if isinstance(page, dict) else getattr(page, "path", None)


def _retrieve_sync(question: str, page_path: Optional[str]) -> List[Passage]:
    settings = get_settings()
    store = get_store()
    if not store.configured:
        logger.warning("QDRANT_URL is not set: answering without course excerpts")
        return []
    vector = embed_query(question)
    top_k = settings.rag_top_k
    threshold = settings.rag_score_threshold

    page_hits: List[Hit] = []
    page_url = page_url_from_path(page_path)
    if page_url:
        page_hits = store.search(vector, PAGE_TOP_K, threshold * PAGE_THRESHOLD_FACTOR, url_prefix=page_url)
    global_hits = store.search(vector, top_k, threshold)

    # The current page's best chunks are guaranteed a place; the rest is filled by global score.
    chosen: Dict[str, Hit] = {}
    for hit in page_hits:
        chosen.setdefault(hit.id, hit)
    for hit in sorted(global_hits, key=lambda h: -h.score):
        if len(chosen) >= top_k:
            break
        chosen.setdefault(hit.id, hit)
    return [_passage(h) for h in sorted(chosen.values(), key=lambda h: -h.score)]


FOLLOW_UP_WORDS = 14


def search_query(question: str, history: Optional[Sequence[Any]] = None) -> str:
    """A short follow-up borrows the previous question, so "and how do I tune it?" still finds the PID lesson."""
    if len(question.split()) >= FOLLOW_UP_WORDS or not history:
        return question
    for item in reversed(list(history)):
        role = item.get("role") if isinstance(item, dict) else getattr(item, "role", None)
        content = item.get("content") if isinstance(item, dict) else getattr(item, "content", "")
        if role == "user" and content and content.strip() != question.strip():
            return content.strip()[:500] + "\n" + question
    return question


async def retrieve(question: str, page: Any = None) -> List[Passage]:
    """Course excerpts for a question. Never raises for Qdrant or embedding trouble: returns [] and logs."""
    try:
        return await asyncio.to_thread(_retrieve_sync, question, _page_path(page))
    except StoreUnavailable as exc:
        logger.warning("Course index unavailable: %s", exc)
    except Exception as exc:
        logger.warning("Retrieval failed (%s): answering without course excerpts", type(exc).__name__)
    return []


def _chat_kwargs(messages: List[Dict[str, str]], model: str) -> Dict[str, Any]:
    kwargs: Dict[str, Any] = {"model": model, "messages": messages, "temperature": TEMPERATURE, "max_tokens": MAX_TOKENS}
    effort = get_settings().chat_reasoning_effort.strip()
    if effort:
        kwargs["reasoning_effort"] = effort
    return kwargs


async def _complete(messages: List[Dict[str, str]], **extra: Any) -> Any:
    """Call the chat model; when it is out of free quota (429) or overloaded (5xx), try the fallback model once."""
    settings = get_settings()
    client = llm.get_async_openai_client()
    try:
        return await client.chat.completions.create(**_chat_kwargs(messages, settings.chat_model), **extra)
    except Exception as exc:
        fallback = settings.chat_fallback_model.strip()
        code = getattr(exc, "status_code", None) or 0
        if not (code == 429 or code >= 500) or not fallback or fallback == settings.chat_model:
            raise
        logger.warning("%s answered %s; falling back to %s", settings.chat_model, code, fallback)
        return await client.chat.completions.create(**_chat_kwargs(messages, fallback), **extra)


async def answer(question: str, history: Optional[Sequence[Any]] = None, page: Any = None) -> Tuple[str, List[Dict[str, Any]]]:
    """Return (answer text, sources)."""
    passages = await retrieve(search_query(question, history), page)
    messages = prompts.build_messages(question, passages, history, page)
    response = await _complete(messages)
    text = (response.choices[0].message.content or "").strip()
    return text, sources_of(passages)


async def answer_stream(
    question: str, history: Optional[Sequence[Any]] = None, page: Any = None
) -> AsyncIterator[Tuple[str, Any]]:
    """Yield ("meta", {"sources": [...]}) once, then ("token", "text") for every piece of the answer."""
    passages = await retrieve(search_query(question, history), page)
    yield "meta", {"sources": sources_of(passages)}
    messages = prompts.build_messages(question, passages, history, page)
    stream = await _complete(messages, stream=True)
    async for chunk in stream:
        if not getattr(chunk, "choices", None):
            continue
        delta = getattr(chunk.choices[0], "delta", None)
        piece = getattr(delta, "content", None) if delta is not None else None
        if piece:
            yield "token", piece


async def search_passages(query: str, top_k: int) -> List[Passage]:
    """Plain semantic search for /api/search. Raises StoreUnavailable when the index is not configured."""

    def run() -> List[Passage]:
        store = get_store()
        if not store.configured:
            raise StoreUnavailable("QDRANT_URL is not set")
        vector = embed_query(query)
        return [_passage(h) for h in store.search(vector, top_k, get_settings().rag_score_threshold)]

    return await asyncio.to_thread(run)


def excerpt(passage: Passage, limit: int = 400) -> str:
    text = " ".join(strip_prefix(passage.text).split())
    return text if len(text) <= limit else text[: limit - 1].rstrip() + "…"
