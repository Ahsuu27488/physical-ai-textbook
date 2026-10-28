"""Rivet's HTTP API: /api/chat, /api/chat/stream, /api/search (signed-in only), /api/documents/health."""
import json
import logging
import uuid
from typing import Any, AsyncIterator, Dict, List, Literal, Optional

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field

from app import llm
from app.config import get_settings
from app.rag import service
from app.rag.store import StoreUnavailable, get_store
from app.api.v1.progress import get_progress_user_id
from app.ratelimit import enforce_chat_rate_limit

logger = logging.getLogger(__name__)

router = APIRouter()


# --- models ----------------------------------------------------------------------------------
class HistoryItem(BaseModel):
    role: Literal["user", "assistant"]
    content: str = Field(..., max_length=20000)


class PageInfo(BaseModel):
    title: Optional[str] = Field(None, max_length=300)
    path: Optional[str] = Field(None, max_length=500)


class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=4000)
    conversation_id: Optional[str] = Field(None, max_length=100)
    history: Optional[List[HistoryItem]] = Field(None, max_length=50)
    page: Optional[PageInfo] = None


class Source(BaseModel):
    n: int
    title: str
    section: str
    url: str
    score: float


class ChatResponse(BaseModel):
    response: str
    conversation_id: str
    sources: List[Source]


class SearchRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000)
    top_k: int = Field(5, ge=1, le=20)


class SearchResult(Source):
    text: str


class SearchResponse(BaseModel):
    query: str
    results: List[SearchResult]


# --- helpers ---------------------------------------------------------------------------------
def _require_llm() -> None:
    if not llm.llm_configured():
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail=llm.NOT_CONFIGURED)


def _failure(exc: Exception) -> Dict[str, Any]:
    """A safe, user-facing message and HTTP-ish status for a failed model call (never leaks details)."""
    code = getattr(exc, "status_code", None)
    if code == 429:
        return {"status": 429, "message": "The AI service is busy or out of quota right now. Please try again in a moment."}
    if code in (401, 403):
        return {"status": 503, "message": "The AI service is not configured correctly."}
    if code == 400:
        return {"status": 502, "message": "The AI service rejected the request."}
    return {"status": 502, "message": "Rivet could not reach the AI service. Please try again."}


def _sse(event: str, data: Dict[str, Any]) -> str:
    return f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n"


def signed_in_learner(user_id: str = Depends(get_progress_user_id)) -> str:
    """Rivet is for signed-in learners only (401 otherwise); limits are per account."""
    enforce_chat_rate_limit(user_id)
    return user_id


# --- endpoints -------------------------------------------------------------------------------
@router.post("/chat", response_model=ChatResponse, dependencies=[Depends(signed_in_learner)])
async def chat_endpoint(req: ChatRequest):
    """Answer a question about the course, with numbered sources."""
    _require_llm()
    conversation_id = req.conversation_id or str(uuid.uuid4())
    try:
        text, sources = await service.answer(req.message, req.history, req.page)
    except HTTPException:
        raise
    except Exception as exc:
        logger.error("Chat failed: %s: %s", type(exc).__name__, exc)
        failure = _failure(exc)
        raise HTTPException(status_code=failure["status"], detail=failure["message"])
    return ChatResponse(response=text, conversation_id=conversation_id, sources=sources)


@router.post("/chat/stream", dependencies=[Depends(signed_in_learner)])
async def chat_stream_endpoint(req: ChatRequest):
    """Server-sent events: meta (sources), token (text pieces), done. On failure: error."""
    _require_llm()  # a plain 503 before any streaming starts
    conversation_id = req.conversation_id or str(uuid.uuid4())

    async def events() -> AsyncIterator[str]:
        try:
            async for kind, payload in service.answer_stream(req.message, req.history, req.page):
                if kind == "meta":
                    yield _sse("meta", {"conversation_id": conversation_id, "sources": payload["sources"]})
                else:
                    yield _sse("token", {"text": payload})
            yield _sse("done", {})
        except Exception as exc:
            logger.error("Chat stream failed: %s: %s", type(exc).__name__, exc)
            yield _sse("error", _failure(exc))

    return StreamingResponse(
        events(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no", "Connection": "keep-alive"},
    )


@router.post("/search", response_model=SearchResponse, dependencies=[Depends(signed_in_learner)])
async def search_endpoint(req: SearchRequest):
    """Semantic search over the course, returning the same source objects chat uses."""
    _require_llm()
    try:
        passages = await service.search_passages(req.query, req.top_k)
    except HTTPException:
        raise
    except StoreUnavailable:
        raise HTTPException(status_code=503, detail="The course index is not configured (QDRANT_URL is not set).")
    except Exception as exc:
        logger.error("Search failed: %s: %s", type(exc).__name__, exc)
        failure = _failure(exc)
        raise HTTPException(status_code=failure["status"], detail=failure["message"])
    sources = service.sources_of(passages)
    return SearchResponse(
        query=req.query,
        results=[SearchResult(**s, text=service.excerpt(p)) for s, p in zip(sources, passages)],
    )


@router.get("/documents/health")
async def documents_health():
    """Collection name, points count and embed model (status: healthy / unhealthy / unconfigured)."""
    import asyncio

    return await asyncio.to_thread(get_store().health)
