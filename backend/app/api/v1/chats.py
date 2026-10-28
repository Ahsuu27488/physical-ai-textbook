"""Rivet conversations saved to the account (continue on another device) and thumbs up/down on answers."""
from datetime import datetime
from typing import List, Literal, Optional

from fastapi import APIRouter, Depends, HTTPException, Response, status
from pydantic import BaseModel, Field, model_validator
from sqlalchemy.orm import Session

from ...database.database import get_db
from ...database.models import AnswerFeedback, ChatThread
from .progress import get_progress_user_id

router = APIRouter()

MAX_THREADS = 50  # per learner; the oldest beyond this are dropped
MAX_MESSAGES = 80
MAX_TEXT = 20000  # per message
MAX_SOURCES = 12


class Source(BaseModel):
    n: int = Field(..., ge=0, le=100)
    title: str = Field("", max_length=300)
    section: str = Field("", max_length=300)
    url: str = Field("", max_length=500)


class Message(BaseModel):
    id: str = Field(..., min_length=1, max_length=64)
    role: Literal["user", "assistant"]
    text: str = Field(..., max_length=MAX_TEXT)
    sources: List[Source] = Field(default_factory=list, max_length=MAX_SOURCES)
    feedback: Optional[Literal[1, -1]] = None
    stopped: bool = False


class ThreadIn(BaseModel):
    title: str = Field("", max_length=200)
    messages: List[Message] = Field(..., min_length=1, max_length=MAX_MESSAGES)


class ThreadSummary(BaseModel):
    id: str
    title: str
    updated_at: Optional[datetime]
    count: int


class Thread(ThreadSummary):
    messages: List[Message]


class FeedbackIn(BaseModel):
    conversation_id: str = Field(..., min_length=1, max_length=100)
    message_id: str = Field(..., min_length=1, max_length=64)
    rating: Optional[Literal[1, -1]]  # null takes the vote back
    question: str = Field("", max_length=4000)
    answer: str = Field("", max_length=MAX_TEXT)
    page: Optional[str] = Field(None, max_length=300)

    @model_validator(mode="after")
    def _strip(self):
        self.question, self.answer = self.question.strip(), self.answer.strip()
        return self


def _title(messages: List[Message], title: str) -> str:
    if title.strip():
        return title.strip()[:200]
    first = next((m.text for m in messages if m.role == "user" and m.text.strip()), "New chat")
    first = " ".join(first.split())
    return first if len(first) <= 80 else first[:79].rstrip() + "…"


def _summary(t: ChatThread) -> ThreadSummary:
    return ThreadSummary(id=t.id, title=t.title, updated_at=t.updated_at, count=len(t.messages or []))


def _own(db: Session, user_id: str, thread_id: str) -> Optional[ChatThread]:
    return db.query(ChatThread).filter(ChatThread.id == thread_id, ChatThread.user_id == user_id).first()


@router.get("/", response_model=List[ThreadSummary])
@router.get("", response_model=List[ThreadSummary], include_in_schema=False)
def list_threads(user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    rows = (
        db.query(ChatThread)
        .filter(ChatThread.user_id == user_id)
        .order_by(ChatThread.updated_at.desc())
        .limit(MAX_THREADS)
        .all()
    )
    return [_summary(t) for t in rows]


@router.get("/{thread_id}", response_model=Thread)
def get_thread(thread_id: str, user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    t = _own(db, user_id, thread_id)
    if t is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, "Chat not found")
    return Thread(**_summary(t).model_dump(), messages=t.messages or [])


@router.put("/{thread_id}", response_model=ThreadSummary)
def save_thread(thread_id: str, body: ThreadIn, user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    """Create or replace a conversation (the browser holds the whole thing and saves it after each answer)."""
    if not thread_id or len(thread_id) > 100:
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_ENTITY, "Invalid chat id")
    messages = [m.model_dump() for m in body.messages]
    t = _own(db, user_id, thread_id)
    if t is None:
        if db.query(ChatThread.id).filter(ChatThread.id == thread_id).first() is not None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, "Chat not found")  # someone else's id
        t = ChatThread(id=thread_id, user_id=user_id, title=_title(body.messages, body.title), messages=messages)
        db.add(t)
    else:
        t.messages = messages
        t.title = _title(body.messages, body.title)
        t.updated_at = datetime.utcnow()
    db.commit()
    # keep only the newest MAX_THREADS
    stale = (
        db.query(ChatThread.id)
        .filter(ChatThread.user_id == user_id)
        .order_by(ChatThread.updated_at.desc())
        .offset(MAX_THREADS)
        .all()
    )
    if stale:
        db.query(ChatThread).filter(ChatThread.id.in_([s.id for s in stale])).delete(synchronize_session=False)
        db.commit()
    db.refresh(t)
    return _summary(t)


@router.delete("/{thread_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_thread(thread_id: str, user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    db.query(ChatThread).filter(ChatThread.id == thread_id, ChatThread.user_id == user_id).delete(synchronize_session=False)
    db.commit()
    return Response(status_code=status.HTTP_204_NO_CONTENT)


@router.post("/feedback")
def answer_feedback(body: FeedbackIn, user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    """One vote per answer per learner; a new vote replaces the old one, null removes it."""
    row = (
        db.query(AnswerFeedback)
        .filter(
            AnswerFeedback.user_id == user_id,
            AnswerFeedback.conversation_id == body.conversation_id,
            AnswerFeedback.message_id == body.message_id,
        )
        .first()
    )
    if body.rating is None:
        if row is not None:
            db.delete(row)
            db.commit()
        return {"rating": None}
    if row is None:
        row = AnswerFeedback(user_id=user_id, conversation_id=body.conversation_id, message_id=body.message_id)
        db.add(row)
    row.rating, row.question, row.answer, row.page = body.rating, body.question, body.answer, body.page
    db.commit()
    return {"rating": row.rating}
