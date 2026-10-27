"""Learning progress sync: completed lessons and best quiz scores, per signed-in user."""
from typing import Dict, List

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel, Field, model_validator
from sqlalchemy.orm import Session

from ...auth.utils import verify_token
from ...database.database import get_db
from ...database.models import QuizScore, User as UserModel, UserProgress

router = APIRouter()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/v1/auth/token")

MAX_ITEMS = 1000


class QuizResult(BaseModel):
    best: int = Field(..., ge=0, le=10000)
    total: int = Field(..., ge=0, le=10000)

    @model_validator(mode="after")
    def _best_within_total(self):
        if self.best > self.total:
            raise ValueError("best cannot be greater than total")
        return self


class ProgressState(BaseModel):
    completed: List[str] = Field(default_factory=list, max_length=MAX_ITEMS)
    quizzes: Dict[str, QuizResult] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _limits(self):
        if len(self.quizzes) > MAX_ITEMS:
            raise ValueError("too many quizzes")
        if any(not d.strip() or len(d) > 300 for d in self.completed):
            raise ValueError("invalid doc_id")
        if any(not p.strip() or len(p) > 300 for p in self.quizzes):
            raise ValueError("invalid page_path")
        return self


def get_progress_user_id(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)) -> str:
    """Bearer token -> user id, and the user must still exist (a deleted account's token stops working)."""
    unauthorized = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    payload = verify_token(token)
    user_id = payload.get("user_id") if payload else None
    if not user_id or db.query(UserModel.id).filter(UserModel.id == user_id).first() is None:
        raise unauthorized
    return user_id


def _state(db: Session, user_id: str) -> ProgressState:
    completed = [
        r.doc_id for r in db.query(UserProgress).filter(UserProgress.user_id == user_id).order_by(UserProgress.doc_id)
    ]
    quizzes = {
        r.page_path: QuizResult(best=r.best_score, total=r.total)
        for r in db.query(QuizScore).filter(QuizScore.user_id == user_id).order_by(QuizScore.page_path)
    }
    return ProgressState(completed=completed, quizzes=quizzes)


@router.get("/", response_model=ProgressState)
@router.get("", response_model=ProgressState, include_in_schema=False)
def get_progress(user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    return _state(db, user_id)


@router.put("/", response_model=ProgressState)
@router.put("", response_model=ProgressState, include_in_schema=False)
def merge_progress(body: ProgressState, user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    """Merge, never overwrite: completed lessons are unioned, quiz bests take the maximum."""
    have = {r.doc_id for r in db.query(UserProgress).filter(UserProgress.user_id == user_id)}
    for doc_id in dict.fromkeys(d.strip() for d in body.completed):
        if doc_id not in have:
            db.add(UserProgress(user_id=user_id, doc_id=doc_id))

    existing = {r.page_path: r for r in db.query(QuizScore).filter(QuizScore.user_id == user_id)}
    for page_path, result in body.quizzes.items():
        row = existing.get(page_path)
        if row is None:
            db.add(QuizScore(user_id=user_id, page_path=page_path, best_score=result.best, total=result.total))
        elif result.best > row.best_score:
            row.best_score, row.total = result.best, result.total
    db.commit()
    return _state(db, user_id)


@router.delete("/completed/{doc_id:path}", response_model=ProgressState)
def uncomplete(doc_id: str, user_id: str = Depends(get_progress_user_id), db: Session = Depends(get_db)):
    db.query(UserProgress).filter(UserProgress.user_id == user_id, UserProgress.doc_id == doc_id).delete()
    db.commit()
    return _state(db, user_id)
