"""A small in-memory sliding-window rate limiter (per process; fine for a single Space/dyno)."""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

from fastapi import HTTPException, Request, status

from app.config import get_settings

WINDOW_SECONDS = 60.0


class SlidingWindowLimiter:
    def __init__(self, window: float = WINDOW_SECONDS) -> None:
        self.window = window
        self._hits: Dict[str, Deque[float]] = {}
        self._lock = threading.Lock()

    def check(self, key: str, limit: int, now: Optional[float] = None) -> Tuple[bool, int]:
        """Record a hit. Returns (allowed, retry_after_seconds). limit <= 0 disables the limiter."""
        if limit <= 0:
            return True, 0
        now = time.monotonic() if now is None else now
        with self._lock:
            q = self._hits.setdefault(key, deque())
            while q and now - q[0] >= self.window:
                q.popleft()
            if len(q) >= limit:
                return False, max(1, math.ceil(self.window - (now - q[0])))
            q.append(now)
            if len(self._hits) > 5000:  # keep memory bounded
                for k in [k for k, v in self._hits.items() if not v or now - v[-1] >= self.window]:
                    self._hits.pop(k, None)
            return True, 0

    def reset(self) -> None:
        with self._lock:
            self._hits.clear()


chat_limiter = SlidingWindowLimiter()


def client_ip(request: Request) -> str:
    forwarded = request.headers.get("x-forwarded-for", "")
    if forwarded:
        return forwarded.split(",")[0].strip() or "unknown"
    return request.client.host if request.client else "unknown"


def enforce_chat_rate_limit(request: Request) -> None:
    """FastAPI dependency for the chat endpoints."""
    limit = get_settings().chat_rate_limit_per_min
    allowed, retry_after = chat_limiter.check(client_ip(request), limit)
    if not allowed:
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=f"Slow down a little: Rivet answers up to {limit} questions per minute. Try again in {retry_after} seconds.",
            headers={"Retry-After": str(retry_after)},
        )
