"""Small in-memory sliding-window rate limits for Rivet, per signed-in learner (per process; fine for one Space)."""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

from fastapi import HTTPException, status

from app.config import get_settings

WINDOW_SECONDS = 60.0
DAY_SECONDS = 24 * 60 * 60.0


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
daily_limiter = SlidingWindowLimiter(DAY_SECONDS)


def _too_many(detail: str, retry_after: int) -> HTTPException:
    return HTTPException(
        status_code=status.HTTP_429_TOO_MANY_REQUESTS, detail=detail, headers={"Retry-After": str(retry_after)}
    )


def enforce_chat_rate_limit(user_id: str) -> None:
    """Per-minute and per-day limits for one learner (keyed on the account, not the IP)."""
    settings = get_settings()
    limit = settings.chat_rate_limit_per_min
    allowed, retry_after = chat_limiter.check(user_id, limit)
    if not allowed:
        raise _too_many(
            f"Slow down a little: Rivet answers up to {limit} questions per minute. Try again in {retry_after} seconds.",
            retry_after,
        )
    daily = settings.chat_daily_limit_per_user
    allowed, retry_after = daily_limiter.check(user_id, daily)
    if not allowed:
        hours = max(1, round(retry_after / 3600))
        raise _too_many(
            f"You've asked Rivet {daily} questions today, the daily limit on the free plan. More in about {hours} h.",
            retry_after,
        )
