import itertools
import os
import sys
import tempfile
from pathlib import Path

import pytest

# A fresh database and no secrets: the app must still start and serve what doesn't need them.
_db = Path(tempfile.mkdtemp()) / "test.db"
os.environ["NEON_DATABASE_URL"] = f"sqlite:///{_db.as_posix()}"
os.environ["SECRET_KEY"] = "test-secret"
# Blank (not removed) so a developer's local .env cannot leak a real key into the tests.
for name in ("GEMINI_API_KEY", "LLM_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"):
    os.environ[name] = ""

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture(scope="session")
def client():
    from fastapi.testclient import TestClient
    from app.main import app

    with TestClient(app) as c:  # runs the lifespan, which creates the tables
        yield c


@pytest.fixture(autouse=True)
def _fresh_state():
    """Every test starts with fresh settings, a fresh rate limiter and no cached store."""
    from app.config import get_settings
    from app.rag import store
    from app.ratelimit import chat_limiter, daily_limiter

    get_settings.cache_clear()
    chat_limiter.reset()
    daily_limiter.reset()
    store.reset_store()
    yield
    get_settings.cache_clear()
    store.reset_store()


@pytest.fixture
def with_key(monkeypatch):
    """Pretend GEMINI_API_KEY is configured (no real client is ever built: tests fake the model API)."""
    from app.config import get_settings

    monkeypatch.setenv("GEMINI_API_KEY", "test-key-not-real")
    get_settings.cache_clear()
    return "test-key-not-real"


_counter = itertools.count(1)


def sign_in(client, email=None, password="pw-123456"):
    """Register + sign in a fresh user. Returns (headers, email, password)."""
    email = email or f"learner{next(_counter)}@example.com"
    r = client.post("/api/v1/auth/register", json={
        "email": email, "password": password, "name": "Ada",
        "software_background": "beginner", "hardware_background": "intermediate",
    })
    assert r.status_code == 200, r.text
    r = client.post("/api/v1/auth/token", data={"username": email, "password": password})
    assert r.status_code == 200, r.text
    return {"Authorization": f"Bearer {r.json()['access_token']}"}, email, password


@pytest.fixture
def user(client):
    return sign_in(client)


class SignedIn:
    """The test client with a learner's bearer token on every request."""

    def __init__(self, client, headers):
        self.client, self.headers = client, headers

    def post(self, url, **kw):
        return self.client.post(url, headers={**self.headers, **kw.pop("headers", {})}, **kw)

    def stream(self, method, url, **kw):
        return self.client.stream(method, url, headers={**self.headers, **kw.pop("headers", {})}, **kw)


@pytest.fixture
def api(client, user):
    return SignedIn(client, user[0])
