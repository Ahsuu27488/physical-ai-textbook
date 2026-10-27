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
for name in ("OPENAI_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"):
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
    from app.ratelimit import chat_limiter

    get_settings.cache_clear()
    chat_limiter.reset()
    store.reset_store()
    yield
    get_settings.cache_clear()
    store.reset_store()


@pytest.fixture
def with_key(monkeypatch):
    """Pretend OPENAI_API_KEY is configured (no real client is ever built: tests fake OpenAI)."""
    from app.config import get_settings

    monkeypatch.setenv("OPENAI_API_KEY", "sk-test-not-real")
    get_settings.cache_clear()
    return "sk-test-not-real"
