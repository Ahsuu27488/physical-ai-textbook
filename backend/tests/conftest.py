import os
import sys
import tempfile
from pathlib import Path

import pytest

# A fresh database and no secrets: the app must still start and serve what doesn't need them.
_db = Path(tempfile.mkdtemp()) / "test.db"
os.environ["NEON_DATABASE_URL"] = f"sqlite:///{_db.as_posix()}"
os.environ["SECRET_KEY"] = "test-secret"
for name in ("OPENAI_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"):
    os.environ.pop(name, None)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture(scope="session")
def client():
    from fastapi.testclient import TestClient
    from app.main import app

    with TestClient(app) as c:  # runs the lifespan, which creates the tables
        yield c
