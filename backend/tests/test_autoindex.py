"""The API indexes the deployed lessons itself: first run embeds, an unchanged run is free, a change re-indexes."""
from qdrant_client import QdrantClient

from app.rag import autoindex, embeddings
from app.rag.store import CourseStore
from test_ingest import make_docs


def fake_embed(calls):
    def embed(texts, **kw):
        calls.append(len(texts))
        if kw.get("on_batch"):
            kw["on_batch"](len(texts), len(texts))
        vectors = []
        for i, _ in enumerate(texts):
            v = [0.0] * 3072
            v[i % 3072] = 1.0
            vectors.append(v)
        return vectors

    return embed


def test_indexes_once_then_skips_until_lessons_change(tmp_path, monkeypatch, with_key):
    docs = make_docs(tmp_path)
    store = CourseStore(client=QdrantClient(":memory:"), collection="auto_test")
    calls = []
    monkeypatch.setattr(embeddings, "embed_texts", fake_embed(calls))

    first = autoindex.run_once(store, docs)
    assert first["state"] == "indexed" and first["chunks"] > 0
    assert store.health()["points_count"] == first["chunks"]

    second = autoindex.run_once(store, docs)
    assert second["state"] == "up to date" and len(calls) == 1  # nothing embedded the second time

    (docs / "03-ros2" / "02-nodes-topics-services.mdx").unlink()
    third = autoindex.run_once(store, docs)
    assert third["state"] == "indexed" and third["removed"] > 0 and len(calls) == 2


def test_skips_without_lessons_or_key(tmp_path, monkeypatch):
    assert autoindex.run_once(docs=tmp_path / "missing")["state"] == "skipped"
    docs = make_docs(tmp_path)
    out = autoindex.run_once(CourseStore(client=QdrantClient(":memory:"), collection="x"), docs)
    assert out["state"] == "skipped" and "GEMINI_API_KEY" in out["reason"]


def test_failure_reports_the_error_kind_not_its_message(tmp_path, monkeypatch, with_key):
    docs = make_docs(tmp_path)

    class Unauthorized(Exception):
        status_code = 401

    def boom(texts, **kw):
        raise Unauthorized("key=secret-value")

    monkeypatch.setattr(embeddings, "embed_texts", boom)
    out = autoindex.run_once(CourseStore(client=QdrantClient(":memory:"), collection="y"), docs)
    assert out["state"] == "failed" and out["reason"] == "Unauthorized (401)"
    assert "secret" not in str(out)


def test_health_says_empty_before_indexing(monkeypatch):
    store = CourseStore(client=QdrantClient(":memory:"), collection="never_made")
    assert store.health()["status"] == "empty"
