"""service.retrieve and the Qdrant store, against fakes and Qdrant's in-memory mode (no network)."""
import asyncio

from qdrant_client import QdrantClient

from app.rag import service
from app.rag.chunking import Chunk
from app.rag.store import CourseStore, Hit, normalize_page_url, point_id


def hit(i, score, page="/docs/a/b", url=None):
    return Hit(id=f"id{i}", score=score, payload={
        "page_title": f"T{i}", "section": f"S{i}", "url": url or f"{page}#s{i}", "text": f"text {i}", "page_url": page,
    })


class FakeStore:
    configured = True

    def __init__(self, page_hits, global_hits):
        self.page_hits, self.global_hits, self.calls = page_hits, global_hits, []

    def search(self, vector, top_k, score_threshold=None, url_prefix=None):
        self.calls.append((top_k, score_threshold, url_prefix))
        return list(self.page_hits if url_prefix else self.global_hits)


def test_retrieve_merges_page_and_global_hits_dedupes_and_keeps_best(monkeypatch):
    store = FakeStore(
        page_hits=[hit(1, 0.4), hit(2, 0.3), hit(3, 0.28)],
        global_hits=[hit(1, 0.4), hit(10, 0.9), hit(11, 0.8), hit(12, 0.7), hit(13, 0.6), hit(14, 0.5), hit(15, 0.45)],
    )
    monkeypatch.setattr(service, "get_store", lambda: store)
    monkeypatch.setattr(service, "embed_query", lambda q: [0.0] * 4)
    passages = asyncio.run(service.retrieve("q", {"title": "x", "path": "/physical-ai-textbook/docs/a/b/"}))

    assert len(passages) == 6  # RAG_TOP_K
    assert len({p.key for p in passages}) == 6  # id1 is not repeated
    assert [p.score for p in passages] == sorted((p.score for p in passages), reverse=True)
    assert {"id1", "id2", "id3"} <= {p.key for p in passages}  # the current page's best chunks are guaranteed
    page_call = next(c for c in store.calls if c[2])
    assert page_call[0] == 3 and page_call[2] == "/docs/a/b"  # baseUrl stripped, page-restricted top 3
    assert page_call[1] < next(c for c in store.calls if not c[2])[1]  # lower bar on the current page


def test_retrieve_without_page_only_searches_globally(monkeypatch):
    store = FakeStore([], [hit(1, 0.9)])
    monkeypatch.setattr(service, "get_store", lambda: store)
    monkeypatch.setattr(service, "embed_query", lambda q: [0.0])
    passages = asyncio.run(service.retrieve("q", None))
    assert [p.key for p in passages] == ["id1"]
    assert len(store.calls) == 1 and store.calls[0][2] is None


def test_retrieve_degrades_to_no_sources_when_qdrant_is_missing_or_broken(monkeypatch):
    # not configured at all (the default in tests)
    assert asyncio.run(service.retrieve("q", None)) == []

    class Broken(FakeStore):
        def search(self, *a, **k):
            raise ConnectionError("qdrant down")

    monkeypatch.setattr(service, "get_store", lambda: Broken([], []))
    monkeypatch.setattr(service, "embed_query", lambda q: [0.0])
    assert asyncio.run(service.retrieve("q", None)) == []


def test_page_url_from_path():
    assert service.page_url_from_path("/docs/ros2/foo/") == "/docs/ros2/foo"
    assert service.page_url_from_path("/physical-ai-textbook/docs/ros2/foo#bar") == "/docs/ros2/foo"
    assert service.page_url_from_path("/") is None
    assert service.page_url_from_path(None) is None
    assert normalize_page_url("/docs/") == "/docs"


def _chunk(url_page, anchor, i, module="ros2"):
    return Chunk(page_title="P", section=anchor, anchor=anchor, url=f"{url_page}#{anchor}", page_url=url_page,
                 module=module, text=f"text {anchor}", chunk_index=i, path="x.mdx")


def test_store_upsert_search_filter_and_stale_cleanup_in_memory():
    store = CourseStore(client=QdrantClient(":memory:"), collection="test_course")
    store.ensure_collection()
    store.ensure_collection()  # idempotent

    def vec(i):
        v = [0.0] * 3072
        v[i] = 1.0
        return v

    chunks = [_chunk("/docs/ros2/a", "one", 0), _chunk("/docs/ros2/a", "two", 1), _chunk("/docs/sim/b", "three", 0, "sim")]
    ids = store.upsert(chunks, [vec(0), vec(1), vec(2)])
    assert ids[0] == point_id("/docs/ros2/a#one", 0)
    assert store.upsert(chunks, [vec(0), vec(1), vec(2)]) == ids  # deterministic: no duplicates
    assert store.health()["points_count"] == 3

    top = store.search(vec(0), top_k=2, score_threshold=0.5)
    assert len(top) == 1 and top[0].payload["url"] == "/docs/ros2/a#one" and top[0].score > 0.99

    only_b = store.search(vec(0), top_k=5, score_threshold=None, url_prefix="/docs/sim/b/")
    assert [h.payload["page_url"] for h in only_b] == ["/docs/sim/b"]

    removed = store.delete_stale(ids[:2])  # the third chunk no longer exists in the docs
    assert removed == 1
    assert store.health()["points_count"] == 2


def test_ensure_collection_refuses_a_different_vector_size(monkeypatch):
    import pytest
    from qdrant_client import QdrantClient

    from app.config import get_settings
    from app.rag.store import CourseStore, StoreUnavailable

    store = CourseStore(client=QdrantClient(":memory:"), collection="size_test")
    store.ensure_collection()  # created at the default 3072
    monkeypatch.setenv("EMBED_DIM", "768")
    get_settings.cache_clear()
    with pytest.raises(StoreUnavailable, match="3072-d vectors but EMBED_DIM is 768"):
        store.ensure_collection()
