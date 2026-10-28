"""Qdrant wrapper for the course index. The client is created lazily so the API boots without Qdrant."""
from __future__ import annotations

import logging
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set

from app.config import get_settings

logger = logging.getLogger(__name__)

UPSERT_BATCH = 64


class StoreUnavailable(RuntimeError):
    """Qdrant is not configured or cannot be reached."""


@dataclass
class Hit:
    id: str
    score: float
    payload: Dict[str, Any]


def point_id(url: str, chunk_index: int) -> str:
    """Deterministic id so re-ingesting overwrites instead of duplicating."""
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"{url}#{chunk_index}"))


def normalize_page_url(value: str) -> str:
    """'/docs/ros2/foo/#bar' -> '/docs/ros2/foo' (the root '/docs/' becomes '/docs')."""
    value = (value or "").split("#", 1)[0].split("?", 1)[0].strip()
    return value.rstrip("/") or "/"


class CourseStore:
    def __init__(self, client: Any = None, collection: Optional[str] = None) -> None:
        settings = get_settings()
        self._client = client
        self.collection = collection or settings.qdrant_collection_name
        self.url = settings.qdrant_url
        self.api_key = settings.qdrant_api_key

    # -- connection ---------------------------------------------------------------------------
    @property
    def configured(self) -> bool:
        return self._client is not None or bool(self.url)

    @property
    def client(self):
        if self._client is None:
            if not self.url:
                raise StoreUnavailable("QDRANT_URL is not set")
            from qdrant_client import QdrantClient

            self._client = QdrantClient(url=self.url, api_key=self.api_key, timeout=20)
        return self._client

    # -- collection ---------------------------------------------------------------------------
    def ensure_collection(self) -> None:
        from qdrant_client.http import models

        client = self.client
        size = get_settings().embed_dim
        existing = {c.name for c in client.get_collections().collections}
        if self.collection not in existing:
            client.create_collection(
                collection_name=self.collection,
                vectors_config=models.VectorParams(size=size, distance=models.Distance.COSINE),
            )
            logger.info("Created Qdrant collection %s (%d-d)", self.collection, size)
        else:
            current = getattr(client.get_collection(self.collection).config.params.vectors, "size", size)
            if current != size:
                raise StoreUnavailable(
                    f"Collection '{self.collection}' holds {current}-d vectors but EMBED_DIM is {size}: "
                    "set QDRANT_COLLECTION_NAME to a new name (or delete the old collection) and re-index."
                )
        for field in ("url", "page_url", "module"):
            try:
                client.create_payload_index(
                    collection_name=self.collection,
                    field_name=field,
                    field_schema=models.PayloadSchemaType.KEYWORD,
                )
            except Exception as exc:  # already exists, or local mode without indexes
                logger.debug("payload index %s: %s", field, exc)

    # -- writes -------------------------------------------------------------------------------
    def upsert(self, chunks: Sequence[Any], vectors: Sequence[Sequence[float]]) -> List[str]:
        """Upsert chunks (app.rag.chunking.Chunk) with their vectors. Returns the point ids written."""
        from qdrant_client.http import models

        if len(chunks) != len(vectors):
            raise ValueError("chunks and vectors differ in length")
        ids: List[str] = []
        points = []
        for chunk, vector in zip(chunks, vectors):
            pid = point_id(chunk.url, chunk.chunk_index)
            ids.append(pid)
            points.append(
                models.PointStruct(
                    id=pid,
                    vector=list(vector),
                    payload={
                        "page_title": chunk.page_title,
                        "section": chunk.section,
                        "anchor": chunk.anchor,
                        "url": chunk.url,
                        "page_url": normalize_page_url(chunk.page_url),
                        "module": chunk.module,
                        "text": chunk.text,
                        "chunk_index": chunk.chunk_index,
                        "path": chunk.path,
                    },
                )
            )
        for i in range(0, len(points), UPSERT_BATCH):
            self.client.upsert(collection_name=self.collection, points=points[i:i + UPSERT_BATCH], wait=True)
        return ids

    def all_ids(self) -> Set[str]:
        found: Set[str] = set()
        offset = None
        while True:
            points, offset = self.client.scroll(
                collection_name=self.collection, limit=512, offset=offset, with_payload=False, with_vectors=False
            )
            found.update(str(p.id) for p in points)
            if offset is None:
                return found

    def delete_stale(self, keep_ids: Iterable[str]) -> int:
        """Delete every point whose id is not in `keep_ids`. Returns how many were removed."""
        from qdrant_client.http import models

        stale = sorted(self.all_ids() - set(keep_ids))
        for i in range(0, len(stale), 256):
            self.client.delete(
                collection_name=self.collection,
                points_selector=models.PointIdsList(points=stale[i:i + 256]),
                wait=True,
            )
        return len(stale)

    # -- reads --------------------------------------------------------------------------------
    def search(
        self,
        vector: Sequence[float],
        top_k: int,
        score_threshold: Optional[float] = None,
        url_prefix: Optional[str] = None,
    ) -> List[Hit]:
        """Nearest chunks. `url_prefix` restricts to one lesson page (anchors and trailing slashes ignored)."""
        from qdrant_client.http import models

        query_filter = None
        if url_prefix:
            page = normalize_page_url(url_prefix)
            query_filter = models.Filter(
                must=[models.FieldCondition(key="page_url", match=models.MatchValue(value=page))]
            )
        response = self.client.query_points(
            collection_name=self.collection,
            query=list(vector),
            limit=top_k,
            score_threshold=score_threshold,
            query_filter=query_filter,
            with_payload=True,
        )
        return [Hit(id=str(p.id), score=float(p.score), payload=dict(p.payload or {})) for p in response.points]

    def health(self) -> Dict[str, Any]:
        settings = get_settings()
        info: Dict[str, Any] = {
            "collection": self.collection,
            "embed_model": settings.embed_model,
        }
        if not self.configured:
            return {"status": "unconfigured", "message": "QDRANT_URL is not set", **info}
        try:
            if not self.client.collection_exists(self.collection):
                return {"status": "empty", "message": "Not indexed yet", **info}
            count = self.client.count(collection_name=self.collection, exact=True).count
            return {"status": "healthy", "points_count": count, "vectors_count": count, **info}
        except Exception as exc:
            logger.warning("Qdrant health check failed: %s", exc)
            return {"status": "unhealthy", "message": "Could not read the collection", **info}


_store: Optional[CourseStore] = None


def get_store() -> CourseStore:
    global _store
    if _store is None:
        _store = CourseStore()
    return _store


def reset_store() -> None:
    global _store
    _store = None
