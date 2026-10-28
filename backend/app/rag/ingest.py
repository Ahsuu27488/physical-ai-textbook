"""Index the course into Qdrant.

    python -m app.rag.ingest --docs ../frontend/docs            # chunk, embed, upsert, clean stale points
    python -m app.rag.ingest --docs ../frontend/docs --dry-run  # chunk only: no network, no keys needed
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path
from typing import List, Optional, Sequence

from app.rag.chunking import Chunk, chunk_docs, iter_doc_files


def stats(chunks: Sequence[Chunk], files: int) -> dict:
    words = [len(c.body.split()) for c in chunks]
    modules = Counter(c.module for c in chunks)
    return {
        "files": files,
        "chunks": len(chunks),
        "words_total": sum(words),
        "words_min": min(words) if words else 0,
        "words_avg": round(sum(words) / len(words)) if words else 0,
        "words_max": max(words) if words else 0,
        "modules": dict(sorted(modules.items())),
    }


def print_stats(info: dict) -> None:
    print(f"files:  {info['files']}")
    print(f"chunks: {info['chunks']}")
    print(f"words per chunk: min {info['words_min']} / avg {info['words_avg']} / max {info['words_max']}")
    print("chunks per module:")
    for module, count in info["modules"].items():
        print(f"  {module or '(root)'}: {count}")


def run(docs: Path, dry_run: bool = False) -> int:
    docs = Path(docs)
    if not docs.is_dir():
        print(f"error: docs folder not found: {docs}", file=sys.stderr)
        return 2
    files = sum(1 for _ in iter_doc_files(docs))
    chunks = chunk_docs(docs)
    print(f"Read {files} lesson files from {docs}")
    print_stats(stats(chunks, files))
    if not chunks:
        print("No chunks produced; nothing to do.", file=sys.stderr)
        return 1
    if dry_run:
        print("Dry run: nothing was embedded or written.")
        return 0

    # Imported here so --dry-run works without any configuration or network.
    from app.config import get_settings
    from app.rag.embeddings import as_document, embed_texts
    from app.rag.store import get_store

    settings = get_settings()
    if not settings.llm_api_key:
        print("error: GEMINI_API_KEY is not set", file=sys.stderr)
        return 2
    store = get_store()
    if not store.configured:
        print("error: QDRANT_URL is not set", file=sys.stderr)
        return 2

    print(f"Embedding {len(chunks)} chunks with {settings.embed_model} ...")
    vectors = embed_texts(
        [as_document(c.text, c.page_title) for c in chunks],
        on_batch=lambda done, total: print(f"  embedded {done}/{total}"),
    )
    store.ensure_collection()
    ids = store.upsert(chunks, vectors)
    removed = store.delete_stale(ids)
    print(f"Upserted {len(ids)} points into '{store.collection}'; removed {removed} stale points.")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Chunk, embed and index the Kinemata course for Rivet.")
    parser.add_argument("--docs", default="../frontend/docs", help="docs folder (default: ../frontend/docs)")
    parser.add_argument("--dry-run", action="store_true", help="only chunk and print stats (no network)")
    args = parser.parse_args(argv)
    return run(Path(args.docs), dry_run=args.dry_run)


if __name__ == "__main__":
    # Load .env for the live run (the app itself reads it through app.config).
    try:
        from dotenv import load_dotenv

        load_dotenv()
    except ImportError:
        pass
    sys.exit(main())
