#!/usr/bin/env python3
"""Re-index the course into Qdrant (live: needs GEMINI_API_KEY and QDRANT_URL; free tier).

Same as: python -m app.rag.ingest --docs ../frontend/docs   (add --dry-run to only chunk)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from dotenv import load_dotenv  # noqa: E402

load_dotenv(os.path.join(HERE, ".env"))

from app.rag.ingest import main  # noqa: E402

if __name__ == "__main__":
    args = sys.argv[1:]
    if "--docs" not in args:
        args = ["--docs", os.path.join(HERE, "..", "frontend", "docs"), *args]
    sys.exit(main(args))
