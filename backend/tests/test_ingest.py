import sys
from pathlib import Path

from app.rag import embeddings, ingest
from app.rag.chunking import chunk_docs

LESSON = """---
title: "Nodes, topics and services"
sidebar_label: "Nodes"
description: "How ROS 2 nodes talk."
---

<LessonMeta level="Beginner" time="30 min" prereqs={["Python"]} />

A node is a process.

## Publishers

""" + " ".join(f"word{i}" for i in range(200)) + """

## Quiz time

<Quiz questions={[
  {q: "What is a node?", options: ["A process", "A wire"], answer: 0, explain: "It is a process."}
]} />
"""


def make_docs(tmp_path: Path) -> Path:
    docs = tmp_path / "docs"
    (docs / "03-ros2").mkdir(parents=True)
    (docs / "03-ros2" / "02-nodes-topics-services.mdx").write_text(LESSON, encoding="utf-8")
    (docs / "03-ros2" / "index.mdx").write_text('---\ntitle: "ROS 2"\n---\n\nWelcome to ROS 2 module. ' + "x " * 30, encoding="utf-8")
    (docs / "03-ros2" / "_category_.json").write_text("{}", encoding="utf-8")
    (docs / "index.mdx").write_text('---\ntitle: "Welcome"\n---\n\nHello learner. ' + "y " * 30, encoding="utf-8")
    return docs


def test_chunk_docs_walks_folders_and_skips_non_lessons(tmp_path):
    docs = make_docs(tmp_path)
    chunks = chunk_docs(docs)
    urls = {c.page_url for c in chunks}
    assert urls == {"/docs/", "/docs/ros2", "/docs/ros2/nodes-topics-services"}
    assert all(c.text.startswith("Lesson: ") for c in chunks)


def test_dry_run_prints_stats_and_never_touches_the_network(tmp_path, monkeypatch, capsys):
    docs = make_docs(tmp_path)

    def boom(*a, **k):
        raise AssertionError("dry run must not embed or connect")

    monkeypatch.setattr(embeddings, "embed_texts", boom)
    monkeypatch.setattr("app.rag.store.get_store", boom)
    assert ingest.main(["--docs", str(docs), "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "files:  3" in out
    assert "chunks: " in out and "ros2: " in out
    assert "Dry run" in out


def test_dry_run_as_a_module_subprocess(tmp_path):
    import subprocess

    docs = make_docs(tmp_path)
    backend = Path(__file__).resolve().parents[1]
    done = subprocess.run(
        [sys.executable, "-m", "app.rag.ingest", "--docs", str(docs), "--dry-run"],
        cwd=backend, capture_output=True, text=True, timeout=60,
        env={"PATH": "", "SYSTEMROOT": "C:\\Windows", "GEMINI_API_KEY": "", "LLM_API_KEY": "", "QDRANT_URL": ""},
    )
    assert done.returncode == 0, done.stderr
    assert "Dry run" in done.stdout


def test_missing_docs_folder_is_an_error(tmp_path, capsys):
    assert ingest.main(["--docs", str(tmp_path / "nope"), "--dry-run"]) == 2


def test_live_run_without_keys_fails_cleanly(tmp_path, capsys):
    docs = make_docs(tmp_path)
    assert ingest.main(["--docs", str(docs)]) == 2
    assert "GEMINI_API_KEY" in capsys.readouterr().err


def test_live_run_embeds_upserts_and_cleans_up(tmp_path, monkeypatch, with_key):
    from qdrant_client import QdrantClient

    from app.rag import store as store_mod

    docs = make_docs(tmp_path)
    fake_store = store_mod.CourseStore(client=QdrantClient(":memory:"), collection="ingest_test")
    monkeypatch.setattr(store_mod, "get_store", lambda: fake_store)

    def fake_embed(texts, **kw):
        vectors = []
        for i, _ in enumerate(texts):
            v = [0.0] * 3072
            v[i % 3072] = 1.0
            vectors.append(v)
        return vectors

    monkeypatch.setattr(embeddings, "embed_texts", fake_embed)
    assert ingest.run(docs) == 0
    first = fake_store.health()["points_count"]
    assert first == len(chunk_docs(docs))
    # a lesson disappears: its points are removed on the next run
    (docs / "03-ros2" / "02-nodes-topics-services.mdx").unlink()
    assert ingest.run(docs) == 0
    assert fake_store.health()["points_count"] < first


def test_embed_texts_batches_of_50_and_raises_on_error(monkeypatch, with_key):
    from types import SimpleNamespace

    from app import llm

    sizes = []

    class Emb:
        def create(self, model, input):
            sizes.append(len(input))
            if len(sizes) == 4:
                raise RuntimeError("api down")
            return SimpleNamespace(data=[SimpleNamespace(index=i, embedding=[float(i)] * 3072) for i in range(len(input))])

    monkeypatch.setattr(llm, "get_openai_client", lambda: SimpleNamespace(embeddings=Emb()))
    out = embeddings.embed_texts(["t"] * 101)
    assert sizes == [50, 50, 1] and len(out) == 101
    import pytest

    with pytest.raises(RuntimeError):
        embeddings.embed_texts(["t"] * 5)


def test_embed_retries_rate_limits_then_gives_up(monkeypatch, with_key):
    from types import SimpleNamespace

    import pytest

    from app import llm

    class Limited(Exception):
        status_code = 429

    calls, waits = [], []

    class Emb:
        def create(self, model, input):
            calls.append(model)
            if len(calls) < 3:
                raise Limited("slow down")
            return SimpleNamespace(data=[SimpleNamespace(index=0, embedding=[0.5] * 3072)])

    monkeypatch.setattr(llm, "get_openai_client", lambda: SimpleNamespace(embeddings=Emb()))
    assert len(embeddings.embed_texts(["t"], sleep=waits.append)[0]) == 3072
    assert waits == [2.0, 4.0]

    class Always(Emb):
        def create(self, model, input):
            raise Limited("slow down")

    monkeypatch.setattr(llm, "get_openai_client", lambda: SimpleNamespace(embeddings=Always()))
    with pytest.raises(Limited):
        embeddings.embed_texts(["t"], sleep=lambda s: None)


def test_embed_rejects_vectors_of_the_wrong_size(monkeypatch, with_key):
    from types import SimpleNamespace

    import pytest

    from app import llm

    class Emb:
        def create(self, model, input):
            return SimpleNamespace(data=[SimpleNamespace(index=0, embedding=[0.1] * 768)])

    monkeypatch.setattr(llm, "get_openai_client", lambda: SimpleNamespace(embeddings=Emb()))
    with pytest.raises(RuntimeError, match="768-d"):
        embeddings.embed_texts(["t"])


def test_gemini_embedding_2_task_prefixes():
    assert embeddings.as_query("what is a node?") == "task: search result | query: what is a node?"
    assert embeddings.as_document("body", "ROS 2") == "title: ROS 2 | text: body"
    assert embeddings.as_document("body") == "title: none | text: body"
    assert embeddings.as_query("q", model="text-embedding-3-small") == "q"
