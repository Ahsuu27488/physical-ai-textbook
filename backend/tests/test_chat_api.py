"""/api/chat and /api/chat/stream with a fake retriever and a fake OpenAI. No network, no secrets."""
import json
from types import SimpleNamespace

import pytest

from app import llm
from app.rag import service
from app.rag.service import Passage

PASSAGES = [
    Passage("ROS 2 architecture", "Nodes", "/docs/ros2/architecture#nodes", "A node is a process.", 0.8),
    Passage("Frames", "Frames", "/docs/robot-fundamentals/frames", "A frame is a coordinate system.", 0.6),
]
TOKENS = ["A node ", "is a ", "process [1]."]


class FakeCompletions:
    def __init__(self, fail=None):
        self.calls = []
        self.fail = fail

    async def create(self, **kwargs):
        self.calls.append(kwargs)
        if self.fail:
            raise self.fail
        if kwargs.get("stream"):
            return self._stream()
        message = SimpleNamespace(content="".join(TOKENS))
        return SimpleNamespace(choices=[SimpleNamespace(message=message)])

    async def _stream(self):
        yield SimpleNamespace(choices=[])  # e.g. a usage-only chunk
        yield SimpleNamespace(choices=[SimpleNamespace(delta=SimpleNamespace(content=None))])
        for t in TOKENS:
            yield SimpleNamespace(choices=[SimpleNamespace(delta=SimpleNamespace(content=t))])


@pytest.fixture
def fake_openai(monkeypatch, with_key):
    completions = FakeCompletions()
    client = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    monkeypatch.setattr(llm, "get_async_openai_client", lambda: client)

    async def fake_retrieve(question, page=None):
        completions.retrieved_for = (question, page)
        return PASSAGES

    monkeypatch.setattr(service, "retrieve", fake_retrieve)
    return completions


def parse_sse(body: str):
    events = []
    for block in body.strip().split("\n\n"):
        lines = block.split("\n")
        assert lines[0].startswith("event: ") and lines[1].startswith("data: "), block
        events.append((lines[0][7:], json.loads(lines[1][6:])))
    return events


def test_chat_returns_answer_sources_and_conversation_id(client, fake_openai):
    r = client.post("/api/chat", json={
        "message": "What is a node?",
        "history": [{"role": "user", "content": "hi"}, {"role": "assistant", "content": "hello"}],
        "page": {"title": "ROS 2 architecture", "path": "/docs/ros2/architecture"},
    })
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["response"] == "A node is a process [1]."
    assert len(body["conversation_id"]) == 36
    assert [s["n"] for s in body["sources"]] == [1, 2]
    assert body["sources"][0] == {"n": 1, "title": "ROS 2 architecture", "section": "Nodes",
                                  "url": "/docs/ros2/architecture#nodes", "score": 0.8}
    # the prompt carried the excerpts, the history and the page
    messages = fake_openai.calls[0]["messages"]
    assert "[1] ROS 2 architecture — Nodes" in messages[0]["content"]
    assert [m["role"] for m in messages] == ["system", "user", "assistant", "user"]
    assert fake_openai.retrieved_for[1].path == "/docs/ros2/architecture"
    assert fake_openai.calls[0]["model"] == "gpt-4o-mini"


def test_chat_keeps_the_given_conversation_id(client, fake_openai):
    r = client.post("/api/chat", json={"message": "hi", "conversation_id": "abc-123"})
    assert r.json()["conversation_id"] == "abc-123"


def test_chat_validates_the_message(client, fake_openai):
    assert client.post("/api/chat", json={"message": ""}).status_code == 422
    assert client.post("/api/chat", json={"message": "x" * 4001}).status_code == 422
    assert client.post("/api/chat", json={"message": "hi", "history": [{"role": "robot", "content": "x"}]}).status_code == 422


def test_stream_event_order_and_joined_text(client, fake_openai):
    with client.stream("POST", "/api/chat/stream", json={"message": "What is a node?"}) as r:
        assert r.status_code == 200
        assert r.headers["content-type"].startswith("text/event-stream")
        assert r.headers["cache-control"] == "no-cache"
        assert r.headers["x-accel-buffering"] == "no"
        body = "".join(r.iter_text())
    events = parse_sse(body)
    names = [n for n, _ in events]
    assert names[0] == "meta" and names[-1] == "done"
    assert names[1:-1] == ["token"] * len(TOKENS)
    meta = events[0][1]
    assert len(meta["conversation_id"]) == 36
    assert [s["n"] for s in meta["sources"]] == [1, 2]
    assert "".join(d["text"] for n, d in events if n == "token") == "".join(TOKENS)
    assert fake_openai.calls[0]["stream"] is True


def test_stream_reports_model_failures_as_an_error_event(client, monkeypatch, with_key):
    class Boom(Exception):
        status_code = 429

    completions = FakeCompletions(fail=Boom("secret details"))
    monkeypatch.setattr(llm, "get_async_openai_client", lambda: SimpleNamespace(chat=SimpleNamespace(completions=completions)))

    async def no_passages(question, page=None):
        return []

    monkeypatch.setattr(service, "retrieve", no_passages)
    with client.stream("POST", "/api/chat/stream", json={"message": "hi"}) as r:
        body = "".join(r.iter_text())
    events = parse_sse(body)
    assert [n for n, _ in events] == ["meta", "error"]
    assert events[0][1]["sources"] == []
    assert events[1][1]["status"] == 429
    assert "secret" not in body


def test_chat_model_failure_is_a_clean_502_or_429(client, monkeypatch, with_key):
    completions = FakeCompletions(fail=RuntimeError("secret details"))
    monkeypatch.setattr(llm, "get_async_openai_client", lambda: SimpleNamespace(chat=SimpleNamespace(completions=completions)))

    async def no_passages(question, page=None):
        return []

    monkeypatch.setattr(service, "retrieve", no_passages)
    r = client.post("/api/chat", json={"message": "hi"})
    assert r.status_code == 502
    assert "secret" not in r.text


def test_503_without_openai_key_on_both_endpoints(client):
    r = client.post("/api/chat", json={"message": "hi"})
    assert r.status_code == 503 and "OPENAI_API_KEY" in r.json()["detail"]
    r = client.post("/api/chat/stream", json={"message": "hi"})
    assert r.status_code == 503
    assert r.headers["content-type"].startswith("application/json")
    assert "OPENAI_API_KEY" in r.json()["detail"]


def test_rate_limit_returns_429_on_chat_and_stream(client, fake_openai, monkeypatch):
    monkeypatch.setenv("CHAT_RATE_LIMIT_PER_MIN", "3")
    from app.config import get_settings

    get_settings.cache_clear()
    assert client.post("/api/chat", json={"message": "1"}).status_code == 200
    assert client.post("/api/chat", json={"message": "2"}).status_code == 200
    with client.stream("POST", "/api/chat/stream", json={"message": "3"}) as r:
        assert r.status_code == 200
        "".join(r.iter_text())
    r = client.post("/api/chat", json={"message": "4"})
    assert r.status_code == 429
    assert "3 questions per minute" in r.json()["detail"]
    assert int(r.headers["retry-after"]) >= 1
    assert client.post("/api/chat/stream", json={"message": "5"}).status_code == 429


def test_rate_limit_is_per_client_ip(client, fake_openai, monkeypatch):
    monkeypatch.setenv("CHAT_RATE_LIMIT_PER_MIN", "1")
    from app.config import get_settings

    get_settings.cache_clear()
    assert client.post("/api/chat", json={"message": "a"}, headers={"X-Forwarded-For": "1.1.1.1"}).status_code == 200
    assert client.post("/api/chat", json={"message": "a"}, headers={"X-Forwarded-For": "1.1.1.1"}).status_code == 429
    assert client.post("/api/chat", json={"message": "a"}, headers={"X-Forwarded-For": "2.2.2.2"}).status_code == 200


def test_documents_health_without_qdrant(client):
    r = client.get("/api/documents/health")
    assert r.status_code == 200
    body = r.json()
    assert body["status"] == "unconfigured"
    assert body["collection"] == "proprio_course"
    assert body["embed_model"] == "text-embedding-3-small"


def test_search_returns_source_objects(client, monkeypatch, with_key):
    async def fake_search(query, top_k):
        return PASSAGES[:top_k]

    monkeypatch.setattr(service, "search_passages", fake_search)
    r = client.post("/api/search", json={"query": "node", "top_k": 1})
    assert r.status_code == 200, r.text
    results = r.json()["results"]
    assert len(results) == 1
    assert results[0]["n"] == 1 and results[0]["url"] == "/docs/ros2/architecture#nodes"
    assert results[0]["text"] == "A node is a process."


def test_cors_allows_local_dev_and_site_origins(client):
    for origin in ("http://localhost:3000", "http://localhost:3123", "https://physical-ai-textbook-puce-two.vercel.app"):
        r = client.options("/api/chat", headers={"Origin": origin, "Access-Control-Request-Method": "POST"})
        assert r.headers.get("access-control-allow-origin") == origin


def test_settings_cors_origins_parse_comma_lists(monkeypatch):
    from app.config import Settings

    s = Settings(_env_file=None, additional_cors_origin="https://a.example.com/, https://b.example.com", frontend_url="")
    assert "https://a.example.com" in s.cors_origins and "https://b.example.com" in s.cors_origins
    assert "http://localhost:3123" in s.cors_origins
    assert len(s.cors_origins) == len(set(s.cors_origins))
