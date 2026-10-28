"""Saved Rivet conversations and answer feedback."""
from conftest import sign_in


def _thread(text="What is a PID controller?"):
    return {"messages": [
        {"id": "u1", "role": "user", "text": text},
        {"id": "a1", "role": "assistant", "text": "A controller that...", "sources": [{"n": 1, "title": "PID", "section": "", "url": "/docs/motion/pid"}]},
    ]}


def test_needs_an_account(client):
    assert client.get("/api/v1/chats").status_code == 401
    assert client.put("/api/v1/chats/c1", json=_thread()).status_code == 401
    assert client.post("/api/v1/chats/feedback", json={"conversation_id": "c", "message_id": "a", "rating": 1}).status_code == 401


def test_save_list_load_and_delete(client, user):
    headers = user[0]
    r = client.put("/api/v1/chats/c1", json=_thread(), headers=headers)
    assert r.status_code == 200, r.text
    assert r.json()["title"] == "What is a PID controller?" and r.json()["count"] == 2

    body = _thread()
    body["messages"].append({"id": "u2", "role": "user", "text": "and tuning?"})
    assert client.put("/api/v1/chats/c1", json=body, headers=headers).json()["count"] == 3

    listed = client.get("/api/v1/chats", headers=headers).json()
    assert [t["id"] for t in listed] == ["c1"]
    loaded = client.get("/api/v1/chats/c1", headers=headers).json()
    assert loaded["messages"][1]["sources"][0]["url"] == "/docs/motion/pid"

    assert client.delete("/api/v1/chats/c1", headers=headers).status_code == 204
    assert client.get("/api/v1/chats/c1", headers=headers).status_code == 404


def test_another_learner_cannot_read_or_overwrite(client, user):
    assert client.put("/api/v1/chats/mine", json=_thread("secret question"), headers=user[0]).status_code == 200
    other = sign_in(client)[0]
    assert client.get("/api/v1/chats/mine", headers=other).status_code == 404
    assert client.put("/api/v1/chats/mine", json=_thread("hijack"), headers=other).status_code == 404
    assert client.get("/api/v1/chats", headers=other).json() == []
    assert client.get("/api/v1/chats/mine", headers=user[0]).json()["title"] == "secret question"


def test_limits(client, user):
    too_long = {"messages": [{"id": "u", "role": "user", "text": "x" * 20001}]}
    assert client.put("/api/v1/chats/c", json=too_long, headers=user[0]).status_code == 422
    assert client.put("/api/v1/chats/c", json={"messages": []}, headers=user[0]).status_code == 422
    bad_role = {"messages": [{"id": "u", "role": "system", "text": "x"}]}
    assert client.put("/api/v1/chats/c", json=bad_role, headers=user[0]).status_code == 422


def test_feedback_vote_change_and_take_back(client, user):
    from app.database.database import SessionLocal
    from app.database.models import AnswerFeedback

    vote = {"conversation_id": "c9", "message_id": "a1", "rating": 1, "question": "q", "answer": "a", "page": "/docs/x"}
    assert client.post("/api/v1/chats/feedback", json=vote, headers=user[0]).json() == {"rating": 1}
    assert client.post("/api/v1/chats/feedback", json={**vote, "rating": -1}, headers=user[0]).json() == {"rating": -1}
    with SessionLocal() as db:
        rows = db.query(AnswerFeedback).filter(AnswerFeedback.conversation_id == "c9").all()
        assert [r.rating for r in rows] == [-1]
    assert client.post("/api/v1/chats/feedback", json={**vote, "rating": None}, headers=user[0]).json() == {"rating": None}
    with SessionLocal() as db:
        assert db.query(AnswerFeedback).filter(AnswerFeedback.conversation_id == "c9").count() == 0
    assert client.post("/api/v1/chats/feedback", json={**vote, "rating": 5}, headers=user[0]).status_code == 422
