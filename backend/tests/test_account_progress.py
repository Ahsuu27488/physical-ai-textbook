"""Account endpoints and progress sync. Fresh SQLite database, no network, no secrets."""
import pytest


# --- account ---------------------------------------------------------------------------------
def test_me_returns_profile_fields(client, user):
    headers, email, _ = user
    body = client.get("/api/v1/auth/me", headers=headers).json()
    assert body["email"] == email
    assert body["name"] == "Ada"
    assert body["software_background"] == "beginner"
    assert body["hardware_background"] == "intermediate"
    assert body["created_at"]
    assert "hashed_password" not in body


def test_update_profile_changes_only_the_fields_sent(client, user):
    headers, _, _ = user
    r = client.put("/api/v1/auth/profile", headers=headers, json={"name": "Ada L.", "software_background": "Advanced"})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["name"] == "Ada L."
    assert body["software_background"] == "advanced"  # normalised
    assert body["hardware_background"] == "intermediate"  # untouched

    r = client.put("/api/v1/auth/profile", headers=headers, json={"hardware_background": ""})
    assert r.json()["hardware_background"] is None  # empty clears it
    assert client.get("/api/v1/auth/me", headers=headers).json()["hardware_background"] is None


def test_update_profile_validates_levels(client, user):
    headers, _, _ = user
    r = client.put("/api/v1/auth/profile", headers=headers, json={"software_background": "wizard"})
    assert r.status_code == 422
    r = client.put("/api/v1/auth/profile", headers=headers, params={"hardware_background": "wizard"})
    assert r.status_code == 422
    r = client.put("/api/v1/auth/profile", headers=headers, params={"hardware_background": "advanced"})  # legacy form
    assert r.status_code == 200 and r.json()["hardware_background"] == "advanced"


def test_profile_requires_auth(client):
    assert client.put("/api/v1/auth/profile", json={"name": "x"}).status_code == 401
    assert client.get("/api/v1/auth/me").status_code == 401


def test_change_password(client, user):
    headers, email, password = user
    r = client.post("/api/v1/auth/change-password", headers=headers,
                    json={"current_password": "wrong-password", "new_password": "new-password-1"})
    assert r.status_code == 400
    r = client.post("/api/v1/auth/change-password", headers=headers,
                    json={"current_password": password, "new_password": "short"})
    assert r.status_code == 422
    r = client.post("/api/v1/auth/change-password", headers=headers,
                    json={"current_password": password, "new_password": "new-password-1"})
    assert r.status_code == 200, r.text
    assert client.post("/api/v1/auth/token", data={"username": email, "password": password}).status_code == 401
    assert client.post("/api/v1/auth/token", data={"username": email, "password": "new-password-1"}).status_code == 200


def test_delete_account_removes_user_progress_and_preferences(client, user):
    headers, email, password = user
    client.put("/api/v1/progress/", headers=headers, json={"completed": ["ros2/a"], "quizzes": {"/docs/ros2/a": {"best": 2, "total": 4}}})
    r = client.post("/api/v1/personalization/preference", headers=headers,
                    json={"chapter_id": "ros2/a", "preferences": {"level": "beginner"}})
    assert r.status_code in (200, 201), r.text

    r = client.request("DELETE", "/api/v1/auth/account", headers=headers, json={"password": "nope-nope"})
    assert r.status_code == 400
    assert client.get("/api/v1/auth/me", headers=headers).status_code == 200  # still there

    r = client.request("DELETE", "/api/v1/auth/account", headers=headers, json={"password": password})
    assert r.status_code == 200, r.text

    assert client.post("/api/v1/auth/token", data={"username": email, "password": password}).status_code == 401
    assert client.get("/api/v1/auth/me", headers=headers).status_code == 401  # the old token is dead
    assert client.get("/api/v1/progress/", headers=headers).status_code == 401

    from app.database.database import SessionLocal
    from app.database.models import PersonalizationPreference, QuizScore, UserProgress

    with SessionLocal() as db:
        assert db.query(UserProgress).filter(UserProgress.doc_id == "ros2/a").count() == 0
        assert db.query(QuizScore).filter(QuizScore.page_path == "/docs/ros2/a").count() == 0
        assert db.query(PersonalizationPreference).filter(PersonalizationPreference.chapter_id == "ros2/a").count() == 0

    # the email can be registered again
    assert client.post("/api/v1/auth/register", json={"email": email, "password": password}).status_code == 200


# --- progress --------------------------------------------------------------------------------
def test_progress_requires_a_bearer_token(client):
    assert client.get("/api/v1/progress/").status_code == 401
    assert client.put("/api/v1/progress/", json={}).status_code == 401
    assert client.delete("/api/v1/progress/completed/x").status_code == 401
    assert client.get("/api/v1/progress/", headers={"Authorization": "Bearer garbage"}).status_code == 401


def test_progress_starts_empty(client, user):
    headers, _, _ = user
    assert client.get("/api/v1/progress/", headers=headers).json() == {"completed": [], "quizzes": {}}


def test_put_merges_completed_by_union_and_quiz_best_by_max(client, user):
    headers, _, _ = user
    r = client.put("/api/v1/progress/", headers=headers, json={
        "completed": ["ros2/architecture", "foundations/python"],
        "quizzes": {"/docs/ros2/architecture": {"best": 2, "total": 4}},
    })
    assert r.status_code == 200, r.text
    assert r.json()["completed"] == ["foundations/python", "ros2/architecture"]

    r = client.put("/api/v1/progress/", headers=headers, json={
        "completed": ["ros2/architecture", "motion/nav2"],  # one repeat, one new
        "quizzes": {
            "/docs/ros2/architecture": {"best": 1, "total": 4},  # worse: ignored
            "/docs/motion/nav2": {"best": 3, "total": 4},
        },
    })
    body = r.json()
    assert body["completed"] == ["foundations/python", "motion/nav2", "ros2/architecture"]
    assert body["quizzes"] == {
        "/docs/motion/nav2": {"best": 3, "total": 4},
        "/docs/ros2/architecture": {"best": 2, "total": 4},
    }

    r = client.put("/api/v1/progress/", headers=headers, json={"quizzes": {"/docs/ros2/architecture": {"best": 4, "total": 4}}})
    assert r.json()["quizzes"]["/docs/ros2/architecture"] == {"best": 4, "total": 4}
    assert client.get("/api/v1/progress/", headers=headers).json() == r.json()


def test_put_with_an_empty_body_is_a_read(client, user):
    headers, _, _ = user
    client.put("/api/v1/progress/", headers=headers, json={"completed": ["a/b"]})
    assert client.put("/api/v1/progress/", headers=headers, json={}).json()["completed"] == ["a/b"]


def test_put_rejects_bad_scores(client, user):
    headers, _, _ = user
    r = client.put("/api/v1/progress/", headers=headers, json={"quizzes": {"/docs/x": {"best": 5, "total": 4}}})
    assert r.status_code == 422
    r = client.put("/api/v1/progress/", headers=headers, json={"completed": ["  "]})
    assert r.status_code == 422


def test_delete_completed_uncompletes_one_lesson_including_nested_ids(client, user):
    headers, _, _ = user
    client.put("/api/v1/progress/", headers=headers, json={"completed": ["ros2/architecture", "ros2/nodes"]})
    r = client.delete("/api/v1/progress/completed/ros2/architecture", headers=headers)
    assert r.status_code == 200
    assert r.json()["completed"] == ["ros2/nodes"]
    # idempotent
    assert client.delete("/api/v1/progress/completed/ros2/architecture", headers=headers).json()["completed"] == ["ros2/nodes"]


def test_progress_is_private_per_user(client, user):
    headers, _, _ = user
    client.put("/api/v1/progress/", headers=headers, json={"completed": ["secret/lesson"]})
    other = client.post("/api/v1/auth/register", json={"email": "other-private@example.com", "password": "pw-123456"})
    assert other.status_code == 200
    token = client.post("/api/v1/auth/token", data={"username": other.json()["email"], "password": "pw-123456"}).json()["access_token"]
    assert client.get("/api/v1/progress/", headers={"Authorization": f"Bearer {token}"}).json()["completed"] == []
