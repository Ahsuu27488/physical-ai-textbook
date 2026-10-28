def test_health_without_secrets(client):
    assert client.get("/health").json() == {
        "status": "healthy", "database": "sqlite (temporary)", "database_reachable": True,
    }


def test_register_login_me_on_fresh_database(client):
    user = {"email": "ada@example.com", "password": "pw-123456", "name": "Ada",
            "software_background": "beginner", "hardware_background": "intermediate"}
    r = client.post("/api/v1/auth/register", json=user)
    assert r.status_code == 200, r.text

    r = client.post("/api/v1/auth/token", data={"username": user["email"], "password": user["password"]})
    assert r.status_code == 200, r.text
    token = r.json()["access_token"]

    r = client.get("/api/v1/auth/me", headers={"Authorization": f"Bearer {token}"})
    assert r.status_code == 200, r.text
    assert r.json()["email"] == user["email"]


def test_chat_without_sign_in_is_401(client):
    r = client.post("/api/chat", json={"message": "What is ROS 2?"})
    assert r.status_code == 401


def test_translate_to_urdu_accepts_form_data(client):
    # The UI posts FormData; this used to be a 422 because content was read from the query string.
    r = client.post("/api/v1/translation/translate-to-urdu", data={"content": "Hello robot"})
    assert r.status_code == 503, r.text


def test_translate_chapter_works_without_token(client):
    # "Optional" auth used to reject anonymous callers with 401.
    r = client.post("/api/v1/translation/translate-chapter", params={"content": "Hello", "target_language": "ur"})
    assert r.status_code == 503, r.text


def test_secret_key_without_env_is_random_not_a_known_string(monkeypatch):
    import importlib

    import app.auth.utils as utils

    monkeypatch.delenv("SECRET_KEY", raising=False)
    monkeypatch.setattr("dotenv.load_dotenv", lambda *a, **k: False)
    first = importlib.reload(utils).SECRET_KEY
    second = importlib.reload(utils).SECRET_KEY
    assert first != second and len(first) >= 40
    assert "fallback" not in first
    monkeypatch.setenv("SECRET_KEY", "test-secret")
    assert importlib.reload(utils).SECRET_KEY == "test-secret"


def test_secret_key_also_read_from_jwt_secret_key(monkeypatch):
    import importlib

    import app.auth.utils as utils

    monkeypatch.delenv("SECRET_KEY", raising=False)
    monkeypatch.setenv("JWT_SECRET_KEY", "from-the-space")
    monkeypatch.setattr("dotenv.load_dotenv", lambda *a, **k: False)
    assert importlib.reload(utils).SECRET_KEY == "from-the-space"
    monkeypatch.setenv("SECRET_KEY", "test-secret")
    assert importlib.reload(utils).SECRET_KEY == "test-secret"
