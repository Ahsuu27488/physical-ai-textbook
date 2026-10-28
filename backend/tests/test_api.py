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


def test_translate_needs_sign_in_then_a_key(client, user):
    body = {"content": "Hello robot", "target_language": "ja"}
    assert client.post("/api/v1/translation/translate", json=body).status_code == 401
    r = client.post("/api/v1/translation/translate", json=body, headers=user[0])
    assert r.status_code == 503, r.text


def test_translate_uses_the_model_once_then_the_cache(client, user, monkeypatch, with_key):
    from types import SimpleNamespace

    from app import llm
    from app.services import translation as svc

    calls = []

    def create(**kw):
        calls.append(kw)
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content="こんにちは ロボット"))])

    monkeypatch.setattr(llm, "get_openai_client", lambda: SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create))))
    body = {"content": "Hello robot, unique text 42", "target_language": "ja"}
    for _ in range(2):
        r = client.post("/api/v1/translation/translate", json=body, headers=user[0])
        assert r.status_code == 200 and r.json()["translated_content"] == "こんにちは ロボット"
    assert len(calls) == 1 and "Japanese" in calls[0]["messages"][0]["content"]
    assert client.post("/api/v1/translation/translate", json={"content": "x", "target_language": "xx"}, headers=user[0]).status_code == 400


def test_supported_languages_lists_rtl_ones():
    from app.services.translation import supported_languages

    langs = {l["code"]: l for l in supported_languages()}
    assert {"ur", "zh", "ja", "ar", "hi"} <= set(langs) and langs["ur"]["rtl"] and not langs["ja"]["rtl"]


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


def test_personalize_rewrites_for_the_profile_levels(client, user, monkeypatch, with_key):
    from types import SimpleNamespace

    from app import llm

    body = {"content": "A PID controller adds three terms. unique 7", "page_title": "PID"}
    assert client.post("/api/v1/personalization/rewrite", json=body).status_code == 401
    calls = []

    def create(**kw):
        calls.append(kw)
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content="**Tuned for you:** simpler."))])

    monkeypatch.setattr(llm, "get_openai_client", lambda: SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create))))
    for _ in range(2):
        r = client.post("/api/v1/personalization/rewrite", json=body, headers=user[0])
        assert r.status_code == 200, r.text
        assert r.json()["personalized_content"].startswith("**Tuned for you:**")
    assert len(calls) == 1  # second time from the cache
    system = calls[0]["messages"][0]["content"]
    assert "software background: beginner" in system and "hardware/electronics background: intermediate" in system


def test_follow_up_questions_search_with_the_previous_question():
    from app.rag.service import search_query

    history = [{"role": "user", "content": "What is a PID controller?"}, {"role": "assistant", "content": "It is..."}]
    assert search_query("How do I tune it?", history) == "What is a PID controller?\nHow do I tune it?"
    long_q = "Explain in detail how the extended Kalman filter linearizes a nonlinear motion model for a robot"
    assert search_query(long_q, history) == long_q
    assert search_query("How do I tune it?", []) == "How do I tune it?"
