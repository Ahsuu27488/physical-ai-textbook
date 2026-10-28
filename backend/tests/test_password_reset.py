"""Forgot password: one-time links by email (mail API faked)."""
import re
from datetime import datetime, timedelta, timezone

import pytest

from app import mail
from app.api.v1 import password_reset


@pytest.fixture
def outbox(monkeypatch):
    monkeypatch.setenv("MAIL_API_KEY", "test-key")
    monkeypatch.setenv("MAIL_FROM", "Kinemata <sender@example.com>")
    from app.config import get_settings

    get_settings.cache_clear()
    sent = []
    monkeypatch.setattr(password_reset, "send_email", lambda to, subject, text, html: sent.append({"to": to, "text": text}))
    return sent


def _token(message):
    return re.search(r"reset-password\?token=([\w-]+)", message["text"]).group(1)


def test_off_without_a_mail_key(client):
    r = client.post("/api/v1/auth/forgot-password", json={"email": "a@example.com"})
    assert r.status_code == 503
    assert client.get("/health").json()["email"] is False


def test_same_reply_for_unknown_and_known_email(client, user, outbox):
    _, email, _ = user
    unknown = client.post("/api/v1/auth/forgot-password", json={"email": "nobody@example.com"})
    known = client.post("/api/v1/auth/forgot-password", json={"email": email.upper()})
    assert unknown.status_code == known.status_code == 200
    assert unknown.json() == known.json()
    assert [m["to"] for m in outbox] == [email]
    assert "/reset-password?token=" in outbox[0]["text"]
    assert client.get("/health").json()["email"] is True


def test_link_works_once_and_old_password_stops(client, user, outbox):
    _, email, password = user
    client.post("/api/v1/auth/forgot-password", json={"email": email})
    token = _token(outbox[0])
    r = client.post("/api/v1/auth/reset-password", json={"token": token, "new_password": "brand-new-pass"})
    assert r.status_code == 200, r.text
    assert client.post("/api/v1/auth/token", data={"username": email, "password": password}).status_code == 401
    assert client.post("/api/v1/auth/token", data={"username": email, "password": "brand-new-pass"}).status_code == 200
    again = client.post("/api/v1/auth/reset-password", json={"token": token, "new_password": "another-pass-1"})
    assert again.status_code == 400


def test_only_the_hash_is_stored_and_expired_links_fail(client, user, outbox):
    from app.database.database import SessionLocal
    from app.database.models import PasswordReset

    client.post("/api/v1/auth/forgot-password", json={"email": user[1]})
    token = _token(outbox[0])
    with SessionLocal() as db:
        row = db.query(PasswordReset).order_by(PasswordReset.id.desc()).first()
        assert row.token_hash != token and len(row.token_hash) == 64
        row.expires_at = datetime.now(timezone.utc) - timedelta(minutes=1)
        db.commit()
    r = client.post("/api/v1/auth/reset-password", json={"token": token, "new_password": "brand-new-pass"})
    assert r.status_code == 400
    bogus = client.post("/api/v1/auth/reset-password", json={"token": "x" * 43, "new_password": "brand-new-pass"})
    assert bogus.status_code == 400


def test_at_most_three_emails_an_hour_per_address(client, user, outbox):
    for _ in range(5):
        assert client.post("/api/v1/auth/forgot-password", json={"email": user[1]}).status_code == 200
    assert len(outbox) == 3


def test_short_password_rejected(client, user, outbox):
    client.post("/api/v1/auth/forgot-password", json={"email": user[1]})
    r = client.post("/api/v1/auth/reset-password", json={"token": _token(outbox[0]), "new_password": "short"})
    assert r.status_code == 422


def test_mail_provider_request_shape(monkeypatch):
    monkeypatch.setenv("MAIL_API_KEY", "k")
    monkeypatch.setenv("MAIL_FROM", "Kinemata <sender@example.com>")
    from app.config import get_settings

    get_settings.cache_clear()
    calls = []

    class Reply:
        status_code = 201

    monkeypatch.setattr(mail.httpx, "post", lambda url, json, headers, timeout: calls.append((url, json, headers)) or Reply())
    mail.send_email("to@example.com", "Subject", "text", "<p>html</p>")
    url, body, headers = calls[0]
    assert url == mail.BREVO_URL and headers["api-key"] == "k"
    assert body["sender"] == {"name": "Kinemata", "email": "sender@example.com"} and body["to"] == [{"email": "to@example.com"}]

    monkeypatch.setenv("MAIL_PROVIDER", "resend")
    get_settings.cache_clear()
    mail.send_email("to@example.com", "Subject", "text", "<p>html</p>")
    url, body, headers = calls[1]
    assert url == mail.RESEND_URL and headers["Authorization"] == "Bearer k" and body["from"] == "Kinemata <sender@example.com>"

    Reply.status_code = 401
    with pytest.raises(mail.MailError) as err:
        mail.send_email("to@example.com", "Subject", "text", "<p>html</p>")
    assert "k" not in str(err.value).replace("HTTP", "")
