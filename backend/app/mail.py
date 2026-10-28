"""Sending email through an HTTPS mail API (Brevo or Resend). Off unless MAIL_API_KEY is set."""
import logging
import re
from typing import Tuple

import httpx

from app.config import get_settings

logger = logging.getLogger(__name__)

BREVO_URL = "https://api.brevo.com/v3/smtp/email"
RESEND_URL = "https://api.resend.com/emails"


class MailError(RuntimeError):
    pass


def mail_configured() -> bool:
    settings = get_settings()
    return bool(settings.mail_api_key and settings.mail_from.strip())


def _sender(value: str) -> Tuple[str, str]:
    """'Kinemata <a@b.c>' -> ('Kinemata', 'a@b.c'); 'a@b.c' -> ('Kinemata', 'a@b.c')."""
    m = re.match(r"\s*(.*?)\s*<([^>]+)>\s*$", value)
    return (m.group(1) or "Kinemata", m.group(2).strip()) if m else ("Kinemata", value.strip())


def send_email(to: str, subject: str, text: str, html: str) -> None:
    """Raises MailError on any failure (never with the key or the provider's raw reply in the message)."""
    settings = get_settings()
    if not mail_configured():
        raise MailError("email is not configured")
    provider = settings.mail_provider.strip().lower()
    name, address = _sender(settings.mail_from)
    if provider == "resend":
        url, headers = RESEND_URL, {"Authorization": f"Bearer {settings.mail_api_key}"}
        body = {"from": f"{name} <{address}>", "to": [to], "subject": subject, "text": text, "html": html}
    else:
        url, headers = BREVO_URL, {"api-key": settings.mail_api_key, "accept": "application/json"}
        body = {"sender": {"name": name, "email": address}, "to": [{"email": to}], "subject": subject,
                "textContent": text, "htmlContent": html}
    try:
        r = httpx.post(url, json=body, headers=headers, timeout=20)
    except httpx.HTTPError as exc:
        raise MailError(type(exc).__name__) from None
    if r.status_code >= 300:
        logger.warning("%s refused the email: HTTP %s", provider, r.status_code)
        raise MailError(f"HTTP {r.status_code}")
