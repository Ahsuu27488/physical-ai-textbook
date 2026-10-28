# Password reset by email (T2: auth)

**Goal:** a learner who forgot their password gets a one-time link by email and sets a new one.

## Mail provider

The Space can only call out over HTTPS (Hugging Face blocks SMTP ports), and the course has no domain of its own
(`*.vercel.app`). Resend's free tier sends only to the account owner's address until a domain is verified, so it
can't reach learners. **Brevo** (free: 300 emails/day) sends over an HTTPS API from a *single verified sender
address*, no domain needed. Code supports both: `MAIL_PROVIDER=brevo|resend`, `MAIL_API_KEY`, `MAIL_FROM`
(`Kinemata <you@gmail.com>`). Without `MAIL_API_KEY` the feature is off: the endpoints answer 503 and the site hides
"Forgot password?" (`/health` reports `email: false`).

## Flow

1. `POST /api/v1/auth/forgot-password {email}` → always the same 200 reply (no account enumeration). If the account
   exists: a random token (32 bytes, URL-safe) is emailed as `SITE_URL/reset-password?token=…`; only its SHA-256 is
   stored in `password_resets` (user_id, token_hash, expires_at = now + 30 min, used_at). At most 3 emails per address
   per hour (in memory).
2. `POST /api/v1/auth/reset-password {token, new_password ≥ 8}` → the token must exist, be unused and unexpired; sets
   the password, marks every open token of that user used. 400 "This link is invalid or has expired" otherwise.
3. Site: "Forgot password?" in the sign-in dialog → email form → "If an account exists, we've sent a link". Page
   `/reset-password` reads the token, asks for the new password twice, then opens sign-in.

## Not in scope

Signing out other sessions after a reset (JWTs carry no password version; adding one needs a column on `users`, and
`create_all` doesn't alter tables). Tokens last 7 days, as today.

## Checks

pytest: generic reply for unknown email; token works once; expired/used/unknown token → 400; per-address limit;
503 without a key; token stored only as a hash. Browser (mocked API): dialog → forgot form → message; reset page →
success → sign-in.

## Rollback

Revert the PR. The `password_resets` table is additive; it can stay or be dropped. Remove `MAIL_*` secrets.

## Owner steps (after merge)

1. Create a free Brevo account, verify a sender address (Senders & IP → Senders), create an API key (SMTP & API →
   API keys).
2. On the Space: secrets `MAIL_API_KEY` (the key), `MAIL_FROM` (`Kinemata <the verified address>`). `MAIL_PROVIDER`
   defaults to `brevo`.
