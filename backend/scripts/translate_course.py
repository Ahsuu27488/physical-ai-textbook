"""Translate the course into a locale with the chat model (Gemini), once, into Docusaurus i18n folders.

    cd backend && .venv/Scripts/python scripts/translate_course.py --locale ur          # all lessons + UI strings
    cd backend && .venv/Scripts/python scripts/translate_course.py --locale ur --only 03-ros2

Needs GEMINI_API_KEY in backend/.env. Resumable: a manifest keeps a hash of each English source, so unchanged
lessons are skipped on the next run and edited ones are re-translated. Each translated lesson is compiled with
frontend/scripts/check-mdx.mjs; one that still fails after a retry is kept in English (and listed at the end).
"""
from __future__ import annotations

import argparse
import re
import hashlib
import json
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
from pathlib import Path

import httpx
from openai import OpenAI

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from dotenv import load_dotenv  # noqa: E402

load_dotenv(HERE.parent / ".env")

from app.config import get_settings  # noqa: E402

FRONTEND = HERE.parents[1] / "frontend"
DOCS = FRONTEND / "docs"
NAMES = {"ur": "Urdu"}
STALL_SECONDS = 180  # no new tokens for this long → give up on the request and retry
WORKERS = 3  # lessons in flight at once; the free tier's per-minute request limit is the ceiling
# Free-tier daily request quotas are per model (gemini-3.8-flash: 20 a day), so when one model's day is used up the
# job moves on to the next instead of waiting until midnight Pacific.
# Flash models only: the flash-lite models mixed Hindi, Bengali and Gurmukhi letters into Urdu words (38 of 41
# lessons, 2026-10-28). Lite can still be passed with --models for a language it handles well.
MODELS = ["gemini-3.8-flash", "gemini-3.6-flash"]
models = list(MODELS)
models_lock = threading.Lock()


def current_model() -> str:
    with models_lock:
        if not models:
            raise RuntimeError("every model's daily free quota is used up; run again tomorrow (it resumes)")
        return models[0]


def drop_model(model: str) -> None:
    with models_lock:
        if model in models:
            models.remove(model)
            print(f"    {model}: daily quota used up, now {models[0] if models else 'none left'}", flush=True)

LESSON_PROMPT = """You translate one MDX lesson of Kinemata, a course in robotics and Physical AI, into {name}.
Return the complete file, translated, and nothing else (no code fence around it, no notes).
Rules:
- Front matter: translate the values of title, sidebar_label and description; keep every key and any other value.
- Translate all prose, headings, list items, table cells, link text and image alt text into natural, clear {name}.
- MDX components (<LessonMeta>, <Objectives>, <KeyIdea>, <Term>, <Steps>, <Exercise>, <Solution>, <Takeaways>,
  <Quiz>, <Tabs>, <TabItem>, <Details>): keep tags, prop names and structure exactly; translate only human-readable
  string values (e.g. title, label, q, options, explain, prereqs). Keep numbers, booleans, `answer` indexes and value=
  keys of TabItem unchanged. Keep JavaScript syntax valid: escape quotes inside strings exactly as in the source.
- Never translate or change: code blocks, inline code, commands, file paths, URLs, link targets, import lines, math
  ($...$ and $$...$$), mermaid diagram code, HTML/JSX attribute names, heading anchors like {{#id}}.
- Keep established technical terms in English (ROS 2, URDF, LiDAR, PID, SLAM, Kalman filter, ...); you may add a short
  {name} gloss in parentheses the first time.
- Keep blank lines around components and fences exactly as in the source."""

JSON_PROMPT = """Translate the "message" values of this Docusaurus translation JSON into {name}. Keep keys, "description"
values and placeholders like {{count}} unchanged. Brand names (Kinemata, Rivet, GitHub) stay as they are.
Return only the JSON object."""


@lru_cache(maxsize=1)
def client() -> OpenAI:
    # Streamed, with a short read timeout and no hidden SDK retries: a stalled Gemini request used to sit out the
    # SDK's 10-minute default (twice) before anything was printed.
    settings = get_settings()
    timeout = httpx.Timeout(connect=20, read=STALL_SECONDS, write=60, pool=60)
    return OpenAI(api_key=settings.llm_api_key, base_url=settings.llm_base_url, timeout=timeout, max_retries=0)


RETRIES = 40  # per call; the free tier is shared with the owner's other projects, so be patient
ROTATE_AFTER = 6  # failures in a row on one model before the next model gets a turn


def rotate_model(model: str) -> None:
    with models_lock:
        if len(models) > 1 and models[0] == model:
            models.append(models.pop(0))
            print(f"    {model} keeps failing: trying {models[0]} first for now", flush=True)


def chat(system: str, user: str, retries: int = 0) -> str:
    retries = retries or RETRIES
    attempt = 0
    while attempt < retries:
        model = current_model()
        try:
            stream = client().chat.completions.create(
                model=model,
                messages=[{"role": "system", "content": system}, {"role": "user", "content": user}],
                temperature=0.2,
                max_tokens=32000,
                reasoning_effort="low",
                stream=True,
            )
            parts = []
            for chunk in stream:
                if chunk.choices and chunk.choices[0].delta and chunk.choices[0].delta.content:
                    parts.append(chunk.choices[0].delta.content)
            text = "".join(parts).strip()
            if text.startswith("```") and text.endswith("```"):  # a wrapper fence despite the instructions
                text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
            if text:
                return text + "\n"
            raise RuntimeError("empty reply")
        except Exception as exc:
            if getattr(exc, "status_code", None) == 429 and "per day" in str(exc):
                drop_model(model)
                continue
            attempt += 1
            if attempt % ROTATE_AFTER == 0:
                rotate_model(model)
            limited = getattr(exc, "status_code", None) == 429
            wait = min(120, 15 * 2 ** min(attempt, 6)) if limited else min(60, 5 * 2 ** min(attempt, 4))
            metric = re.search(r"Quota exceeded for metric: [^,]*?(\w+), limit: (\d+)", str(exc))
            why = f"{metric.group(1)} {metric.group(2)}" if metric else type(exc).__name__
            print(f"    {model}: retry in {wait}s ({why})", flush=True)
            time.sleep(wait)
    raise RuntimeError("model call kept failing")


def check_mdx(path: Path) -> str:
    """'' when the lesson compiles and has no letters from other scripts and no locale words inside math."""
    for script in ("check-mdx.mjs", "check-translation.mjs"):
        r = subprocess.run(["node", f"scripts/{script}", str(path)], cwd=FRONTEND, capture_output=True, text=True,
                           encoding="utf-8")
        if r.returncode != 0:
            return (r.stdout + r.stderr).strip()[-800:]
    return ""


def translate_lessons(locale: str, only: str | None, workers: int) -> list:
    name = NAMES[locale]
    out_root = FRONTEND / "i18n" / locale / "docusaurus-plugin-content-docs" / "current"
    manifest_path = out_root / ".translation-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    lock = threading.Lock()
    kept_english = []
    todo = []
    for src in sorted(p for p in DOCS.rglob("*") if p.suffix in (".md", ".mdx")):
        rel = src.relative_to(DOCS).as_posix()
        if only and not rel.startswith(only):
            continue
        raw = src.read_text(encoding="utf-8")
        digest = hashlib.sha256(raw.encode()).hexdigest()
        if manifest.get(rel) == digest and (out_root / rel).exists():
            continue
        todo.append((rel, raw, digest))
    print(f"{len(todo)} lessons to translate, {workers} at a time", flush=True)

    def one(item) -> None:
        rel, raw, digest = item
        started = time.time()
        dest = out_root / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        system = LESSON_PROMPT.format(name=name)
        dest.write_text(chat(system, raw), encoding="utf-8")
        error = check_mdx(dest)
        if error:
            print(f"  {rel}: failed the checks, retrying once", flush=True)
            note = (f"The previous translation failed these checks: {error[-300:]}. Write {name} in its own script only "
                    f"(no Hindi, Bengali or Gurmukhi letters) and keep {name} words out of $...$ math.")
            dest.write_text(chat(system, raw + f"\n\n<!-- {note} -->"), encoding="utf-8")
            error = check_mdx(dest)
        if error:
            dest.write_text(raw, encoding="utf-8")  # keep the site building; listed below
            digest = "english:" + digest
        with lock:
            if error:
                kept_english.append(rel)
            manifest[rel] = digest
            manifest_path.write_text(json.dumps(manifest, indent=1, sort_keys=True), encoding="utf-8")
            print(f"  {rel}: {'kept in English' if error else 'ok'} ({time.time() - started:.0f}s, {len(manifest)} done)", flush=True)

    with ThreadPoolExecutor(max_workers=workers) as pool:
        for future in [pool.submit(one, item) for item in todo]:
            try:
                future.result()
            except Exception as exc:  # a lesson whose model calls kept failing: the next run picks it up again
                print(f"  failed: {type(exc).__name__}: {exc}", flush=True)
    return sorted(kept_english)


def translate_json(locale: str) -> None:
    """UI strings: sidebar category labels, navbar, footer and theme strings (from `docusaurus write-translations`)."""
    name = NAMES[locale]
    root = FRONTEND / "i18n" / locale
    done_path = root / ".json-translated.json"  # files already translated (re-run write-translations adds keys)
    done = set(json.loads(done_path.read_text(encoding="utf-8"))) if done_path.exists() else set()
    for path in sorted(root.rglob("*.json")):
        rel = path.relative_to(root).as_posix()
        if path.name.startswith(".") or rel in done:
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        if not data:
            continue
        print(f"  {rel}", flush=True)
        try:
            reply = chat(JSON_PROMPT.format(name=name), json.dumps(data, ensure_ascii=False, indent=1))
        except RuntimeError as exc:  # quota: the next run picks this file up again
            print(f"    skipped: {exc}", flush=True)
            continue
        translated = json.loads(reply[reply.index("{"): reply.rindex("}") + 1])
        for key, value in data.items():  # keep only real keys, fall back to English per entry
            if isinstance(value, dict) and isinstance(translated.get(key), dict) and translated[key].get("message"):
                value["message"] = translated[key]["message"]
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        done.add(rel)
        done_path.write_text(json.dumps(sorted(done), indent=1), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--locale", default="ur", choices=sorted(NAMES))
    ap.add_argument("--only", help="translate only lessons whose path starts with this")
    ap.add_argument("--skip-json", action="store_true")
    ap.add_argument("--workers", type=int, default=WORKERS)
    ap.add_argument("--models", default=",".join(MODELS), help="comma-separated, tried in this order")
    ap.add_argument("--retries", type=int, default=RETRIES, help="tries per model call before a lesson is skipped")
    args = ap.parse_args()
    if not get_settings().llm_api_key:
        print("error: GEMINI_API_KEY is not set in backend/.env", file=sys.stderr)
        return 2
    models[:] = [m.strip() for m in args.models.split(",") if m.strip()]
    globals()["RETRIES"] = max(1, args.retries)
    print(f"Translating lessons to {NAMES[args.locale]} with {', '.join(models)}")
    kept = translate_lessons(args.locale, args.only, max(1, args.workers))
    if not args.skip_json:
        print("Translating UI strings")
        translate_json(args.locale)
    if kept:
        print("Kept in English (did not compile after a retry):", *kept, sep="\n  ")
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
