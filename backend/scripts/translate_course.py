"""Translate the course into a locale with the chat model (Gemini), once, into Docusaurus i18n folders.

    cd backend && .venv/Scripts/python scripts/translate_course.py --locale ur          # all lessons + UI strings
    cd backend && .venv/Scripts/python scripts/translate_course.py --locale ur --only 03-ros2

Needs GEMINI_API_KEY in backend/.env. Resumable: a manifest keeps a hash of each English source, so unchanged
lessons are skipped on the next run and edited ones are re-translated. Each translated lesson is compiled with
frontend/scripts/check-mdx.mjs; one that still fails after a retry is kept in English (and listed at the end).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from dotenv import load_dotenv  # noqa: E402

load_dotenv(HERE.parent / ".env")

from app import llm  # noqa: E402
from app.config import get_settings  # noqa: E402

FRONTEND = HERE.parents[1] / "frontend"
DOCS = FRONTEND / "docs"
NAMES = {"ur": "Urdu"}

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


def chat(system: str, user: str, retries: int = 6) -> str:
    client = llm.get_openai_client()
    for attempt in range(retries):
        try:
            r = client.chat.completions.create(
                model=get_settings().chat_model,
                messages=[{"role": "system", "content": system}, {"role": "user", "content": user}],
                temperature=0.2,
                max_tokens=32000,
                reasoning_effort="low",
            )
            text = (r.choices[0].message.content or "").strip()
            if text.startswith("```") and text.endswith("```"):  # a wrapper fence despite the instructions
                text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
            if text:
                return text + "\n"
            raise RuntimeError("empty reply")
        except Exception as exc:
            wait = min(90, 10 * 2 ** attempt) if getattr(exc, "status_code", None) == 429 else 5
            print(f"    retry in {wait}s ({type(exc).__name__})", flush=True)
            time.sleep(wait)
    raise RuntimeError("model call kept failing")


def check_mdx(path: Path) -> str:
    r = subprocess.run(["node", "scripts/check-mdx.mjs", str(path)], cwd=FRONTEND, capture_output=True, text=True)
    return "" if r.returncode == 0 else (r.stdout + r.stderr).strip()[-800:]


def translate_lessons(locale: str, only: str | None) -> list:
    name = NAMES[locale]
    out_root = FRONTEND / "i18n" / locale / "docusaurus-plugin-content-docs" / "current"
    manifest_path = out_root / ".translation-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    kept_english = []
    sources = sorted(p for p in DOCS.rglob("*") if p.suffix in (".md", ".mdx"))
    for src in sources:
        rel = src.relative_to(DOCS).as_posix()
        if only and not rel.startswith(only):
            continue
        raw = src.read_text(encoding="utf-8")
        digest = hashlib.sha256(raw.encode()).hexdigest()
        dest = out_root / rel
        if manifest.get(rel) == digest and dest.exists():
            continue
        print(f"  {rel}", flush=True)
        dest.parent.mkdir(parents=True, exist_ok=True)
        system = LESSON_PROMPT.format(name=name)
        dest.write_text(chat(system, raw), encoding="utf-8")
        error = check_mdx(dest)
        if error:
            print("    does not compile, retrying once", flush=True)
            dest.write_text(chat(system, raw + f"\n\n<!-- The previous translation failed to compile: {error[-300:]} -->"), encoding="utf-8")
            error = check_mdx(dest)
        if error:
            dest.write_text(raw, encoding="utf-8")  # keep the site building; listed below
            kept_english.append(rel)
            digest = "english:" + digest
        manifest[rel] = digest
        manifest_path.write_text(json.dumps(manifest, indent=1, sort_keys=True), encoding="utf-8")
    return kept_english


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
        reply = chat(JSON_PROMPT.format(name=name), json.dumps(data, ensure_ascii=False, indent=1))
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
    args = ap.parse_args()
    if not get_settings().llm_api_key:
        print("error: GEMINI_API_KEY is not set in backend/.env", file=sys.stderr)
        return 2
    print(f"Translating lessons to {NAMES[args.locale]} with {get_settings().chat_model}")
    kept = translate_lessons(args.locale, args.only)
    if not args.skip_json:
        print("Translating UI strings")
        translate_json(args.locale)
    if kept:
        print("Kept in English (did not compile after a retry):", *kept, sep="\n  ")
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
