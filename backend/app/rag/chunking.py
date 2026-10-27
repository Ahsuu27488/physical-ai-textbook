"""Turn the course's .mdx lessons into retrievable chunks.

Pure functions, no network. The output of `chunk_docs()` is what gets embedded and stored in Qdrant.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

# --- sizes (words) ---------------------------------------------------------------------------
TARGET_WORDS = 400   # split big sections into pieces of about this size
MAX_WORDS = 450      # a section up to this size stays in one chunk
MIN_WORDS = 120      # smaller sections are merged with the next one when they fit
OVERLAP_WORDS = 40   # tail of the previous piece repeated at the start of the next one

DOC_ROUTE = "/docs"
DOC_EXTENSIONS = (".mdx", ".md")


@dataclass
class Chunk:
    page_title: str
    section: str
    anchor: str
    url: str          # /docs/ros2/nodes-topics-services#anchor
    page_url: str     # same without the anchor
    module: str
    text: str         # starts with "Lesson: <title> — Section: <heading>\n"
    chunk_index: int  # position within the page
    description: str = ""
    path: str = ""    # source file relative to the docs root (posix)

    @property
    def body(self) -> str:
        return strip_prefix(self.text)


def make_prefix(page_title: str, section: str) -> str:
    return f"Lesson: {page_title} — Section: {section}\n"


def strip_prefix(text: str) -> str:
    if text.startswith("Lesson: ") and "\n" in text:
        first, rest = text.split("\n", 1)
        if " — Section: " in first:
            return rest
    return text


# --- front matter ----------------------------------------------------------------------------
_FM_RE = re.compile(r"\A﻿?---[ \t]*\r?\n(.*?)\r?\n---[ \t]*(?:\r?\n|\Z)", re.S)


def _unquote(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        quote = value[0]
        inner = value[1:-1]
        if quote == '"':
            inner = inner.replace('\\"', '"').replace("\\\\", "\\")
        else:
            inner = inner.replace("''", "'")
        return inner
    return value


def parse_front_matter(raw: str) -> Tuple[Dict[str, str], str]:
    """Return ({key: value}, body). Only flat `key: value` lines are understood (PyYAML is not installed)."""
    match = _FM_RE.match(raw)
    if not match:
        return {}, raw
    meta: Dict[str, str] = {}
    for line in match.group(1).splitlines():
        if not line.strip() or line.lstrip().startswith("#") or line[0] in " \t-":
            continue
        key, sep, value = line.partition(":")
        if sep and key.strip():
            meta[key.strip()] = _unquote(value)
    return meta, raw[match.end():]


# --- urls ------------------------------------------------------------------------------------
_NUM_PREFIX = re.compile(r"^\d+\s*[-_.]\s*")


def _strip_number_prefix(segment: str) -> str:
    return _NUM_PREFIX.sub("", segment)


def doc_url(rel_path) -> str:
    """Docusaurus URL for a file path relative to the docs root.

    03-ros2/02-nodes-topics-services.mdx -> /docs/ros2/nodes-topics-services
    03-ros2/index.mdx                    -> /docs/ros2
    index.mdx                            -> /docs/
    """
    parts = list(Path(str(rel_path).replace("\\", "/")).parts)
    if not parts:
        return DOC_ROUTE + "/"
    name = parts[-1]
    for ext in DOC_EXTENSIONS:
        if name.endswith(ext):
            name = name[: -len(ext)]
            break
    parts[-1] = name
    parts = [_strip_number_prefix(p) for p in parts]
    folder = parts[:-1]
    last = parts[-1]
    if last.lower() in ("index", "readme") or (folder and last == folder[-1]):
        parts = folder
    if not parts:
        return DOC_ROUTE + "/"
    return DOC_ROUTE + "/" + "/".join(parts)


def doc_module(rel_path) -> str:
    parts = Path(str(rel_path).replace("\\", "/")).parts
    if len(parts) < 2:
        return "overview"
    return _strip_number_prefix(parts[0])


# --- headings --------------------------------------------------------------------------------
_EXPLICIT_ID = re.compile(r"\s*\{#([^}\s]+)\}\s*$")


def slugify(text: str) -> str:
    """Docusaurus (github-slugger) heading id: lowercase, drop punctuation except - and _, spaces to -."""
    text = text.strip().lower()
    text = "".join(ch for ch in text if ch.isalnum() or ch in "-_ ")
    return text.replace(" ", "-")


class _Slugger:
    """One per page: repeated headings get -1, -2 ... like Docusaurus."""

    def __init__(self) -> None:
        self.seen: Dict[str, int] = {}

    def slug(self, heading: str, explicit: Optional[str] = None) -> str:
        base = explicit or slugify(heading)
        if explicit:
            self.seen.setdefault(base, 0)
            return base
        count = self.seen.get(base)
        if count is None:
            self.seen[base] = 0
            return base
        while True:
            count += 1
            candidate = f"{base}-{count}"
            if candidate not in self.seen:
                self.seen[base] = count
                self.seen[candidate] = 0
                return candidate


def _plain_inline(text: str) -> str:
    """Heading text without markdown/JSX decoration (what Docusaurus slugs)."""
    text = re.sub(r"!\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"</?[A-Za-z][^>]*>", "", text)
    text = text.replace("`", "").replace("**", "").replace("__", "")
    text = re.sub(r"(?<![\w*])\*([^*]+)\*(?![\w*])", r"\1", text)
    text = re.sub(r"(?<!\w)_([^_]+)_(?!\w)", r"\1", text)
    return re.sub(r"\s+", " ", text).strip()


# --- MDX -> text -----------------------------------------------------------------------------
_FENCE_RE = re.compile(r"^[ \t]*(```+|~~~+)[^\n]*\n.*?^[ \t]*\1[ \t]*$", re.S | re.M)
_INLINE_CODE_RE = re.compile(r"(`+)(?!`)(.+?)(?<!`)\1(?!`)")
_PH = "\x00"


class _Protected:
    """Swap code (and inline code) for placeholders so later regexes cannot touch it."""

    def __init__(self) -> None:
        self.blocks: List[str] = []

    def hide(self, text: str) -> str:
        text = _FENCE_RE.sub(lambda m: self._store(m.group(0), "B"), text)
        return _INLINE_CODE_RE.sub(lambda m: self._store(m.group(0), "I"), text)

    def _store(self, value: str, kind: str) -> str:
        self.blocks.append(value)
        return f"{_PH}{kind}{len(self.blocks) - 1}{_PH}"

    _RESTORE = re.compile(_PH + r"[BI](\d+)" + _PH)

    def restore(self, text: str) -> str:
        # code can in principle contain placeholders only if the source did; one pass is enough
        return self._RESTORE.sub(lambda m: self.blocks[int(m.group(1))], text)

    def words(self, text: str) -> int:
        return len(self.restore(text).split())


_ATTR_RE = re.compile(r"""([A-Za-z_][\w-]*)\s*=\s*(?:"((?:[^"\\]|\\.)*)"|'((?:[^'\\]|\\.)*)'|\{(.*?)\})""", re.S)
_STR_RE = re.compile(r"""(["'`])((?:\\.|(?!\1).)*)\1""", re.S)
_TAG_ATTRS = r"""(?:\s+(?:[^<>{}"']|"[^"]*"|'[^']*'|\{(?:[^{}]|\{[^{}]*\})*\})*)?"""
_ANY_TAG_RE = re.compile(r"</?[A-Z][A-Za-z0-9.]*" + _TAG_ATTRS + r"/?>")
_HTML_TAG_RE = re.compile(
    r"</?(?:div|span|details|summary|p|center|kbd|sup|sub|figure|figcaption|img|video|iframe|u|mark)\b[^>]*>",
    re.I,
)


def _attrs(tag: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for m in _ATTR_RE.finditer(tag):
        value = m.group(2) if m.group(2) is not None else m.group(3) if m.group(3) is not None else m.group(4)
        out[m.group(1)] = value or ""
    return out


def _strings(js: str) -> List[str]:
    return [m.group(2).replace("\\'", "'").replace('\\"', '"') for m in _STR_RE.finditer(js)]


_QUIZ_RE = re.compile(r"<Quiz\b.*?/>", re.S)
_QUIZ_ITEM_RE = re.compile(
    r"""\bq\s*:\s*(["'`])((?:\\.|(?!\1).)*)\1\s*,\s*"""
    r"""options\s*:\s*\[((?:\s*(?:"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|`[^`]*`)\s*,?)*)\]\s*,\s*"""
    r"""answer\s*:\s*(\d+)"""
    r"""(?:\s*,\s*explain\s*:\s*(["'`])((?:\\.|(?!\5).)*)\5)?""",
    re.S,
)


def _unescape(s: str) -> str:
    return s.replace("\\'", "'").replace('\\"', '"').replace("\\n", " ").strip()


def quiz_to_text(tag: str) -> str:
    """Render <Quiz questions={[...]} /> as plain text. Returns '' when the array cannot be parsed."""
    try:
        items = []
        for i, m in enumerate(_QUIZ_ITEM_RE.finditer(tag), start=1):
            question = _unescape(m.group(2))
            options = [_unescape(o) for o in _strings(m.group(3))]
            answer = int(m.group(4))
            correct = options[answer] if 0 <= answer < len(options) else ""
            line = f"{i}. {question}"
            if options:
                line += " Options: " + "; ".join(options) + "."
            if correct:
                line += f" Correct answer: {correct}."
            if m.group(6):
                line += f" Explanation: {_unescape(m.group(6))}"
            items.append(line)
        if not items:
            return ""
        return "Quiz:\n" + "\n".join(items)
    except Exception:  # lenient by design: a broken quiz must not break ingestion
        return ""


def _lesson_meta_to_text(tag: str) -> str:
    attrs = _attrs(tag)
    bits = []
    if attrs.get("level"):
        bits.append(f"Level: {attrs['level']}")
    if attrs.get("time"):
        bits.append(f"Time: {attrs['time']}")
    prereqs = _strings(attrs["prereqs"]) if attrs.get("prereqs") else []
    if prereqs:
        bits.append("Prerequisites: " + "; ".join(p.strip() for p in prereqs))
    return " · ".join(bits)


_TERM_RE = re.compile(r"""<Term\b(?P<attrs>(?:[^>"']|"[^"]*"|'[^']*')*)>(?P<word>.*?)</Term>""", re.S)
_LESSON_META_RE = re.compile(r"<LessonMeta\b" + _TAG_ATTRS + r"/>", re.S)
_ADMONITION_RE = re.compile(r"^:::(\w+)(?:\[([^\]]*)\])?[ \t]*$", re.M)
_IMPORT_RE = re.compile(r"""^[ \t]*import\s+(?:[^'"\n]*?\s+from\s+)?['"][^'"\n]+['"];?[ \t]*$""", re.M)
_COMMENT_RE = re.compile(r"<!--.*?-->|\{/\*.*?\*/\}", re.S)
_IMAGE_RE = re.compile(r"!\[([^\]]*)\]\(([^)\s]*)[^)]*\)")
_LINK_RE = re.compile(r"(?<!\!)\[([^\]]+)\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")


def _term(m: "re.Match[str]") -> str:
    attrs = _attrs("<Term" + m.group("attrs") + ">")
    word = m.group("word").strip()
    definition = attrs.get("def", "").strip()
    return f"{word} ({definition})" if definition else word


def _titled(label: str):
    def repl(m: "re.Match[str]") -> str:
        title = _attrs(m.group(0)).get("title", "").strip()
        return f"\n{label} — {title}:\n" if title else f"\n{label}:\n"

    return repl


def _link(m: "re.Match[str]") -> str:
    text, url = m.group(1), m.group(2)
    return f"{text} ({url})" if url.startswith(("http://", "https://")) else text


def convert_components(text: str) -> str:
    """MDX -> plain text on already code-protected text."""
    text = _COMMENT_RE.sub("", text)
    text = _IMPORT_RE.sub("", text)
    text = _QUIZ_RE.sub(lambda m: "\n" + quiz_to_text(m.group(0)) + "\n", text)
    text = _LESSON_META_RE.sub(lambda m: _lesson_meta_to_text(m.group(0)), text)
    text = _TERM_RE.sub(_term, text)
    text = re.sub(r"<KeyIdea\b" + _TAG_ATTRS + r">", _titled("Key idea"), text)
    text = re.sub(r"<Exercise\b" + _TAG_ATTRS + r">", _titled("Exercise"), text)
    text = re.sub(r"<Solution\b" + _TAG_ATTRS + r">", "\nSolution:\n", text)
    text = re.sub(r"<Objectives\b" + _TAG_ATTRS + r">", "\nObjectives:\n", text)
    text = re.sub(r"<Takeaways\b" + _TAG_ATTRS + r">", "\nTakeaways:\n", text)
    text = re.sub(
        r"<TabItem\b" + _TAG_ATTRS + r">",
        lambda m: f"\n{_attrs(m.group(0)).get('label', 'Option')}:\n",
        text,
    )
    text = _ADMONITION_RE.sub(
        lambda m: f"{m.group(1).capitalize()}{' — ' + m.group(2) if m.group(2) else ''}:", text
    )
    text = re.sub(r"^:::[ \t]*$", "", text, flags=re.M)
    text = _ANY_TAG_RE.sub("", text)  # remaining components: drop the tag, keep the inner text
    text = re.sub(r"<br\s*/?>", "\n", text, flags=re.I)
    text = _HTML_TAG_RE.sub("", text)
    text = _IMAGE_RE.sub(lambda m: f"[Figure: {m.group(1)}]" if m.group(1).strip() else "", text)
    text = _LINK_RE.sub(_link, text)
    text = text.replace("\r\n", "\n")
    text = re.sub(r"[ \t]+\n", "\n", text)
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.strip("\n")


def mdx_to_text(body: str) -> str:
    """Convert an MDX body to plain, meaning-preserving text (code and KaTeX kept)."""
    prot = _Protected()
    return prot.restore(convert_components(prot.hide(body)))


# --- sections and chunks ---------------------------------------------------------------------
_HEADING_RE = re.compile(r"^(#{1,3})[ \t]+(.+?)[ \t]*#*[ \t]*$", re.M)


@dataclass
class _Section:
    heading: str
    anchor: str
    level: int
    text: str  # placeholders still in place


def _split_sections(text: str, page_title: str, slugger: _Slugger, prot: _Protected) -> List[_Section]:
    sections: List[_Section] = []
    pos = 0
    current = _Section(page_title, "", 0, "")
    for m in _HEADING_RE.finditer(text):
        current.text = text[pos:m.start()]
        sections.append(current)
        raw = prot.restore(m.group(2))
        explicit = None
        em = _EXPLICIT_ID.search(raw)
        if em:
            explicit = em.group(1)
            raw = raw[: em.start()]
        heading = _plain_inline(raw) or page_title
        current = _Section(heading, slugger.slug(heading, explicit), len(m.group(1)), "")
        pos = m.end()
    current.text = text[pos:]
    sections.append(current)
    return sections


def _split_big(text: str, prot: _Protected) -> List[str]:
    """Split an oversized section into pieces of ~TARGET_WORDS at block boundaries, with a small overlap."""
    blocks: List[str] = []
    for block in re.split(r"\n\s*\n", text):
        block = block.strip("\n")
        if not block.strip():
            continue
        if prot.words(block) > TARGET_WORDS:
            blocks.extend(_split_block_by_lines(block, prot))
        else:
            blocks.append(block)

    pieces: List[str] = []
    current: List[str] = []
    count = 0
    for block in blocks:
        w = prot.words(block)
        if current and count + w > TARGET_WORDS:
            pieces.append("\n\n".join(current))
            tail = _overlap_tail(current[-1], prot)
            current, count = ([tail], len(tail.split())) if tail else ([], 0)
        current.append(block)
        count += w
    if current:
        pieces.append("\n\n".join(current))
    return pieces


def _overlap_tail(block: str, prot: _Protected) -> str:
    if (_PH + "B") in block or block.lstrip().startswith(("```", "~~~", "|")):
        return ""  # never repeat code or table rows as overlap
    words = block.split()
    if len(words) < 8:
        return ""
    return "… " + " ".join(words[-OVERLAP_WORDS:])


def _split_block_by_lines(block: str, prot: _Protected) -> List[str]:
    """One huge paragraph or code block: cut at line (or, failing that, word) boundaries."""
    restored = prot.restore(block)
    fence = None
    lines = restored.split("\n")
    opener = closer = ""
    if lines and re.match(r"^\s*(```+|~~~+)", lines[0]) and len(lines) >= 2:
        opener = lines[0]
        closer = lines[-1] if re.match(r"^\s*(```+|~~~+)\s*$", lines[-1]) else ""
        lines = lines[1:-1] if closer else lines[1:]
        fence = True
    units: List[str] = []
    for line in lines:
        if len(line.split()) > TARGET_WORDS:
            words = line.split()
            units.extend(" ".join(words[i:i + TARGET_WORDS]) for i in range(0, len(words), TARGET_WORDS))
        else:
            units.append(line)
    out: List[str] = []
    cur: List[str] = []
    count = 0
    for unit in units:
        w = len(unit.split())
        if cur and count + w > TARGET_WORDS:
            out.append("\n".join(cur))
            cur, count = [], 0
        cur.append(unit)
        count += w
    if cur:
        out.append("\n".join(cur))
    if fence:
        out = [f"{opener}\n{piece}\n{closer or '```'}" for piece in out]
    # re-protect so the caller can treat every piece as one opaque block
    return [prot._store(piece, "B") for piece in out] if fence else out


def chunk_mdx(raw: str, rel_path: str) -> List[Chunk]:
    """Chunk one lesson. `rel_path` is relative to the docs root, e.g. '03-ros2/01-ros2-architecture-and-dds.mdx'."""
    meta, body = parse_front_matter(raw)
    rel_path = str(rel_path).replace("\\", "/")
    stem = Path(rel_path).stem
    prot = _Protected()
    text = convert_components(prot.hide(body))

    page_title = _plain_inline(meta.get("title", ""))  # titles are plain text
    if not page_title:
        h1 = re.search(r"^#[ \t]+(.+?)[ \t]*$", text, re.M)
        page_title = _plain_inline(prot.restore(h1.group(1))) if h1 else _strip_number_prefix(stem).replace("-", " ").title()
    description = meta.get("description", "").strip()

    if meta.get("slug", "").startswith("/"):
        page_url = DOC_ROUTE + meta["slug"].rstrip("/") if meta["slug"] != "/" else DOC_ROUTE + "/"
    else:
        page_url = doc_url(rel_path)
    module = doc_module(rel_path)

    sections = _split_sections(text, page_title, _Slugger(), prot)
    if description:
        first = sections[0]
        first.text = f"Summary: {description}\n\n{first.text.lstrip(chr(10))}"

    # drop empty sections, then merge small neighbours
    sections = [s for s in sections if prot.words(s.text) > 0]
    merged: List[_Section] = []
    for sec in sections:
        if merged:
            prev = merged[-1]
            if prot.words(prev.text) < MIN_WORDS and prot.words(prev.text) + prot.words(sec.text) + 4 <= MAX_WORDS:
                prev.text = f"{prev.text.strip(chr(10))}\n\n{'#' * max(sec.level, 2)} {sec.heading}\n{sec.text.strip(chr(10))}"
                continue
        merged.append(_Section(sec.heading, sec.anchor, sec.level, sec.text))

    chunks: List[Chunk] = []
    for sec in merged:
        pieces = [sec.text.strip("\n")] if prot.words(sec.text) <= MAX_WORDS else _split_big(sec.text.strip("\n"), prot)
        url = f"{page_url}#{sec.anchor}" if sec.anchor else page_url
        for piece in pieces:
            piece = prot.restore(piece).strip()
            if not piece:
                continue
            chunks.append(
                Chunk(
                    page_title=page_title,
                    section=sec.heading,
                    anchor=sec.anchor,
                    url=url,
                    page_url=page_url,
                    module=module,
                    text=make_prefix(page_title, sec.heading) + piece,
                    chunk_index=len(chunks),
                    description=description,
                    path=rel_path,
                )
            )
    return chunks


def chunk_file(path: Path, docs_root: Path) -> List[Chunk]:
    rel = Path(path).resolve().relative_to(Path(docs_root).resolve()).as_posix()
    return chunk_mdx(Path(path).read_text(encoding="utf-8"), rel)


def iter_doc_files(docs_root: Path) -> Iterable[Path]:
    root = Path(docs_root)
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.suffix in DOC_EXTENSIONS and not path.name.startswith("_"):
            if any(part.startswith(("_", ".")) for part in path.relative_to(root).parts):
                continue
            yield path


def chunk_docs(docs_root: Path) -> List[Chunk]:
    chunks: List[Chunk] = []
    for path in iter_doc_files(docs_root):
        chunks.extend(chunk_file(path, docs_root))
    return chunks
