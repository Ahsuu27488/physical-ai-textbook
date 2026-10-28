from app.rag.chunking import (
    MAX_WORDS,
    OVERLAP_WORDS,
    chunk_mdx,
    doc_module,
    doc_url,
    mdx_to_text,
    parse_front_matter,
    quiz_to_text,
    slugify,
)


# --- urls ------------------------------------------------------------------------------------
def test_doc_url_strips_number_prefixes_and_extension():
    assert doc_url("03-ros2/02-nodes-topics-services.mdx") == "/docs/ros2/nodes-topics-services"
    assert doc_url("10-capstone-and-career/05-careers-and-portfolio.md") == "/docs/capstone-and-career/careers-and-portfolio"


def test_doc_url_index_is_the_folder():
    assert doc_url("03-ros2/index.mdx") == "/docs/ros2"
    assert doc_url("index.mdx") == "/docs/"
    assert doc_url("00-start-here\\index.mdx") == "/docs/start-here"  # windows separators


def test_doc_module():
    assert doc_module("03-ros2/02-nodes.mdx") == "ros2"
    assert doc_module("index.mdx") == "overview"


# --- anchors ---------------------------------------------------------------------------------
def test_slugify_like_docusaurus():
    assert slugify("Poses with a dataclass") == "poses-with-a-dataclass"
    assert slugify("Q&A: what's next?") == "qa-whats-next"
    assert slugify("Arrays -- the robot's data type") == "arrays----the-robots-data-type"
    assert slugify("Step 2: Install ROS_2") == "step-2-install-ros_2"


def test_duplicate_headings_get_numbered_anchors_and_code_headings_are_ignored():
    raw = (
        '---\ntitle: "T"\n---\n\n'
        "## Setup\n\n" + _words(130) + "\n\n## Setup\n\n" + _words(130) + "\n\n"
        "## Using `rclpy`\n\n" + _words(130) + "\n\n```bash\n# not a heading\necho hi\n```\n"
    )
    chunks = chunk_mdx(raw, "01-a/01-b.mdx")
    anchors = [c.anchor for c in chunks]
    assert "setup" in anchors and "setup-1" in anchors
    assert "using-rclpy" in anchors
    assert not any("not-a-heading" in a for a in anchors)
    code_chunk = next(c for c in chunks if c.anchor == "using-rclpy")
    assert "# not a heading" in code_chunk.text  # code is kept


def test_explicit_heading_id_is_honoured():
    chunks = chunk_mdx('---\ntitle: "T"\n---\n\n## Hello world {#custom-id}\n\ntext here\n', "a.mdx")
    assert chunks[0].anchor == "custom-id"
    assert chunks[0].section == "Hello world"


# --- front matter ----------------------------------------------------------------------------
def test_front_matter_parser():
    meta, body = parse_front_matter(
        '---\ntitle: "Nodes, topics and \\"services\\""\nsidebar_label: Nodes\ndescription: "A: b"\nsidebar_position: 2\n---\nBody\n'
    )
    assert meta["title"] == 'Nodes, topics and "services"'
    assert meta["sidebar_label"] == "Nodes"
    assert meta["description"] == "A: b"
    assert body.strip() == "Body"
    assert parse_front_matter("no front matter")[0] == {}


# --- component conversion --------------------------------------------------------------------
def test_term_keyidea_exercise_solution_and_imports():
    src = (
        "import Foo from '@site/src/Foo';\n\n"
        'A <Term def="A grid of numbers.">NumPy array</Term> is fast.\n\n'
        '<KeyIdea title="Know your shape">\nPrint `.shape` first.\n</KeyIdea>\n\n'
        '<Exercise title="Filter a scan" level="Beginner">\nKeep valid readings.\n\n<Solution>\n```python\nprint(1)\n```\n</Solution>\n</Exercise>\n\n'
        "<Custom foo={1}>inner text stays</Custom>\n"
    )
    out = mdx_to_text(src)
    assert "import Foo" not in out
    assert "NumPy array (A grid of numbers.) is fast." in out
    assert "Key idea — Know your shape:" in out
    assert "Print `.shape` first." in out
    assert "Exercise — Filter a scan:" in out
    assert "Solution:" in out
    assert "```python\nprint(1)\n```" in out
    assert "inner text stays" in out and "<Custom" not in out and "</" not in out


def test_python_imports_in_code_fences_are_kept():
    out = mdx_to_text("```python\nimport numpy as np\n```\n")
    assert "import numpy as np" in out


def test_lesson_meta_line():
    out = mdx_to_text('<LessonMeta level="Beginner" time="40 min" prereqs={["Basic Python", "A venv"]} />\n')
    assert out == "Level: Beginner · Time: 40 min · Prerequisites: Basic Python; A venv"


def test_katex_and_tables_survive():
    src = "The rate is $\\dot x = v\\cos\\theta$.\n\n$$\nE = mc^2 < F\n$$\n\n| a | b |\n|---|---|\n| 1 | 2 |\n"
    out = mdx_to_text(src)
    assert "$\\dot x = v\\cos\\theta$" in out
    assert "E = mc^2 < F" in out
    assert "| a | b |" in out


QUIZ = """<Quiz questions={[
  {q: "What does points[:, 0] return?", options: ["The first row", "All values in column 0", "The shape"], answer: 1, explain: "The colon means all rows."},
  {q: "Why is it faster?", options: ["Less accurate", "Compiled code"], answer: 1, explain: "No per-element overhead."}
]} />"""


def test_quiz_becomes_text_with_correct_answers():
    out = mdx_to_text("Before.\n\n" + QUIZ + "\n\nAfter.")
    assert "Quiz:" in out
    assert "1. What does points[:, 0] return? Options: The first row; All values in column 0; The shape." in out
    assert "Correct answer: All values in column 0." in out
    assert "Explanation: The colon means all rows." in out
    assert "2. Why is it faster?" in out and "Correct answer: Compiled code." in out
    assert "Before." in out and "After." in out


def test_broken_quiz_is_dropped_not_fatal():
    out = mdx_to_text("Keep me.\n\n<Quiz questions={[ this is not parseable ]} />\n\nAnd me.")
    assert "Quiz" not in out and "Keep me." in out and "And me." in out
    assert quiz_to_text("<Quiz questions={[ nope ]} />") == ""


def test_admonition_and_links_and_comments():
    src = ":::note[Setup]\nActivate the venv.\n:::\n\nSee [the docs](https://docs.ros.org) and [lesson](./02-x.mdx). {/* hidden */}<!-- gone -->\n"
    out = mdx_to_text(src)
    assert "Note — Setup:" in out and "Activate the venv." in out
    assert "the docs (https://docs.ros.org)" in out
    assert "lesson" in out and "02-x.mdx" not in out
    assert "hidden" not in out and "gone" not in out and ":::" not in out


# --- chunk sizes and metadata ----------------------------------------------------------------
def _words(n, prefix="w"):
    return " ".join(f"{prefix}{i}" for i in range(n))


def test_chunk_metadata_and_prefix():
    raw = '---\ntitle: "Nodes and topics"\ndescription: "How nodes talk."\n---\n\n' + _words(130, "i") + '\n\n## Publishers\n\n' + _words(150) + "\n"
    chunks = chunk_mdx(raw, "03-ros2/02-nodes-topics-services.mdx")
    c = next(c for c in chunks if c.section == "Publishers")
    assert c.page_title == "Nodes and topics"
    assert c.module == "ros2"
    assert c.anchor == "publishers"
    assert c.url == "/docs/ros2/nodes-topics-services#publishers"
    assert c.page_url == "/docs/ros2/nodes-topics-services"
    assert c.text.startswith("Lesson: Nodes and topics — Section: Publishers\n")
    assert c.body.startswith("w0 ")
    assert [x.chunk_index for x in chunks] == list(range(len(chunks)))


def test_tiny_sections_are_merged():
    raw = '---\ntitle: "T"\n---\n\n## A\n\nshort one\n\n## B\n\nshort two\n\n## C\n\n' + _words(200) + "\n"
    chunks = chunk_mdx(raw, "a.mdx")
    assert len(chunks) == 1  # A, B and C all fit together
    assert "short one" in chunks[0].text and "short two" in chunks[0].text
    assert "## B" in chunks[0].text  # merged headings stay visible in the text


def test_big_sections_are_split_with_overlap_and_stay_under_the_limit():
    paragraphs = "\n\n".join(_words(60, f"p{i}x") + "." for i in range(20))  # 1200 words
    raw = '---\ntitle: "T"\n---\n\n## Big\n\n' + paragraphs + "\n"
    chunks = chunk_mdx(raw, "a.mdx")
    assert len(chunks) >= 3
    for c in chunks:
        assert len(c.body.split()) <= MAX_WORDS
        assert c.section == "Big" and c.anchor == "big"
    # overlap: the second chunk starts with the tail of the first
    first_tail = chunks[0].body.split()[-5:]
    assert all(w in chunks[1].body.split()[: OVERLAP_WORDS + 5] for w in first_tail)


def test_huge_code_block_is_split_into_fenced_pieces():
    code = "\n".join(f"x{i} = {i}" for i in range(1500))
    raw = '---\ntitle: "T"\n---\n\n## Code\n\n```python\n' + code + "\n```\n"
    chunks = chunk_mdx(raw, "a.mdx")
    assert len(chunks) >= 2
    for c in chunks:
        assert c.body.count("```") == 2
        assert len(c.body.split()) <= MAX_WORDS


def test_every_chunk_text_is_nonempty_and_has_no_placeholders():
    raw = '---\ntitle: "T"\n---\n\n## One\n\nUse `ros2 run` here.\n\n```bash\nls\n```\n'
    for c in chunk_mdx(raw, "a.mdx"):
        assert "\x00" not in c.text
        assert c.body.strip()
