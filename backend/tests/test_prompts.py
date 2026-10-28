from app.rag.prompts import (
    MAX_HISTORY_CHARS,
    MAX_HISTORY_TURNS,
    NO_CONTEXT_BLOCK,
    SYSTEM_PROMPT,
    build_messages,
    trim_history,
)
from app.rag.service import Passage, sources_of

PASSAGES = [
    Passage("ROS 2 architecture", "Nodes", "/docs/ros2/architecture#nodes",
            "Lesson: ROS 2 architecture — Section: Nodes\nA node is a process that does one job.", 0.81),
    Passage("Frames", "Frames", "/docs/robot-fundamentals/frames", "A frame is a coordinate system.", 0.7),
]


def test_system_prompt_describes_rivet():
    assert "Rivet" in SYSTEM_PROMPT and "Kinemata" in SYSTEM_PROMPT
    assert "[1]" in SYSTEM_PROMPT
    for mode in ("quiz me", "explain simply", "summarize this lesson"):
        assert mode in SYSTEM_PROMPT


def test_sources_are_numbered_from_one_in_the_context_block():
    msgs = build_messages("What is a node?", PASSAGES)
    system = msgs[0]["content"]
    assert msgs[0]["role"] == "system"
    assert "[1] ROS 2 architecture — Nodes (/docs/ros2/architecture#nodes)" in system
    assert "[2] Frames (/docs/robot-fundamentals/frames)" in system  # title == section is not repeated
    assert "A node is a process that does one job." in system
    assert "Lesson: ROS 2 architecture — Section" not in system  # embedding prefix is stripped
    assert msgs[-1] == {"role": "user", "content": "What is a node?"}
    assert [s["n"] for s in sources_of(PASSAGES)] == [1, 2]
    assert sources_of(PASSAGES)[0].keys() == {"n", "title", "section", "url", "score"}


def test_no_passages_tells_the_model_to_use_general_knowledge():
    system = build_messages("Hi", [])[0]["content"]
    assert NO_CONTEXT_BLOCK in system


def test_current_page_is_included():
    system = build_messages("Summarize this lesson", [], page={"title": "Frames", "path": "/docs/robot-fundamentals/frames"})[0]["content"]
    assert "Frames (/docs/robot-fundamentals/frames)" in system


def test_history_keeps_only_the_last_turns_in_order():
    history = [{"role": "user" if i % 2 == 0 else "assistant", "content": f"msg {i}"} for i in range(30)]
    kept = trim_history(history)
    assert len(kept) == MAX_HISTORY_TURNS == 20
    assert [m["content"] for m in kept] == [f"msg {i}" for i in range(10, 30)]


def test_history_is_trimmed_to_the_character_budget_keeping_the_newest():
    history = [
        {"role": "user", "content": "a" * 6000},
        {"role": "assistant", "content": "b" * 6000},
        {"role": "user", "content": "c" * 6000},
    ]
    kept = trim_history(history)
    assert sum(len(m["content"]) for m in kept) <= MAX_HISTORY_CHARS
    assert kept[-1]["content"] == "c" * 6000  # newest message intact
    assert kept[0]["content"].startswith("…") or kept[0]["role"] == "assistant"


def test_history_ignores_junk_roles_and_empty_messages():
    kept = trim_history([{"role": "system", "content": "x"}, {"role": "user", "content": "  "}, {"role": "user", "content": "ok"}])
    assert kept == [{"role": "user", "content": "ok"}]


def test_messages_order_system_history_question():
    msgs = build_messages("Q?", [], history=[{"role": "user", "content": "hi"}, {"role": "assistant", "content": "hello"}])
    assert [m["role"] for m in msgs] == ["system", "user", "assistant", "user"]
