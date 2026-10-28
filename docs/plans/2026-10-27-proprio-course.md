# Plan: Proprio — rebrand + the Pro course

**Goal.** Rebrand the project as **Proprio** (the open course in Physical AI & humanoid robotics) and replace the
old 13-page textbook with a complete beginner-to-expert course. The course assistant is called **Rivet**.
**Tier:** T2 (replaces all content; touches the live RAG index). **Go-ahead:** owner request, 2026-10-27.
**Rollback:** the old content stays in git history (`fix/broken-config-and-deps`); revert the branch.

*Name check:* "Proprio" is short for proprioception (a body's sense of its own position), which is what a robot
must learn. A medical-imaging company uses the same word, so the owner should check the name before a public launch.

## Course shape

11 modules, 52 lessons, each lesson 25–45 minutes of reading plus a lab. Order goes from zero to expert:
foundations → how robots work → ROS 2 → simulation → perception → motion → learning → humanoids →
foundation models → capstone. Every module ends in something that runs.

Files live in `frontend/docs/<NN-module>/<NN-lesson>.mdx`. Docusaurus strips the number prefixes, so URLs are
`/docs/<module>/<lesson>` (e.g. `/docs/ros2/nodes-topics-services`). Each module folder has an `index.mdx`
(module overview) and a `_category_.json`.

| # | Module folder | Lessons (file names, in order) |
|---|---|---|
| 00 | `00-start-here` | `01-how-this-course-works`, `02-workstation-setup`, `03-map-of-physical-ai` |
| 01 | `01-foundations` | `01-python-for-robotics`, `02-linear-algebra-and-transforms`, `03-probability-for-robots`, `04-calculus-and-optimization`, `05-linux-git-and-tooling` |
| 02 | `02-robot-fundamentals` | `01-anatomy-of-a-robot`, `02-frames-and-rigid-transforms`, `03-forward-and-inverse-kinematics`, `04-dynamics-essentials`, `05-control-and-pid` |
| 03 | `03-ros2` | `01-ros2-architecture-and-dds`, `02-nodes-topics-services`, `03-actions-parameters-launch`, `04-tf2-and-urdf`, `05-packages-debugging-and-bags` |
| 04 | `04-simulation` | `01-why-simulate`, `02-gazebo-worlds-and-robots`, `03-simulated-sensors-and-ros-gz`, `04-isaac-sim-essentials`, `05-sim-to-real-gap` |
| 05 | `05-perception` | `01-cameras-and-image-processing`, `02-depth-point-clouds-and-lidar`, `03-deep-learning-detection-segmentation`, `04-state-estimation-and-kalman-filters`, `05-slam-and-localization` |
| 06 | `06-motion` | `01-path-planning-algorithms`, `02-navigation-with-nav2`, `03-manipulation-with-moveit2`, `04-grasping` |
| 07 | `07-robot-learning` | `01-ml-refresher-for-robotics`, `02-imitation-learning`, `03-reinforcement-learning`, `04-rl-in-simulation-isaac-lab`, `05-datasets-and-scaling` |
| 08 | `08-humanoids` | `01-humanoid-anatomy`, `02-balance-and-zmp`, `03-locomotion`, `04-whole-body-control`, `05-hands-and-dexterity` |
| 09 | `09-foundation-models` | `01-llms-as-robot-planners`, `02-vision-language-grounding`, `03-vision-language-action-models`, `04-voice-to-action-pipeline`, `05-evaluation-and-safety` |
| 10 | `10-capstone-and-career` | `01-capstone-brief`, `02-capstone-build-guide`, `03-deploying-on-hardware`, `04-ethics-safety-and-standards`, `05-careers-and-portfolio` |

Plus `frontend/docs/index.mdx`: the course home (who it is for, the path, how to use labs, quizzes and Rivet).

Reference stack (state these, don't invent others): Ubuntu 24.04, ROS 2 **Jazzy Jalisco** (LTS), Gazebo
**Harmonic** with `ros_gz`, Python 3.12, PyTorch, OpenCV, Nav2, MoveIt 2. For NVIDIA Isaac Sim / Isaac Lab and
fast-moving models (OpenVLA, π0, GR00T, RT-2), describe concepts and say "check the current release" instead of
quoting version numbers or benchmark figures you are not certain of.

## Lesson format (every lesson)

Front matter:
```yaml
---
title: "ROS 2 nodes, topics and services"     # full title, sentence case
sidebar_label: "Nodes, topics, services"       # ≤ 28 characters
description: "One sentence on what the learner can do after this lesson."
---
```
Do **not** add an `# H1`; the title comes from front matter. Then, in this order:

1. `<LessonMeta level="Beginner" time="35 min" prereqs={["Python basics", "Linux terminal"]} />`
   (`level`: Beginner | Intermediate | Advanced | Expert)
2. A short hook: a real robot situation that needs this lesson (2–4 sentences).
3. `<Objectives>` + a markdown bullet list of 3–5 "you will be able to…" items + `</Objectives>`
4. Teaching sections (`##` / `###`): intuition first, then precision. Use `<KeyIdea title="…">…</KeyIdea>` for the
   1–3 ideas that matter most, `<Term def="…">word</Term>` the first time a key term appears, math with `$…$` /
   `$$…$$` (KaTeX), ```` ```mermaid ```` diagrams for flows and architectures, and code blocks with a language and a
   title (```` ```python title="talker.py" ````). Code must be correct and runnable as shown.
5. `## Hands-on lab` with `<Steps>` + a numbered markdown list + `</Steps>`: a small thing the learner builds and
   how they know it worked.
6. `## Exercises`: 2 or 3 of `<Exercise title="…" level="…">task… <Solution>answer…</Solution></Exercise>`
7. `## Check your understanding`: `<Quiz questions={[{q: "…", options: ["…", "…", "…", "…"], answer: 1, explain: "…"}]} />`
   with 4 questions (`answer` is the 0-based index). Use double quotes in strings; write `’` instead of `'`.
8. `<Takeaways>` + 4–6 bullets + `</Takeaways>`
9. `## Go deeper`: 2–4 links to official docs or well-known sources (docs.ros.org, gazebosim.org,
   docs.nvidia.com, pytorch.org, opencv.org, moveit.picknik.ai, navigation.ros.org, arxiv.org for papers you are
   sure exist). Then one line linking to the next lesson with a relative file link, e.g.
   `Next: [Actions, parameters and launch files](./03-actions-parameters-launch.mdx)`.

Also available: Docusaurus admonitions (`:::tip`, `:::note`, `:::info`, `:::warning`, `:::danger`, with an
optional title `:::tip[Pro tip]`), and `<Tabs>` / `<TabItem value="…" label="…">` (no import needed).
Nothing else: no other JSX components, no `import` lines, no raw HTML except `<br />`.

Voice: a great teacher talking to a motivated beginner. Second person, plain words, short paragraphs, define
every term before using it, then build to expert depth. Mention the course assistant at most once per lesson
("Stuck? Ask **Rivet**, the course assistant, bottom-right."). Length: 1,400–2,200 words.

Module `index.mdx`: front matter (title, description, `sidebar_label: "Overview"`), a 2-paragraph intro, "By the
end of this module you can…" list, a lesson table (lesson, what you build, time), and the module project.
`_category_.json`: `{"label": "03 · ROS 2", "position": 4, "link": {"type": "doc", "id": "ros2/index"}}`
(position = module number + 1; the id has no number prefixes).

Validate before finishing: `cd frontend && node scripts/check-mdx.mjs docs/<NN-module>` must print only `ok`.

## The rest of this change

- Brand: name, logo (SVG mark + favicons), metadata, social card, navbar (floating glass pill), footer.
- Lesson components (`src/theme/MDXComponents.js`), KaTeX + Mermaid, lesson progress (mark complete, sidebar ticks).
- Rivet: chat UI fixes (scrolling, scrollbar), streaming answers, page-aware questions, better system prompt.
- Backend: config module, heading-aware chunking with lesson URLs, `text-embedding-3-small`, idempotent upserts,
  `query_points`, streaming endpoint, history, tests.
- Hero: denser, crisper point cloud with rim light and a bottom-up scan-in intro.
- Qdrant: new cluster via the owner's browser; the owner pastes keys (agents never handle secrets); re-index.
