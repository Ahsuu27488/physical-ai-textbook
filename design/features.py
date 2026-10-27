"""Screenshots of the course features: lesson components, quiz, profile, search, chat on a lesson.

Usage: python design/features.py <out-dir> [base-url]
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)
LESSON = "/docs/ros2/nodes-topics-services"

with sync_playwright() as p:
    b = p.chromium.launch()
    page = b.new_page(viewport={"width": 1440, "height": 900}, color_scheme="dark")
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(base + LESSON, wait_until="networkidle")
    page.wait_for_timeout(800)
    page.screenshot(path=str(out / "lesson-top.png"))
    for name, sel in [("lesson-keyidea", "[class*='keyIdea']"), ("lesson-steps", "[class*='steps']"),
                      ("lesson-exercise", "[class*='exercise']"), ("lesson-quiz", "[aria-label='Quiz']"),
                      ("lesson-math", ".katex-display"), ("lesson-mermaid", ".docusaurus-mermaid-container")]:
        loc = page.locator(sel).first
        if loc.count():
            loc.scroll_into_view_if_needed()
            page.wait_for_timeout(500)
            page.screenshot(path=str(out / f"{name}.png"))
    quiz = page.locator("[aria-label='Quiz'] label").first
    if quiz.count():
        quiz.click()
        page.wait_for_timeout(300)
        page.locator("[aria-label='Quiz']").first.screenshot(path=str(out / "lesson-quiz-answered.png"))
    done = page.get_by_role("button", name="Mark lesson complete")
    if done.count():
        done.click()
        page.wait_for_timeout(400)
        done_card = page.get_by_role("button", name="Mark as not done")
        done_card.scroll_into_view_if_needed()
        page.screenshot(path=str(out / "lesson-complete.png"))

    page.goto(base + "/profile", wait_until="networkidle")
    page.wait_for_timeout(800)
    page.screenshot(path=str(out / "profile.png"), full_page=True)

    page.goto(base + "/", wait_until="networkidle")
    page.wait_for_timeout(1500)
    page.screenshot(path=str(out / "home-continue.png"))

    page.goto(base + LESSON, wait_until="networkidle")
    search = page.locator("input[type='search'], .navbar__search-input").first
    if search.count():
        search.click()
        search.type("kalman filter", delay=30)
        page.wait_for_timeout(1500)
        page.screenshot(path=str(out / "search.png"))
    page.keyboard.press("Escape")
    page.get_by_role("button", name="Open chat").click()
    page.wait_for_timeout(600)
    page.screenshot(path=str(out / "chat-on-lesson.png"))
    print("page errors:", errors[:5])
    b.close()
print("saved", out)
