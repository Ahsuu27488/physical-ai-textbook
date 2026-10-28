"""Rivet's sign-in wall, end to end against a local backend: signed out -> wall -> sign up -> question sent.

Usage: python design/rivet_wall.py <out-dir> [base-url]
Needs the dev site started with BACKEND_URL pointing at a local backend (any account created here is local).
"""
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

out = Path(sys.argv[1])
base = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:3123"
out.mkdir(parents=True, exist_ok=True)
LESSON = "/docs/ros2/nodes-topics-services"

with sync_playwright() as p:
    b = p.chromium.launch()
    for scheme in ("dark", "light"):
        page = b.new_page(viewport={"width": 1440, "height": 900}, color_scheme=scheme)
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto(base + LESSON, wait_until="networkidle")
        page.evaluate("window.dispatchEvent(new CustomEvent('pa:open-assistant', {detail: {prompt: 'What is a node?'}}))")
        page.wait_for_timeout(600)
        page.screenshot(path=str(out / f"wall-{scheme}.png"))
        assert page.get_by_text("Sign in to start.").count() == 1
        assert page.get_by_label("Your question").count() == 0  # no composer while signed out
        if scheme == "light":
            print("page errors:", errors[:5])
            break
        page.get_by_role("button", name="Create free account").click()
        page.wait_for_timeout(400)
        page.screenshot(path=str(out / "wall-dialog.png"))
        page.fill("#pa-name", "Wall Test")
        page.fill("input[name=email]", f"wall{int(time.time())}@example.com")
        page.fill("input[name=password]", "local-test-only-1")
        page.locator("form button[type=submit]").click()
        page.wait_for_timeout(1500)
        page.locator("form button[type=submit]").click()  # "Account created. Sign in to continue."
        page.wait_for_timeout(2500)
        page.screenshot(path=str(out / "wall-after-signup.png"))
        log = page.locator("[role=log]").inner_text()
        print("asked after sign-in:", "What is a node?" in log)
        print("server reply shown:", page.locator("[role=alert]").all_inner_texts())
        print("page errors:", errors[:5])
        page.close()
    page = b.new_page(viewport={"width": 375, "height": 812}, color_scheme="dark")
    page.goto(base + LESSON, wait_until="networkidle")
    page.evaluate("window.dispatchEvent(new CustomEvent('pa:open-assistant'))")
    page.wait_for_timeout(600)
    page.screenshot(path=str(out / "wall-phone.png"))
    b.close()
print("saved", out)
