# UI inventory: component-by-component pass (2026-10-28)

Every user-facing component on the Kinemata site, the states each needs, and what this pass did.
Screenshots: `python design/shoot.py design/before|after <url>` then `python design/compare.py`
(local output in `design/before`, `design/after`, `design/compare`; not committed). Checked in Chromium only.

Legend: [x] upgraded and screenshotted · [~] checked, small fixes only · [ ] not done (reason given)

## 1. Rivet chat (`src/components/ChatWidget/`) — Operate
- [x] Launcher: Rivet mark, label, Ctrl/⌘ + / hint (desktop), hover lift, press, focus ring, one-time ping (off in reduced motion), hides while open and under the mobile menu
- [x] Open / close: grows from the launcher with a sheet curve (300 ms), faster close (170 ms), fade only in reduced motion; focus returns to the launcher (was broken)
- [x] Header: avatar mark (eyes scan while answering), "Reading along · page" / "Answering…", new chat, larger, close
- [x] Empty state: greeting with serif accent, what Rivet does, reading-along card, "Try asking" rows, select-to-ask tip; fits the panel at 720 px tall
- [x] User message: tinted bubble instead of solid lime
- [x] Assistant message: "Rivet" label row, Markdown (paragraphs, lists with muted markers, headings, blockquote card, inline code, accent-underlined links)
- [x] Code blocks: language bar + Copy/Copied, own horizontal scroll
- [x] Tables (were raw pipes: no GFM plugin before), KaTeX inline + display, task lists
- [x] Citation chips light up their source row; source rows show title + section with a hover arrow
- [x] Thinking shimmer ("Searching the lessons…"), caret inline at the end of the last paragraph, Copy + Regenerate / "Answer again" after Stop
- [x] Error card with icon and Retry (server off 503, rate limit 429, offline, network)
- [x] "Latest" pill when scrolled up
- [x] Composer: glass, auto-grow (native + JS fallback), Enter / Shift+Enter, send (disabled state), Stop with a turning ring, length counter near 4,000, key hints while typing
- [x] Larger mode (≥900 px), content column capped at 640 px
- [x] Phone sheet: full screen, safe areas, follows `visualViewport` so the composer stays above the keyboard, page scroll locked
- [x] "Ask Rivet" selection bubble: dark pill with mark and pointer, flips below the selection near the top, works on touch
- [x] Scrollbars (thin, stable gutter)
- [x] Markdown renderer lazy-loaded (main.js −112 KB gz)

## 2. Navbar — Operate
- [x] Glass pill lifts after scrolling (CSS scroll timeline), brand, links, active link, toggle only below 997 px
- [x] Search box: pill input, kbd hint, focus ring, icon-only on phones (placeholder no longer clipped)
- [x] Search results: quiet active row with an accent edge (was solid lime, matches unreadable), match highlight, footer, no-results
- [x] Tools menu: popover, native language names, arrow keys in the radiogroup, "Translate to …", Personalize section, sign-in path when signed out, loading spinners
- [x] Tools result dialog: icon + page name, partial-translation note, Urdu RTL, Copy/Copied, error with Try again
- [x] Auth dialog: logo, segmented tabs, labelled fields, eye toggle, Caps Lock hint, live 8-character rule, email check on blur, error/notice notes, shake, spinner, switch link; bottom sheet on phones; focus trap
- [x] Account menu: who (name, email), Continue, Profile & progress, Sign out, arrow keys; signed-in block in the mobile menu
- [~] Theme toggle, GitHub icon: round hover states
- [x] Mobile menu sheet: width, backdrop blur, 44 px rows, back button, Tools and auth sections
- [~] Docusaurus dropdowns (`.dropdown__menu`): styled to the popover look; none are used in the navbar today

## 3. Docs / lessons — Read
- [x] Sidebar: active bar, hover, nested rail, module progress line (new `DocSidebarItem/Category`), ringed completion ticks, collapse button
- [x] TOC: single rail with active marker, "On this page" label; mobile collapsible card
- [x] Breadcrumbs, pagination (prev/next cards with arrows)
- [x] Admonitions: tinted by kind, sentence-case titles (new `theme/Admonition`)
- [x] Code blocks: title bar with file icon, copy/wrap buttons, highlighted line accent edge, line numbers
- [x] Tables (frame, header, row hover), Tabs (segmented), Details, Mermaid (dot grid), KaTeX (long inline math scrolls on narrow screens), images
- [x] Lesson components: LessonMeta (stepped level meter), Objectives (numbered), KeyIdea (glow), Term (edge-aware, arrow, keyboard), Steps, Exercise/Solution (Show/Hide + chevron), Quiz (letter keys, states, score bar, verdict), Takeaways (ticks)
- [x] Lesson-complete card: drawn check, course %, Next lesson
- [~] Edit-this-page link, skip link, back-to-top (moved above Rivet's launcher)

## 4. Landing, 404, footer, shared
- [~] Landing (`src/pages/index.js`): rebuilt by the previous session; this pass fixed the closing band (flush on the footer; title invisible in light theme), restyled the example chat to match Rivet, removed dead CSS
- [x] 404: radar scope, search hint, module links
- [x] Footer: link underline on hover, SVG external icon with screen-reader text
- [x] Shared dialog (`.pa-overlay` / `.pa-dialog`): bottom sheet on phones, icon heading, `useDialog` behaviour
- [x] Buttons: primary, secondary, ghost, danger; sm/md/lg; hover, focus, active, disabled, loading spinner (`aria-busy`)
- [x] Inputs, selects (custom chevron), fields, segmented control, popover menu items, notes, kbd (`custom.css`)
- [ ] Toasts: not added; every action reports inline (Copied, Saved, errors next to the control)

## 5. Other surfaces found in the tree
- [x] Profile page: stats, continue card, module progress, shared inputs, danger buttons, inline confirm for Reset progress (was `window.confirm`)
- [x] Landing "Continue where you left off" CTA (unchanged component, checked)

## Not covered by this pass
- Real phones, Safari and Firefox (Chromium only).
- Lighthouse was not re-run; bundle numbers are in CHANGELOG (main.js 285 → 173 KB gz, CSS 28.5 → 36.8 KB gz).
