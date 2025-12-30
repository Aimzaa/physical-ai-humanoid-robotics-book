---
id: 004
title: UI/UX Duplicate Title Fix
stage: implement
date: 2025-12-30
surface: agent
model: claude-sonnet-4-5-20250929
feature: ui-ux-enhancement
branch: 005-ui-ux-structure
user: User
command: /sp.implement (duplicate title fix)
labels: ["ui","fix","css"]
links:
  spec: specs/005-ui-ux-structure/spec.md
  ticket: null
  adr: null
  pr: https://github.com/Aimzaa/physical-ai-humanoid-robotics-book/pull/new/005-ui-ux-structure
files:
  - src/pages/index.js
  - src/pages/index.module.css
tests:
  - npm run build (verified)
---

## Prompt

"Mera book title top left mein 2 baar aa raha hai" (Urdu/Hindi: "My book title is appearing twice at the top left")

The user reported that the book title appears twice on the homepage - once in the navbar and once as the page title/heading. This creates a visual duplicate that should be fixed.

## Response snapshot

Applied a two-part fix:
1. Modified `src/pages/index.js` to set `title={null}` in the Layout component and added `wrapperClassName="homepage-wrapper"` for CSS targeting
2. Modified `src/pages/index.module.css` to hide `.theme-doc-header` and `.theme-doc-title` on the homepage only using the `.homepage-wrapper` class

Committed as `d5d2498` - "fix: hide duplicate page title on homepage for clean UI"

## Outcome

- ✅ Impact: Homepage now displays title only once (in navbar), eliminating visual duplication
- 🧪 Tests: Build validated with `npm run build`
- 📁 Files: 2 files modified (src/pages/index.js, src/pages/index.module.css)
- 🔁 Next prompts: None required - feature implementation complete
- 🧠 Reflection: The fix required understanding Docusaurus Layout behavior and using CSS class targeting to hide elements on specific pages

## Evaluation notes (flywheel)

- Failure modes observed: None - CSS !important rules provided clean solution
- Graders run and results (PASS/FAIL): npm run build - PASS
- Prompt variant (if applicable): N/A - direct user request
- Next experiment (smallest change to try): Could investigate removing !important if scoping becomes issue
