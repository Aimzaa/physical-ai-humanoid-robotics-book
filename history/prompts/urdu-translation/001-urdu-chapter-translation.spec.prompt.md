---
id: "001"
title: "Chapter-Level Urdu Translation"
stage: "spec"
date: "2025-12-28"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "urdu-translation"
branch: "004-urdu-translation"
user: "Human user (Hindi/English)"
command: "/sp.specify"
labels: ["feature-spec", "i18n", "translation", "urdu"]
links:
  spec: "specs/004-urdu-translation/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "specs/004-urdu-translation/spec.md"
  - "specs/004-urdu-translation/checklists/requirements.md"
tests:
  - "null"
---

## Prompt

Feature: Chapter-Level Urdu Translation for Logged-in Users

Goal:
Allow authenticated users to translate chapter content from English to Urdu
by pressing a button at the start of each chapter.

User Story:
As a logged-in learner,
I want to translate the current chapter into Urdu,
so I can better understand complex technical concepts.

Scope:
- Show "Translate to Urdu" button at the top of each chapter
- Feature available only for authenticated users
- Translation applies only to the current chapter content
- Original English content must remain recoverable
- Translation respects user profile (software/hardware level)

Non-Goals:
- No full-site language switch
- No offline translation
- No manual user edits to translated text

Inputs:
- Chapter markdown/HTML content
- User profile (learning depth, background)

Outputs:
- Urdu-translated chapter content rendered in-place

Success Criteria:
- User can toggle English ↔ Urdu per chapter
- Translation is context-aware and readable
- No page reload required

## Response snapshot

Created feature specification for Chapter-Level Urdu Translation feature:

1. Created branch `004-urdu-translation` from analysis of existing branches (highest was `003-personalized-book-auth`)

2. Created spec at `specs/004-urdu-translation/spec.md` with:
   - 4 prioritized user stories (P1-P3) covering: toggle translation, profile-adapted translation, persistence, guest handling
   - 12 functional requirements (FR-001 to FR-012)
   - Key entities: TranslationCache, UserProfile
   - 6 measurable success criteria
   - Assumptions and dependencies documented
   - Out of scope clearly defined

3. Created quality checklist at `specs/004-urdu-translation/checklists/requirements.md`

4. All checklist items passed - specification is complete and ready for planning phase

## Outcome

- Impact: New i18n feature enabling Urdu translation for logged-in learners
- Tests: null
- Files: specs/004-urdu-translation/spec.md, checklists/requirements.md
- Next prompts: /sp.plan or /sp.clarify
- Reflection note: Specification is complete with no clarification markers needed. All requirements are testable and technology-agnostic.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
