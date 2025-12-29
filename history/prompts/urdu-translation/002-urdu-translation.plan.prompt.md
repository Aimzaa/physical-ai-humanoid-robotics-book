---
id: "002"
title: "Urdu Translation Plan"
stage: "plan"
date: "2025-12-28"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "urdu-translation"
branch: "004-urdu-translation"
user: "Human user (Hindi/English)"
command: "/sp.plan"
labels: ["feature-plan", "i18n", "translation", "urdu", "backend", "frontend"]
links:
  spec: "specs/004-urdu-translation/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "specs/004-urdu-translation/plan.md"
  - "specs/004-urdu-translation/research.md"
  - "specs/004-urdu-translation/data-model.md"
  - "specs/004-urdu-translation/quickstart.md"
  - "specs/004-urdu-translation/contracts/openapi.yaml"
  - "specs/004-urdu-translation/contracts/frontend.yml"
tests:
  - "null"
---

## Prompt

Create implementation plan for:

Architecture Sketch:
- Docusaurus Chapter Page (MDX)
  → Translate Button
  → API Call (FastAPI)
  → OpenRouter LLM (Translation Prompt)
  → Urdu Content Response
  → Client-side render (React state)

Technical Decisions:
- Translation Approach: LLM-based dynamic translation
- Storage: No permanent overwrite of source content
- Caching: Optional short-term per-user cache

Phases:
1. Research: Urdu technical translation standards
2. Foundation: API + auth check
3. Analysis: Chapter parsing and prompt design
4. Synthesis: UI toggle + content rendering

Quality Validation:
- Urdu readability (Grade 8–10)
- Terminology consistency
- No hallucinated technical facts

## Response snapshot

Created comprehensive implementation plan for Chapter-Level Urdu Translation feature:

1. **plan.md** - Full implementation plan with:
   - Technical context (Python/FastAPI + React/Docusaurus)
   - Constitution check (all gates passed)
   - Project structure (backend + frontend)
   - Detailed architecture diagram
   - API design specifications
   - Research tasks for Phase 0
   - Phase 1 deliverables list

2. **research.md** - Research findings covering:
   - Urdu technical translation standards (transliteration + explanations)
   - LLM prompt optimization for profile-adaptive translation
   - Docusaurus content injection via MDXContent swizzling
   - Caching strategy (per-user 24-48 hour TTL)

3. **data-model.md** - Complete data model including:
   - TranslationCache entity with TTL
   - TranslationRequest tracking
   - TranslationPreference user settings
   - SQLite schema with indexes
   - Pydantic API models

4. **contracts/openapi.yaml** - OpenAPI 3.0.3 spec with:
   - POST /api/translate/{chapter_id}
   - GET /api/translate/{chapter_id}/status
   - DELETE /api/translate/{chapter_id}
   - Preference management endpoints

5. **contracts/frontend.yml** - Frontend contracts:
   - TranslationToggle component props/state
   - useTranslation hook interface
   - UrduContentRenderer specifications
   - API service integration
   - Accessibility requirements

6. **quickstart.md** - Development guide:
   - Setup instructions
   - Environment configuration
   - Example API calls
   - Project structure overview
   - Testing procedures
   - Troubleshooting guide

## Outcome

- Impact: New i18n feature enabling Urdu translation for logged-in learners
- Tests: null
- Files: plan.md, research.md, data-model.md, quickstart.md, contracts/openapi.yaml, contracts/frontend.yml
- Next prompts: /sp.tasks
- Reflection note: Plan is complete with detailed research, architecture diagrams, API contracts, and development guide. Ready for task breakdown.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
