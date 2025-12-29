---
id: "003"
title: "Urdu Translation Implementation"
stage: "tasks"
date: "2025-12-28"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "urdu-translation"
branch: "004-urdu-translation"
user: "Human user (Hindi/English)"
command: "/sp.implement"
labels: ["feature-implementation", "i18n", "translation", "urdu", "backend", "frontend"]
links:
  spec: "specs/004-urdu-translation/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "backend/src/db_migrations/001_translation_tables.py"
  - "backend/src/api/translation.py"
  - "backend/src/services/translation_service.py"
  - "backend/src/db/init.py"
  - "backend/main.py"
  - "src/components/TranslationToggle/index.js"
  - "src/components/TranslationToggle/styles.module.css"
  - "src/theme/MDXContent/index.js"
  - "src/hooks/useTranslation.js"
tests:
  - "null"
---

## Prompt

Implement the Urdu translation feature based on the 47 tasks in `specs/004-urdu-translation/tasks.md`. The user provided 5 specific tasks:
1. Frontend UI - Add "Translate to Urdu" button at top of chapter pages
2. Backend API - POST /translate/chapter endpoint
3. Translation Prompt - Accurate Urdu with code preservation
4. Client-side Toggle - English ↔ Urdu without page reload
5. Error Handling - Graceful failures

## Response snapshot

Implemented Phase 1-3 (MVP) of the Urdu translation feature:

**Backend Implementation:**
- Created database migration `001_translation_tables.py` with translation_cache, translation_request, and translation_preference tables
- Implemented `translation.py` API router with endpoints:
  - POST /api/translate/{chapter_id} - Translate chapter to Urdu
  - GET /api/translate/{chapter_id}/status - Check cache status
  - DELETE /api/translate/{chapter_id} - Clear cached translation
  - GET/PUT /api/translate/preferences - User preferences
- Created `translation_service.py` with:
  - TranslationCache model with TTL (48 hours default)
  - Profile-adaptive prompt templates (Beginner/Intermediate/Advanced levels)
  - OpenRouter LLM integration (GPT-4o-mini)
  - Caching with content hash validation

**Frontend Implementation:**
- Created `TranslationToggle` component with:
  - Auth check (button hidden for non-authenticated users)
  - "Translate to Urdu" button with loading spinner
  - Toggle back to English functionality
  - Clear cache option
  - Error handling with retry button
  - Markdown rendering for Urdu content (RTL, proper styling)
- Updated `MDXContent/index.js` to inject TranslationToggle on chapter pages
- Created `useTranslation.js` hook with:
  - Translation state management
  - localStorage persistence for language preference
  - API integration helpers
  - Cache status checking

**Database Integration:**
- Modified `backend/src/db/init.py` to call translation migration on startup

## Outcome

- ✅ Impact: Urdu translation feature implemented for logged-in users (MVP - Phases 1-3)
- 🧪 Tests: null
- 📁 Files: 9 files created/modified
- 🔁 Next prompts: Run backend/frontend to test, complete remaining phases (US2-US4, Polish)
- 🧠 Reflection: Feature architecture follows existing personalization pattern (MDXContent swizzle, similar API structure). Profile-adaptive prompts designed for Beginner/Intermediate/Advanced levels.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
