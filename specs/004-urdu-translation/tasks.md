# Tasks: Chapter-Level Urdu Translation

**Feature**: Chapter-Level Urdu Translation for Logged-in Users
**Branch**: `004-urdu-translation`
**Input**: Design documents from `/specs/004-urdu-translation/`
**Tests**: NOT included (not explicitly requested in feature specification)

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Database schema and module structure initialization

- [x] T001 Create database migration for translation tables in backend/src/db_migrations/001_translation_tables.py
- [x] T002 Add translation cache table schema to backend/src/db.py
- [x] T003 [P] Create translation API router placeholder in backend/src/api/translation.py
- [x] T004 [P] Create translation service placeholder in backend/src/services/translation_service.py
- [x] T005 [P] Create TranslationToggle component folder in frontend/src/components/TranslationToggle/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundational

- [x] T006 Integrate translation router into backend/main.py
- [x] T007 [P] Implement auth middleware wrapper in backend/src/api/translation.py (depends on T003)
- [x] T008 [P] Create TranslationCache model with TTL logic in backend/src/services/translation_service.py (depends on T004)

### Translation Prompt System

- [x] T009 Design system prompt template for Urdu translation in backend/src/services/translation_service.py
- [x] T010 Design user prompt template with profile injection in backend/src/services/translation_service.py

### Frontend Foundational

- [x] T011 Create MDXContent theme swizzle in frontend/src/theme/MDXContent/index.js
- [x] T012 [P] Create useTranslation hook in frontend/src/hooks/useTranslation.js

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Toggle Translation (Priority: P1) 🎯 MVP

**Goal**: Enable logged-in users to translate chapter to Urdu and toggle back to English

**Independent Test**: Login, visit chapter, click "Translate to Urdu", verify Urdu content displays. Click "Show English", verify original English content returns without page reload.

### Implementation for User Story 1

- [x] T013 [P] [US1] Create TranslationToggle component UI in frontend/src/components/TranslationToggle/index.js
- [x] T014 [P] [US1] Add loading spinner state in frontend/src/components/TranslationToggle/index.js
- [x] T015 [P] [US1] Implement chapter content extraction from DOM in frontend/src/components/TranslationToggle/index.js
- [x] T016 [US1] Implement API call to POST /api/translate/{chapter_id} in frontend/src/components/TranslationToggle/index.js
- [x] T017 [US1] Implement toggle back to English in frontend/src/components/TranslationToggle/index.js
- [x] T018 [US1] Implement Urdu content rendering in frontend/src/components/TranslationToggle/index.js
- [x] T019 [US1] Implement POST /api/translate/{chapter_id} endpoint in backend/src/api/translation.py
- [x] T020 [US1] Implement LLM call to OpenRouter in backend/src/services/translation_service.py
- [x] T021 [US1] Return translated content in Urdu markdown in backend/src/api/translation.py
- [x] T022 [US1] Add error handling for API failures in frontend/src/components/TranslationToggle/index.js
- [x] T023 [US1] Add error handling for LLM failures in backend/src/services/translation_service.py

**Checkpoint**: User Story 1 complete - toggle translation works end-to-end

---

## Phase 4: User Story 2 - Profile-Adapted Translation (Priority: P2)

**Goal**: Translation adapts complexity based on user's software/hardware/learning depth profile

**Independent Test**: Set profile to Beginner, request translation, verify simple explanations. Set to Advanced, verify technical terminology preserved.

### Implementation for User Story 2

- [x] T024 [P] [US2] Add user profile retrieval in backend/src/services/translation_service.py
- [x] T025 [P] [US2] Implement Beginner-level prompt variations in backend/src/services/translation_service.py
- [x] T026 [P] [US2] Implement Intermediate-level prompt variations in backend/src/services/translation_service.py
- [x] T027 [P] [US2] Implement Advanced-level prompt variations in backend/src/services/translation_service.py
- [x] T028 [US2] Inject user profile into translation prompt in backend/src/services/translation_service.py
- [x] T029 [US2] Add hardware experience adaptation in backend/src/services/translation_service.py
- [x] T030 [US2] Add learning depth adaptation in backend/src/services/translation_service.py

**Checkpoint**: User Story 2 complete - translations adapt to user profile

---

## Phase 5: User Story 3 - Translation Persistence (Priority: P3)

**Goal**: Remember translation preference per chapter to avoid re-translation

**Independent Test**: Translate chapter, navigate away, return, verify Urdu content displays automatically. Click "Show English", verify preference updates.

### Implementation for User Story 3

- [x] T031 [P] [US3] Implement TranslationCache.save() in backend/src/services/translation_service.py
- [x] T032 [P] [US3] Implement TranslationCache.lookup() in backend/src/services/translation_service.py
- [x] T033 [US3] Implement cache hit response in POST /api/translate/{chapter_id} in backend/src/api/translation.py
- [x] T034 [US3] Add content hash validation for cache in backend/src/services/translation_service.py
- [x] T035 [US3] Implement TTL expiration check in backend/src/services/translation_service.py
- [x] T036 [US3] Add localStorage persistence for language preference in frontend/src/hooks/useTranslation.js
- [x] T037 [US3] Implement auto-load cached translation on chapter visit in frontend/src/hooks/useTranslation.js

**Checkpoint**: User Story 3 complete - translations persist between sessions

---

## Phase 6: User Story 4 - Guest User Handling (Priority: P2)

**Goal**: Hide translation for unauthenticated users, block direct API access

**Independent Test**: Visit chapter while logged out, verify no button appears. Try direct API call, verify 401 response.

### Implementation for User Story 4

- [x] T038 [P] [US4] Add authentication check to hide button in frontend/src/components/TranslationToggle/index.js
- [x] T039 [US4] Add auth required decorator to translation endpoints in backend/src/api/translation.py
- [x] T040 [US4] Return 401 Unauthorized for unauthenticated requests in backend/src/api/translation.py

**Checkpoint**: User Story 4 complete - unauthorized access properly blocked

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, user experience improvements, cleanup

- [x] T041 [P] Add loading indicator with spinner in frontend/src/components/TranslationToggle/styles.module.css
- [x] T042 [P] Add error toast/messages for API failures in frontend/src/components/TranslationToggle/index.js
- [x] T043 Add fallback message when translation fails in frontend/src/components/TranslationToggle/index.js
- [x] T044 [P] Add cache cleanup cron job for expired translations in backend/src/services/translation_service.py
- [x] T045 Add timeout handling for slow LLM responses in backend/src/services/translation_service.py
- [x] T046 [P] Update quickstart.md with testing procedures in specs/004-urdu-translation/quickstart.md
- [x] T047 [P] Verify all file paths match implementation in specs/004-urdu-translation/data-model.md

---

## Dependencies & Execution Order

### Phase Dependencies

| Phase | Depends On | Blocks |
|-------|------------|---------|
| Setup (1) | None | Foundational |
| Foundational (2) | Setup | All User Stories |
| User Stories (3-6) | Foundational | Polish |
| Polish (7) | User Stories | Release |

### User Story Dependencies

| Story | Priority | Can Start After | Dependencies |
|-------|----------|-----------------|---------------|
| US1 | P1 | Foundational | None (independent) |
| US2 | P2 | Foundational | US1 helpful but not required |
| US3 | P3 | Foundational | US1 helpful but not required |
| US4 | P2 | Foundational | None (independent) |

### Within Each User Story

1. Backend foundation → frontend components
2. API endpoints → UI integration
3. Core feature → error handling

---

## Parallel Opportunities

### Within Phase 2 (Foundational)
```bash
Task T007: Auth middleware (backend/src/api/translation.py)
Task T008: TranslationCache model (backend/src/services/translation_service.py)
Task T011: MDXContent swizzle (frontend/src/theme/MDXContent/index.js)
Task T012: useTranslation hook (frontend/src/hooks/useTranslation.js)
```
Can all run in parallel since they modify different files.

### Within User Story 1
```bash
Task T013: TranslationToggle UI (frontend/src/components/TranslationToggle/index.js)
Task T019: POST endpoint (backend/src/api/translation.py)
```
Can be developed independently and integrated.

### Between User Stories
All user stories (US1, US2, US3, US4) can proceed in parallel after Phase 2 is complete, as each has its own component files.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test toggle translation independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add Polish → Final release

---

## File Paths Summary

| Component | Path |
|-----------|------|
| Translation API | backend/src/api/translation.py |
| Translation Service | backend/src/services/translation_service.py |
| TranslationToggle Component | frontend/src/components/TranslationToggle/index.js |
| TranslationToggle Styles | frontend/src/components/TranslationToggle/styles.module.css |
| useTranslation Hook | frontend/src/hooks/useTranslation.js |
| MDXContent Swizzle | frontend/src/theme/MDXContent/index.js |
| Database Schema | backend/src/db.py |

---

## Total Task Count

| Phase | Tasks |
|-------|-------|
| Phase 1: Setup | 5 |
| Phase 2: Foundational | 7 |
| Phase 3: US1 (Toggle) | 11 |
| Phase 4: US2 (Profile-Adapted) | 7 |
| Phase 5: US3 (Persistence) | 7 |
| Phase 6: US4 (Guest Handling) | 3 |
| Phase 7: Polish | 7 |
| **Total** | **47 tasks** |

**MVP Scope**: Phases 1-3 (23 tasks) - Basic toggle translation functionality
