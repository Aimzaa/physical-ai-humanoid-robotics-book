# Implementation Plan: Chapter-Level Urdu Translation

**Branch**: `004-urdu-translation` | **Date**: 2025-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/sp.specify` with clarifications

## Summary

Enable authenticated users to translate chapter content from English to Urdu using an LLM-based translation service. The system provides a toggle button at the top of each chapter that triggers dynamic, profile-adapted translation with client-side rendering and per-user caching (24-48 hour TTL).

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/React (frontend) |
**Primary Dependencies**: FastAPI, OpenRouter API, React 18, Docusaurus 3.x |
**Storage**: SQLite (existing auth.db), in-memory/Redis cache for translations |
**Testing**: pytest, Jest, React Testing Library |
**Target Platform**: Web browser (Docusaurus), Linux server (FastAPI) |
**Project Type**: Web application (existing Docusaurus + FastAPI) |
**Performance Goals**: <5s translation toggle, 95% success rate, 100 concurrent requests |
**Constraints**: No page reload, preserve technical terms, maintain readability Grade 8-10 |
**Scale/Scope**: Per-chapter translation, single Urdu locale |

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Requirement | Status | Evidence |
|------|-------------|--------|----------|
| **Clarity & Simplicity** | Translation must be accessible to beginner Urdu readers | PASS | Profile-adapted translation (Beginner/Intermediate/Advanced levels) |
| **Accuracy** | No AI hallucinations in technical content | PASS | LLM prompt includes fact-preservation constraints; code blocks preserved |
| **Technical Excellence** | Code examples must be fully tested | N/A | No new code examples - translation of existing content |
| **Maintainability** | Easy to update and extend | PASS | Modular component structure; clear separation of concerns |
| **Testing** | All functionality must be verifiable | PASS | Clear acceptance scenarios for each user story |

## Project Structure

### Documentation (this feature)

```text
specs/004-urdu-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   ├── openapi.yaml     # Translation API spec
│   └── frontend.yml     # Component contracts
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/src/
├── api/
│   └── translation.py   # NEW: Translation endpoints
├── services/
│   └── translation_service.py  # NEW: LLM translation logic
└── middleware/
    └── auth.py          # EXISTING: Authentication

frontend/src/
├── components/
│   └── TranslationToggle/  # NEW: Urdu toggle button component
├── pages/                 # Existing chapter pages
└── hooks/
    └── useTranslation.js  # NEW: Translation state management
```

**Structure Decision**: Leverages existing Docusaurus + FastAPI backend architecture. New translation module follows established patterns from the personalization feature. Frontend uses theme swizzling for chapter injection.

## Complexity Tracking

> No constitution violations requiring justification.

---

## Technical Context - Detailed

### Frontend Architecture

```mermaid
graph TD
    A[Docusaurus Chapter Page] --> B[MDXContent Wrapper]
    B --> C[TranslationToggle Component]
    C --> D{Authenticated?}
    D -->|No| E[Hide toggle]
    D -->|Yes| F[Show 'Translate to Urdu']
    F --> G[Click handler]
    G --> H[Extract chapter content]
    H --> I[API: POST /api/translate/{chapter_id}]
    I --> J[FastAPI Backend]
    J --> K[Check cache]
    K -->|Cache hit| L[Return cached translation]
    K -->|Cache miss| M[OpenRouter LLM]
    M --> N[Profile-adapted prompt]
    N --> O[Urdu translation]
    O --> P[Update cache]
    P --> Q[Return to frontend]
    Q --> R[Client-side render Urdu]
```

### Backend API Design

```
POST /api/translate/{chapter_id}
  Request:
    - chapter_id: string (path)
    - content: string (body, extracted from chapter)
    - user_id: string (from session cookie)
  Response:
    - translated_content: string (Urdu)
    - cached: boolean
    - ttl_seconds: integer

DELETE /api/translate/{chapter_id}
  Response:
    - success: boolean
    - message: string

GET /api/translate/{chapter_id}/status
  Response:
    - has_translation: boolean
    - cached_at: datetime or null
    - ttl_remaining: integer or null
```

### Translation Prompt Engineering

```
System Prompt Template:
---
You are an expert technical translator specializing in robotics and AI content.
Translate the following English technical chapter to Urdu (Nastaliq script).

Reader Profile:
- Software Level: {software_level}
- Hardware Experience: {hardware_experience}
- Learning Depth: {learning_depth}

Translation Rules:
1. Preserve all technical terms (ROS2, Python, sensors) in transliterated form
2. Add Urdu explanations for technical concepts in parentheses
3. Keep code blocks in original English
4. Maintain all formatting (headers, lists, emphasis)
5. Use Grade 8-10 readability (accessible for learners)
6. Do not add or remove content - translate only

Output Format: Markdown with Urdu text
---
```

### Content Rendering Strategy

1. **Extraction**: Use existing chapter content selector (from PersonalizeButton)
2. **State Management**: React state for current language (English/Urdu)
3. **Rendering**: Replace article content with translated version
4. **Toggle**: Client-side switch between cached versions
5. **Persistence**: localStorage for toggle state between sessions

### Quality Validation Rules

| Validation | Method | Target |
|------------|--------|--------|
| Urdu readability | Flesch-Kincaid Grade 8-10 | 100% of translations |
| Terminology consistency | Glossary check | No unmatched terms |
| Fact preservation | LLM prompt constraints | No hallucinated facts |
| Technical accuracy | Code block preservation | 100% code match |
| Response time | Performance monitoring | <5s p95 |

---

## Phase 0: Research Tasks

### Research 1: Urdu Technical Translation Standards

**Task**: Research best practices for technical Urdu translation
**Output**: Document standards for technical terminology, transliteration rules, and readability targets

### Research 2: LLM Translation Prompt Optimization

**Task**: Research effective prompt engineering for translation tasks
**Output**: Optimized prompt template for profile-adapted Urdu translation

### Research 3: Docusaurus Content Injection Patterns

**Task**: Research existing patterns for injecting components into chapter pages
**Output**: Implementation approach for TranslationToggle component

---

## Phase 1: Design Artifacts

### Required Deliverables

1. **research.md** - Phase 0 research findings
2. **data-model.md** - Translation cache schema, entities
3. **quickstart.md** - Development setup guide
4. **contracts/openapi.yaml** - Translation API specification
5. **contracts/frontend.yml** - React component contracts

---

## Gating Check (Post-Design)

*Evaluate after Phase 1 deliverables are complete*

| Gate | Requirement | Pass/Fail |
|------|-------------|-----------|
| Performance | <5s toggle time achievable | |
| Accuracy | Technical terms preserved | |
| UX | No page reload required | |
| Scalability | 100 concurrent requests | |

---

## Next Steps

1. Execute Phase 0 research tasks
2. Complete research.md
3. Generate Phase 1 design artifacts
4. Proceed to `/sp.tasks` for implementation planning
