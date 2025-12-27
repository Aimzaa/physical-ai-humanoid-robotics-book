# Implementation Plan: Personalized Docusaurus Book Platform with Auth & RAG

**Branch**: `003-personalized-book-auth` | **Date**: 2025-12-24 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/003-personalized-book-auth/spec.md`

## Summary

Build a Docusaurus-based book deployed on GitHub Pages with user authentication (Signup/Signin) using Better Auth, user background profiling at signup, personalized chapter content based on user profile, and integrated RAG chatbot aware of user context.

Technical approach:
- **Frontend**: Docusaurus with React-based auth components
- **Backend**: FastAPI with Better Auth integration
- **Database**: SQLite (dev) / PostgreSQL (prod) for user profiles
- **Personalization**: Hybrid approach with base content + pre-written conditional sections
- **Auth Tokens**: JWT stored in HttpOnly Cookies
- **RAG**: Qdrant + OpenRouter extended with user context

## Technical Context

**Language/Version**: Python 3.11+ (FastAPI), TypeScript (Docusaurus/React), JavaScript/Node.js 18+
**Primary Dependencies**: Better Auth, FastAPI, Pydantic, Qdrant Client, OpenAI SDK, Docusaurus 3.x
**Storage**: SQLite (dev) / PostgreSQL (prod) via Better Auth database adapter; Qdrant for RAG embeddings
**Testing**: pytest (backend), Jest/Playwright (frontend)
**Target Platform**: GitHub Pages (frontend), cloud backend (FastAPI)
**Project Type**: web (frontend + backend)
**Performance Goals**: Personalization response < 10s, Chatbot response < 3s
**Constraints**: HttpOnly Cookies for auth tokens, CORS for GitHub Pages → backend
**Scale/Scope**: Single book, hundreds of users, thousands of chapters

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|-----------|-------|--------|
| AI-Assisted, Spec-Driven | Implementation follows spec with Better Auth integration | ✅ PASS |
| Clarity and Simplicity | Personalized content adapts to user level (Beginner/Intermediate/Advanced) | ✅ PASS |
| Consistent Structure | Docusaurus provides consistent chapter structure | ✅ PASS |
| Accuracy Through Verification | All auth and RAG code must be tested | ✅ PASS |
| Maintainability | Clear separation: frontend (Docusaurus), backend (FastAPI), auth (Better Auth) | ✅ PASS |
| Technical Excellence | Code snippets tested, auth flow verified | ✅ PASS |

## Project Structure

### Documentation (this feature)

```text
specs/003-personalized-book-auth/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/          # SQLAlchemy/Pydantic models
│   ├── services/        # Auth, personalization, RAG services
│   ├── api/             # FastAPI endpoints
│   └── main.py          # Application entry point
└── tests/
    ├── unit/
    └── integration/

frontend/                 # Docusaurus-based book
├── src/
│   ├── components/      # React components (Auth, Chatbot, Personalization)
│   ├── pages/           # Custom pages (SignUp, SignIn, Settings)
│   └── theme/           # Docusaurus theme customization
├── docusaurus.config.js # Docusaurus configuration
└── sidebars.js          # Sidebar configuration
```

**Structure Decision**: Web application with Docusaurus frontend and FastAPI backend. Auth handled by Better Auth with React components in Docusaurus.

## Phase 0: Research (NEEDS CLARIFICATION items)

Research tasks identified from Technical Context:
1. Better Auth + Docusaurus integration patterns
2. Better Auth database adapter for SQLite/PostgreSQL
3. HttpOnly Cookie configuration for cross-origin auth
4. Docusaurus React component lifecycle for auth state

## Phase 1: Design & Contracts

**Prerequisites**: research.md complete

### Entities (from data-model.md)
- User (authentication + profile)
- UserSession
- ChapterPersonalization

### API Contracts (from contracts/)
- POST /auth/signup
- POST /auth/signin
- GET /auth/me
- POST /auth/signout
- GET /api/profile
- PUT /api/profile
- POST /api/personalize
- POST /api/chat (extended with user_context)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | All requirements fit within single project structure | N/A |

## Next Steps

1. Phase 0: Research - resolve all NEEDS CLARIFICATION items
2. Phase 1: Generate data-model.md, contracts/, quickstart.md
3. Phase 2: Run /sp.tasks to generate implementation tasks
