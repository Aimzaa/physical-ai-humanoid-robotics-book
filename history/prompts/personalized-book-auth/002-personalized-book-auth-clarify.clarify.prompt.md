---
id: "002"
title: "personalized-book-auth-clarify"
stage: "clarify"
date: "2025-12-24"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "personalized-book-auth"
branch: "003-personalized-book-auth"
user: "Claude User"
command: "/sp.clarify"
labels: [clarification, authentication, storage, ux, better-auth]
links:
  spec: "specs/003-personalized-book-auth/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/003-personalized-book-auth/spec.md"
tests: []
---

## Prompt

/sp.clarify

Answer the following clarification questions for the Personalized Docusaurus Book Platform with Auth & RAG feature:

### 1. Authentication Flow
How does Better Auth integrate with Docusaurus static frontend deployed on GitHub Pages? What are the options for token/session handling?

### 2. Personalization Approach
Base content + pre-written conditional sections for each profile level (Beginner/Intermediate/Advanced) OR full LLM regeneration of content based on user profile context? What approach works best?

### 3. Better Auth Integration (answered from context)
[Q already answered: Client-side plugin with CORS headers + API key for authenticated backend calls]

### 4. User Data Storage (answered from context)
[Q already answered: SQLite (dev) / PostgreSQL (prod) with JSON personalization rules]

### 5. Personalization UX (answered from context)
[Q already answered: Toggle button with localStorage persistence, auto-apply on refresh]

## Response snapshot

All 5 clarification questions resolved and specification updated:

**Q1: Auth token storage method**
- Selected: HttpOnly Cookies (Recommended)
- Rationale: Secure approach that prevents XSS token theft

**Q2: Personalization approach**
- Selected: Hybrid - Base content + pre-written conditional sections for each profile level
- Rationale: More predictable, faster, no LLM cost per personalization

**Q3: Better Auth integration**
- Client-side plugin with CORS headers + API key for authenticated backend calls
- Better Auth handles signin/signup in React component
- Backend API validates session via JWT in cookies

**Q4: User data storage**
- SQLite for development, PostgreSQL for production
- Personalization rules stored as JSON metadata mapping profile attributes to content variants per section

**Q5: Personalization UX**
- Toggle button at chapter start persists preference in localStorage
- On refresh, preference is auto-applied
- User can toggle off to return to default content

Specification updates made:
- Added FR-012: CORS headers for cross-origin API communication
- Added FR-013: localStorage persistence for toggle state
- Updated assumptions 2, 4, 8, 9 for database, CORS, and integration details

## Outcome

- ✅ Clarifications: 5/5 questions resolved
- 📁 Files Updated: 1 (spec.md with new requirements and assumptions)
- 🔁 Next: Specification ready for /sp.plan or /sp.tasks

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Next experiment (smallest change to try): N/A
