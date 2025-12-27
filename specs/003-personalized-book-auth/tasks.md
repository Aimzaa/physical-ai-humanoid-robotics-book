# Implementation Tasks: Personalized Docusaurus Book Platform with Auth & RAG

**Feature**: Personalized Docusaurus Book Platform with Auth & RAG
**Branch**: `003-personalized-book-auth`
**Date**: 2025-12-24
**Spec**: [spec.md](spec.md) | [Plan](plan.md) | [Data Model](data-model.md) | [Contracts](contracts/)

## Summary

This document contains actionable implementation tasks for building an authenticated, personalized Docusaurus book platform. Tasks are organized by User Story priority with clear dependencies and parallel execution opportunities.

**Total Tasks**: 36
**MVP Scope**: User Story 1 (Signup/Signin + Profile) - Tasks T001-T015

## Dependencies Graph

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Foundation) ────────┐
    │                        │
    ▼                        ▼
US1: Signup/Signin     US3: Chatbot (can start after T006)
    │                        │
    ▼                        ▼
US2: Personalization   US4: Profile Persistence (shares US1 components)
    │
    ▼
US5: Non-Auth Access (can start after US1 components)
```

## Parallel Execution Examples

- **After T003**: T004, T005 can run in parallel (backend setup, frontend setup)
- **After T006**: T007-T011 can run in parallel (models, schemas, services)
- **After US1 complete**: US2, US3, US4 can proceed independently

---

## Phase 1: Project Setup

**Goal**: Initialize project structure and dependencies

- [ ] T001 Create backend directory structure per plan.md: `backend/src/`, `backend/src/models/`, `backend/src/services/`, `backend/src/api/`, `backend/tests/`
- [ ] T002 Create frontend component structure: `frontend/src/components/AuthButton/`, `frontend/src/components/PersonalizeButton/`, `frontend/src/pages/signup/`, `frontend/src/pages/signin/`, `frontend/src/pages/settings/`
- [ ] T003 [P] Create `.env.example` in backend with all required variables: AUTH_SECRET, JWT_SECRET, DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, OPENROUTER_API_KEY, FRONTEND_URL
- [ ] T004 [P] Create `requirements.txt` in backend with: fastapi, uvicorn, python-dotenv, pydantic, better-auth, better-sqlite3-adapter, qdrant-client, sentence-transformers, openai, pytest, pytest-asyncio
- [ ] T005 [P] Create `package.json` dependencies for frontend: docusaurus, @docusaurus/preset-classic, react, react-dom, clsx, @docusaurus/module-type-aliases

**Phase 1 Acceptance Criteria**:
- Backend directory structure exists with empty `__init__.py` files
- Frontend component directories exist with `index.jsx` files
- `.env.example` contains all required environment variables
- Dependencies listed in requirements.txt and package.json

---

## Phase 2: Foundational Components

**Goal**: Create shared infrastructure blocking all user stories

- [ ] T006 Create database initialization script `backend/src/db/init.py` that:
  - Creates SQLite database at `./data/auth.db`
  - Creates `user` table with: id (TEXT PRIMARY KEY), email (UNIQUE), password_hash, email_verified (DEFAULT 0), created_at, last_login
  - Creates `session` table with: id, user_id (FK), token (UNIQUE), expires_at, ip_address, user_agent, created_at
  - Creates `user_profile` table with: id (FK), software_level (DEFAULT 'Beginner'), hardware_experience (DEFAULT 'None'), learning_depth (DEFAULT 'Both'), personalization_enabled (DEFAULT 1), favorite_chapters (JSON), last_personalized_chapter, created_at, updated_at
- [ ] T007 [P] Create Pydantic schemas `backend/src/schemas/auth.py` with: SignupRequest (email, password, confirm_password, software_level, hardware_experience, learning_depth), SigninRequest (email, password), UserResponse, ProfileResponse, ErrorResponse
- [ ] T008 [P] Create Pydantic schemas `backend/src/schemas/profile.py` with: ProfileUpdateRequest, ProfileData, PersonalizationResponse, ChatRequest, ChatResponse
- [ ] T009 [P] Create Better Auth configuration `backend/src/auth/config.py` with:
  - auth = createAuth() with better-sqlite3 adapter
  - Cookie configuration: httpOnly: true, secure: production, sameSite: "lax", maxAge: 60*60*24*30
  - Email/password sign-in/sign-up enabled
- [ ] T010 [P] Create CORS middleware configuration `backend/src/middleware/cors.py`:
  - Allow origins from FRONTEND_URL env var
  - Allow credentials: true
  - Allow methods: ["*"], headers: ["*"]
- [ ] T011 [P] Create auth middleware `backend/src/middleware/auth.py`:
  - Session validation function
  - Optional user dependency for FastAPI endpoints
  - Protected route decorator

**Phase 2 Acceptance Criteria**:
- Database script runs without errors and creates all tables
- Pydantic schemas validate according to contracts/auth.yaml and contracts/api.yaml
- Better Auth configuration produces valid auth instance
- CORS allows requests from localhost:3000 (dev) and GitHub Pages domain (prod)
- Auth middleware correctly validates/invalidates sessions

---

## Phase 3: User Story 1 - Signup and Signin (MVP)

**Goal**: Allow users to create accounts with profile and sign in

**Independent Test**: User visits site, clicks Sign Up, fills form, completes registration, profile is saved.

- [ ] T012 [US1] Create User model `backend/src/models/user.py`:
  - SQLAlchemy model mapping to `user` table
  - Attributes: id, email, password_hash, email_verified, created_at, last_login
- [ ] T013 [US1] Create UserProfile model `backend/src/models/profile.py`:
  - SQLAlchemy model mapping to `user_profile` table
  - Attributes: id (FK), software_level, hardware_experience, learning_depth, personalization_enabled, favorite_chapters, last_personalized_chapter, created_at, updated_at
- [ ] T014 [US1] Create auth service `backend/src/services/auth_service.py` with:
  - `signup(email, password, software_level, hardware_experience, learning_depth)` -> creates user + profile
  - `signin(email, password)` -> validates and returns user
  - `get_user(user_id)` -> returns user with profile
  - Password hashing using bcrypt
- [ ] T015 [US1] Create API endpoints in `backend/src/api/auth.py`:
  - POST `/auth/signup` - accepts SignupRequest, creates account, returns 201
  - POST `/auth/signin` - accepts SigninRequest, sets HttpOnly cookie, returns 200
  - GET `/auth/me` - returns current user with profile
  - POST `/auth/signout` - clears session, returns 200
- [ ] T016 [US1] Create Signup page component `frontend/src/pages/signup/index.jsx`:
  - Form with email, password, confirm_password fields
  - Background questions: software_level (select), hardware_experience (select), learning_depth (select)
  - POST to `/auth/signup` with credentials: 'include'
  - Handle validation errors (400), email exists (409)
  - Redirect to homepage on success
- [ ] T017 [US1] Create Signin page component `frontend/src/pages/signin/index.jsx`:
  - Form with email, password fields
  - POST to `/auth/signin` with credentials: 'include'
  - Handle invalid credentials (401)
  - Redirect to homepage on success
- [ ] T018 [US1] Create AuthButton component `frontend/src/components/AuthButton/index.jsx`:
  - Fetch `/auth/me` on mount with credentials: 'include'
  - Show "Sign Up" / "Sign In" links if not authenticated
  - Show email + "Sign Out" button if authenticated
  - Handle signout by calling POST `/auth/signout` then reload
- [ ] T019 [US1] Register AuthButton in Docusaurus navbar via `docusaurus.config.js` themeConfig.navbar.items

**User Story 1 Acceptance Criteria**:
- Signup form creates account with all profile fields
- Signin returns HttpOnly session cookie
- GET `/auth/me` returns user email + profile data
- Signout clears session
- Frontend shows appropriate auth state (guest vs logged-in)
- Logged-in users see personalized content button

---

## Phase 4: User Story 2 - Personalized Chapter Content

**Goal**: Adapt chapter content based on user profile when personalization is activated

**Independent Test**: Two users with different profiles (Beginner vs Advanced) view same chapter, content differs appropriately.

- [ ] T020 [US2] Create profile API endpoints in `backend/src/api/profile.py`:
  - GET `/api/profile` - returns full user profile
  - PUT `/api/profile` - updates profile fields (software_level, hardware_experience, learning_depth)
- [ ] T021 [US2] Create Settings page `frontend/src/pages/settings/index.jsx`:
  - Display current profile values
  - Form to update software_level, hardware_experience, learning_depth
  - PUT to `/api/profile` with credentials: 'include'
- [ ] T022 [US2] Create UserContext React hook `frontend/src/hooks/useUserContext.js`:
  - `useUser()` hook returning: { user, profile, loading, updateProfile }
  - Fetches `/api/profile` on mount
  - Caches profile in React context
- [ ] T023 [US2] Create PersonalizeButton component `frontend/src/components/PersonalizeButton/index.jsx`:
  - Props: chapterId
  - Reads toggle state from localStorage `personalize_${chapterId}`
  - Button text: "Personalize Content" (off) or "✓ Personalized" (on)
  - On click: toggles localStorage, calls POST `/api/personalize/${chapterId}` if enabling
  - Reloads page to apply changes when enabling
- [ ] T024 [US2] Create personalization service `backend/src/services/personalization.py`:
  - `get_user_preferences(user_id)` -> returns profile dict
  - `personalize_chapter(chapter_id, user_id)` -> validates personalization enabled
- [ ] T025 [US2] Create personalization API endpoint `backend/src/api/personalization.py`:
  - POST `/api/personalize/{chapter_id}` - enables personalization for chapter
  - Validates user is authenticated
  - Returns success response with user level info
- [ ] T026 [US2] Create content adaptation component `frontend/src/components/ContentAdapter/index.jsx`:
  - Wraps chapter content
  - Reads user profile from context
  - Reads personalization toggle from localStorage
  - Conditionally shows/hides sections based on user level
  - Uses Docusaurus admonitions format: `:::note[beginner]`, `:::note[intermediate]`, `:::note[advanced]`

**User Story 2 Acceptance Criteria**:
- Settings page allows profile updates
- Personalize button appears on chapters for authenticated users
- Toggle persists in localStorage across page refreshes
- Beginner users see beginner-specific content
- Advanced users see advanced-specific content
- Personalization response < 10 seconds

---

## Phase 5: User Story 3 - RAG Chatbot with User Context

**Goal**: Chatbot knows user background and tailors responses

**Independent Test**: Same question asked by Beginner and Advanced users, responses differ in depth.

- [ ] T027 [US3] Create chatbot API endpoint `backend/src/api/chat.py`:
  - POST `/api/chat` - accepts ChatRequest with query, model, conversation_history
  - Reads user profile from session
  - Builds system prompt with user context (software_level, hardware_experience, learning_depth)
  - Calls existing RAG pipeline (Qdrant + OpenRouter) with enhanced prompt
  - Returns ChatResponse with response, sources, tokens_used, user_context_applied
- [ ] T028 [US3] Create UserContext-aware system prompt builder `backend/src/services/chatbot.py`:
  - `build_system_prompt(profile)` function
  - Adds level-appropriate instructions based on software_level
  - Adds depth-appropriate instructions based on learning_depth
  - Adds experience-appropriate instructions based on hardware_experience
- [ ] T029 [US3] Create Chatbot component `frontend/src/components/Chatbot/index.jsx`:
  - Floating chat button in corner
  - Chat dialog with message history
  - Input field for questions
  - Sends POST to `/api/chat` with credentials: 'include'
  - Displays response with sources
  - Shows applied user context indicator

**User Story 3 Acceptance Criteria**:
- Chatbot opens and sends messages
- System prompt includes user profile context
- Beginner questions get beginner-appropriate answers
- Advanced questions get advanced-appropriate answers
- Practical preference gets code examples
- Response time < 3 seconds

---

## Phase 6: User Story 4 - Secure Profile Persistence

**Goal**: Profile persists across sessions and devices

**Independent Test**: User creates profile, logs out, logs back in, profile is restored.

- [ ] T030 [US4] Add session refresh logic `backend/src/services/session_service.py`:
  - On signin, update `last_login` timestamp
  - Validate session not expired on `/auth/me`
  - Return error 401 if session invalid
- [ ] T031 [US4] Add session cleanup worker `backend/src/services/session_cleanup.py`:
  - Cron job or periodic task to delete expired sessions
  - Runs every hour, deletes sessions where expires_at < now()
- [ ] T032 [US4] Add profile caching to UserContext `frontend/src/hooks/useUserContext.js`:
  - Cache profile in localStorage for offline access
  - On page load, check localStorage first, then API
  - Handle API errors gracefully with cached data
- [ ] T033 [US4] Add "Remember Me" functionality `frontend/src/pages/signin/index.jsx`:
  - Checkbox for persistent session
  - If checked, HttpOnly cookie maxAge = 30 days
  - If unchecked, session cookie maxAge = 1 day

**User Story 4 Acceptance Criteria**:
- Profile persists after logout/login
- Sessions expire correctly after maxAge
- Expired sessions rejected with 401
- Profile loads from cache when API unavailable
- Remember Me checkbox extends cookie duration

---

## Phase 7: User Story 5 - Non-Authenticated Access

**Goal**: Visitors can browse without signing up

**Independent Test**: User accesses site without logging in, content is readable but not personalized.

- [ ] T034 [US5] Add guest mode to UserContext `frontend/src/hooks/useUserContext.js`:
  - If no session, return default profile: { software_level: 'Intermediate', hardware_experience: 'Basic', learning_depth: 'Both' }
  - Personalization button hidden or disabled for guests
- [ ] T035 [US5] Update PersonalizeButton `frontend/src/components/PersonalizeButton/index.jsx`:
  - If not authenticated, show "Sign Up to Personalize" link
  - Disable toggle, show login prompt on click
- [ ] T036 [US5] Add guest-friendly content `docs/chapters/`:
  - Ensure default content readable for general audience
  - Add fallback admonitions for guest users
  - Mark personalized sections clearly

**User Story 5 Acceptance Criteria**:
- Visitors see default content without personalization
- Sign Up prompts appear where personalization would be
- No errors when accessing as guest
- Clear distinction between guest and authenticated experience

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Address edge cases and improve robustness

- [ ] T037 Add rate limiting for auth endpoints `backend/src/middleware/rate_limit.py`:
  - Limit signup: 5 per hour per IP
  - Limit signin: 20 per hour per IP
- [ ] T038 Add password strength validation `backend/src/validators/password.py`:
  - Minimum 8 characters
  - At least one uppercase, one lowercase, one number
  - Common password blacklist check
- [ ] T039 Add input sanitization `backend/src/middleware/sanitize.py`:
  - Sanitize all user inputs to prevent XSS
  - Escape HTML in profile fields
- [ ] T040 Add error handling wrapper `backend/src/middleware/error_handler.py`:
  - Catch all exceptions, return 500 with generic message
  - Log full error for debugging
  - Include request ID in response
- [ ] T041 Add logging configuration `backend/src/logging.py`:
  - Configure log level from env (DEBUG/INFO/WARNING)
  - Log all auth events (signup, signin, signout, failures)
  - Include user ID where applicable (masked PII)
- [ ] T042 Add health check endpoint `backend/src/api/health.py`:
  - GET `/health` - returns { status: "healthy", version: "1.0.0" }
  - Check database connectivity
  - Check Qdrant connectivity

---

## Implementation Strategy

### MVP First (User Story 1 only)

Focus on T001-T019 to deliver:
- Signup with profile questions
- Signin with session cookie
- Auth state on frontend
- Basic profile viewing

This delivers core value: authenticated users with stored profiles.

### Incremental Delivery

After MVP:
- US2 (T020-T026): Personalization button and content adaptation
- US3 (T027-T029): Context-aware chatbot
- US4 (T030-T033): Profile persistence and session management
- US5 (T034-T036): Guest access mode

### Testing Order

1. Backend unit tests for models, schemas, services
2. Backend integration tests for API endpoints
3. Frontend component tests for AuthButton, PersonalizeButton
4. E2E tests for complete user flows

---

## File Reference

| Path | Purpose |
|------|---------|
| `backend/src/db/init.py` | Database initialization |
| `backend/src/schemas/auth.py` | Auth request/response schemas |
| `backend/src/schemas/profile.py` | Profile request/response schemas |
| `backend/src/auth/config.py` | Better Auth configuration |
| `backend/src/middleware/cors.py` | CORS middleware |
| `backend/src/middleware/auth.py` | Auth session validation |
| `backend/src/models/user.py` | User SQLAlchemy model |
| `backend/src/models/profile.py` | UserProfile SQLAlchemy model |
| `backend/src/services/auth_service.py` | Auth business logic |
| `backend/src/services/personalization.py` | Personalization logic |
| `backend/src/services/chatbot.py` | Chatbot prompt builder |
| `backend/src/api/auth.py` | Auth endpoints |
| `backend/src/api/profile.py` | Profile endpoints |
| `backend/src/api/personalization.py` | Personalization endpoint |
| `backend/src/api/chat.py` | Chatbot endpoint |
| `frontend/src/pages/signup/index.jsx` | Signup page |
| `frontend/src/pages/signin/index.jsx` | Signin page |
| `frontend/src/pages/settings/index.jsx` | Settings page |
| `frontend/src/components/AuthButton/index.jsx` | Auth nav button |
| `frontend/src/components/PersonalizeButton/index.jsx` | Personalize toggle |
| `frontend/src/components/Chatbot/index.jsx` | Chatbot widget |
| `frontend/src/components/ContentAdapter/index.jsx` | Content filter |
| `frontend/src/hooks/useUserContext.js` | User context hook |
