# Research: Better Auth Integration with Docusaurus

**Feature**: Personalized Docusaurus Book Platform with Auth & RAG
**Date**: 2025-12-24
**Status**: Complete

## Better Auth + Docusaurus Integration

### Decision: Client-side React Plugin with CORS

**Chosen Approach**: Client-side Better Auth plugin with CORS headers for cross-origin API communication between GitHub Pages frontend and backend.

**Rationale**:
- Docusaurus supports React-based plugins and custom components
- Better Auth provides client-side SDK for React applications
- HttpOnly Cookies for JWT storage requires CORS configuration
- This pattern is well-documented in Better Auth docs

**Alternatives Considered**:
1. Server-side auth proxy - Rejected: Docusaurus is static, no server-side rendering
2. LocalStorage tokens - Rejected: XSS vulnerability risk
3. JWT in headers - Rejected: CORS preflight issues with HttpOnly requirement

### Better Auth Database Adapters

**Chosen**: SQLite for development, PostgreSQL for production

**Rationale**:
- SQLite requires no external service for local development
- Better Auth provides official adapters for both
- PostgreSQL recommended for production with better concurrency

**Adapter Selection**:
```typescript
// For SQLite (dev)
import { betterSqlite3 } from "better-sqlite3-adapter"

// For PostgreSQL (prod)
import { PrismaClient } from "@prisma/client"
```

### HttpOnly Cookie Configuration

**Configuration**:
```typescript
// Better Auth config
export const auth = createAuth({
  database: adapter,
  advanced: {
    cookiePrefix: "book-auth/",
    cookies: {
      sessionToken: {
        httpOnly: true,
        secure: process.env.NODE_ENV === "production",
        sameSite: "lax",
        maxAge: 60 * 60 * 24 * 30, // 30 days
      },
    },
  },
})
```

**CORS Configuration** (FastAPI):
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://username.github.io"],  # GitHub Pages domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Personalization Architecture

### Decision: Hybrid Approach with Pre-written Sections

**Chosen**: Base content + pre-written conditional sections for each profile level

**Rationale**:
- Predictable rendering (no LLM cost/timing per request)
- Content validated at write-time, not generation-time
- Faster response (<10s requirement achievable)
- Easier translation and localization

**Structure**:
```markdown
# Chapter: Robot Kinematics

<!-- Default content for all readers -->
## Introduction
Basic kinematics concepts...

<!-- Conditional sections -->
:::note[beginner]
## What is a Kinematic Chain?
A kinematic chain is like a series of connected arms...
:::

:::note[intermediate]
## Forward Kinematics Equations
The DH (Denavit-Hartenberg) parameters define...
:::

:::note[advanced]
## Jacobian Analysis
The Jacobian matrix relates joint velocities...
:::
```

### Personalization Toggle UX

**Flow**:
1. User visits chapter → sees default content
2. "Personalize" button appears (if logged in)
3. Click → content renders with user's profile level
4. Preference saved in localStorage
5. Auto-applied on page refresh

## RAG Chatbot with User Context

### Decision: Extend existing /chat endpoint

**Changes to backend/main.py**:
```python
class ChatRequest(BaseModel):
    query: str
    user_context: Optional[str] = None  # User profile for personalization
    model: str = "openai/gpt-3.5-turbo"
```

**System prompt adaptation**:
```python
def build_system_prompt(user_profile: UserProfile) -> str:
    base = "You are an assistant for the Physical AI & Humanoid Robotics Book."
    if user_profile.software_level == "Beginner":
        base += " Use simple analogies and foundational explanations."
    elif user_profile.software_level == "Advanced":
        base += " Reference advanced concepts and technical details."
    if user_profile.learning_depth == "Practical":
        base += " Include code examples and implementation steps."
    return base
```

## API Protection Strategy

### Decision: CORS + API Key for authenticated calls

**Public endpoints** (no auth required):
- GET /health
- GET / (root)

**Protected endpoints** (auth required via session cookie):
- POST /auth/signup
- POST /auth/signin
- GET /auth/me
- POST /auth/signout
- GET /api/profile
- PUT /api/profile
- POST /api/personalize
- POST /api/chat

**RAG endpoints** (protected):
- POST /chat - Sends user_context with profile data
- POST /search - For chatbot context retrieval

## Implementation Order

1. **Phase 1**: Better Auth setup with SQLite
2. **Phase 2**: Signup/Signin React components in Docusaurus
3. **Phase 3**: Profile management API endpoints
4. **Phase 4**: Personalization toggle and content rendering
5. **Phase 5**: RAG chatbot integration with user context

## Key Dependencies

| Package | Purpose | Version |
|---------|---------|---------|
| better-auth | Authentication | Latest |
| better-sqlite3-adapter | SQLite database | Latest |
| @prisma/client | PostgreSQL database | Latest |
| fastapi-cors | CORS middleware | Latest |

## References

- [Better Auth Documentation](https://www.better-auth.com/docs)
- [Better Auth React Integration](https://www.better-auth.com/docs/integrations/react)
- [Docusaurus Custom Components](https://docusaurus.io/docs/customization)
- [GitHub Pages Custom Domains](https://docs.github.com/en/pages/configuring-a-custom-domain-for-your-github-pages-site)
