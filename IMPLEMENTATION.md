# Implementation Guide: Personalized Docusaurus Book with Auth & RAG

This document describes the implementation of a personalized Docusaurus book platform with user authentication and RAG chatbot.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     GitHub Pages (Frontend)                  │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                    Docusaurus                         │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │    │
│  │  │ Signup Page │  │ Signin Page │  │ Settings    │  │    │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  │    │
│  │  ┌───────────────────────────────────────────────┐  │    │
│  │  │ AuthButton, PersonalizeButton, ChatbotWidget  │  │    │
│  │  └───────────────────────────────────────────────┘  │    │
│  └─────────────────────────────────────────────────────┘    │
│                            │                                 │
│                            │ HttpOnly Cookies + CORS         │
│                            ▼                                 │
┌─────────────────────────────────────────────────────────────┐
│                    FastAPI Backend                           │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                   Authentication                      │    │
│  │  POST /auth/signup  │  POST /auth/signin            │    │
│  │  GET /auth/me      │  POST /auth/signout            │    │
│  │  GET /auth/status  │                               │    │
│  └─────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                   Profile API                         │    │
│  │  GET /api/profile  │  PUT /api/profile              │    │
│  │  GET /api/preferences                               │    │
│  └─────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────┐    │
│  │               Personalization API                     │    │
│  │  POST /api/personalize/{chapter_id}                  │    │
│  │  GET /api/personalize/{chapter_id}/sections          │    │
│  └─────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                   Chatbot API                         │    │
│  │  POST /api/chat (with user context)                  │    │
│  └─────────────────────────────────────────────────────┘    │
│                            │                                 │
│                            ▼                                 │
│  ┌────────────────────┐  ┌────────────────────┐             │
│  │   SQLite DB        │  │   Qdrant           │             │
│  │   (User + Profile) │  │   (RAG Embeddings) │             │
│  └────────────────────┘  └────────────────────┘             │
└─────────────────────────────────────────────────────────────┘
```

## Backend Implementation

### Directory Structure

```
backend/
├── src/
│   ├── api/
│   │   ├── auth.py         # Authentication endpoints
│   │   ├── profile.py      # Profile management
│   │   ├── personalization.py  # Content personalization
│   │   └── chat.py         # Chatbot with context
│   ├── db/
│   │   ├── init.py         # Database initialization
│   │   └── __init__.py
│   ├── middleware/
│   │   ├── auth.py         # Session handling
│   │   ├── cors.py         # CORS configuration
│   │   └── __init__.py
│   ├── schemas/
│   │   ├── auth.py         # Auth request/response schemas
│   │   ├── profile.py      # Profile schemas
│   │   ├── chat.py         # Chatbot schemas
│   │   └── __init__.py
│   ├── services/
│   │   ├── auth_service.py     # Auth business logic
│   │   ├── personalization_service.py
│   │   ├── chatbot_service.py
│   │   └── __init__.py
│   └── main.py             # FastAPI application
├── requirements.txt
└── .env
```

### Key Components

#### 1. Authentication (src/api/auth.py)

**Endpoints:**
- `POST /auth/signup` - Create account with profile
- `POST /auth/signin` - Sign in, returns HttpOnly cookie
- `GET /auth/me` - Get current user profile
- `POST /auth/signout` - Sign out
- `GET /auth/status` - Check auth status

**Security Features:**
- Password hashing with bcrypt
- HttpOnly cookies for session tokens
- CORS configuration for cross-origin requests
- Session expiration (30 days)

#### 2. User Profile (src/api/profile.py)

Stores user preferences for personalization:
- `software_level`: Beginner/Intermediate/Advanced
- `hardware_experience`: None/Basic/Hands-on
- `learning_depth`: Conceptual/Practical/Both

#### 3. Personalization (src/api/personalization.py)

- `POST /api/personalize/{chapter_id}` - Enable personalization
- `GET /api/personalize/{chapter_id}/sections` - Get sections to apply

#### 4. Chatbot with Context (src/api/chat.py)

Extends existing RAG chatbot with user context awareness.

## Frontend Implementation

### Directory Structure

```
src/
├── components/
│   ├── AuthButton/
│   │   ├── index.js        # Auth status and menu
│   │   ├── index.tsx
│   │   └── styles.module.css
│   ├── PersonalizeButton/
│   │   ├── index.js        # Personalization toggle
│   │   └── styles.module.css
│   └── ChatbotWidget/      # Existing chatbot
├── pages/
│   ├── signup/
│   │   ├── index.js        # Signup form with profile questions
│   │   └── styles.module.css
│   ├── signin/
│   │   ├── index.js        # Signin form
│   │   └── styles.module.css
│   └── settings/
│       ├── index.js        # Profile settings
│       └── styles.module.css
└── plugins/
    └── AuthPlugin/         # Auth plugin
```

### Key Components

#### AuthButton
- Checks auth status on mount
- Shows Sign In/Sign Up for guests
- Shows user email + menu for authenticated users
- Handles sign out

#### PersonalizeButton
- Reads toggle state from localStorage
- Calls API to enable personalization
- Shows user level info
- Prompts sign up for guests

#### Signup Page
- Collects email, password
- Collects profile questions (software level, hardware experience, learning depth)
- Validates password match

#### Settings Page
- Displays current profile
- Allows updating preferences
- Auto-saves on selection

## Database Schema

```sql
-- Users table
CREATE TABLE user (
    id TEXT PRIMARY KEY,
    email TEXT NOT NULL UNIQUE,
    password_hash TEXT NOT NULL,
    email_verified INTEGER DEFAULT 0,
    created_at TEXT NOT NULL,
    last_login TEXT
);

-- Sessions table
CREATE TABLE session (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL,
    token TEXT NOT NULL UNIQUE,
    expires_at TEXT NOT NULL,
    ip_address TEXT,
    user_agent TEXT,
    created_at TEXT NOT NULL,
    FOREIGN KEY (user_id) REFERENCES user(id)
);

-- User profile table
CREATE TABLE user_profile (
    id TEXT PRIMARY KEY,
    software_level TEXT NOT NULL DEFAULT 'Beginner',
    hardware_experience TEXT NOT NULL DEFAULT 'None',
    learning_depth TEXT NOT NULL DEFAULT 'Both',
    personalization_enabled INTEGER DEFAULT 1,
    favorite_chapters TEXT,
    last_personalized_chapter TEXT,
    created_at TEXT NOT NULL,
    updated_at TEXT,
    FOREIGN KEY (id) REFERENCES user(id)
);
```

## Running the Application

### 1. Backend

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows

# Install dependencies
pip install -r requirements.txt

# Initialize database
python -m src.db.init

# Run server
python main.py
# Server runs on http://localhost:8000
```

### 2. Frontend

```bash
# From project root
npm install
npm start
# Frontend runs on http://localhost:3000
```

### 3. Environment Variables

Add to `.env`:
```env
# Auth
AUTH_SECRET=your-super-secret-key-min-32-chars
JWT_SECRET=your-jwt-secret-key
DATABASE_URL=sqlite:///./backend/data/auth.db
FRONTEND_URL=http://localhost:3000
COOKIE_MAX_AGE=2592000
```

## Deployment

### Backend
Deploy to any Python-compatible hosting (Railway, Render, Fly.io):
1. Set `AUTH_SECRET` and `JWT_SECRET` to secure values
2. Set `DATABASE_URL` to PostgreSQL for production
3. Update `FRONTEND_URL` to your GitHub Pages domain
4. Set `COOKIE_SECURE=true` for HTTPS

### Frontend (GitHub Pages)
```bash
npm run build
npx docusaurus deploy
```

Update `docusaurus.config.js` with your GitHub org/name.

## Content Personalization

### Adding Personalized Sections

In chapter markdown files, use Docusaurus admonitions:

```markdown
# Robot Kinematics

## Introduction
Kinematics is the study of motion...

:::note[beginner]
## What is a Kinematic Chain?
A kinematic chain is like a series of connected arms that can move...
:::

:::note[intermediate]
## Forward Kinematics Equations
The DH (Denavit-Hartenberg) parameters define the geometry...
:::

:::note[advanced]
## Jacobian Analysis
The Jacobian matrix relates joint velocities to end-effector velocity...
:::
```

### How It Works

1. User enables personalization for a chapter
2. Frontend reads user profile from context
3. ContentAdapter component filters sections based on user level
4. Only sections matching user level are displayed

## API Reference

### Authentication Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | /auth/signup | Create account with profile |
| POST | /auth/signin | Sign in, returns cookie |
| GET | /auth/me | Get current user |
| POST | /auth/signout | Sign out |
| GET | /auth/status | Check auth status |

### Profile Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /api/profile | Get user profile |
| PUT | /api/profile | Update profile |
| GET | /api/preferences | Get personalization prefs |

### Personalization Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | /api/personalize/{id} | Enable personalization |
| POST | /api/personalize/{id}/disable | Disable personalization |
| GET | /api/personalize/{id}/sections | Get sections to apply |

### Chatbot Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | /api/chat | Chat with context-aware RAG |

## Security Considerations

1. **Password Storage**: bcrypt hashing with salt
2. **Session Security**: HttpOnly cookies prevent XSS
3. **CORS**: Restricted to frontend domain
4. **Input Validation**: Pydantic schemas validate all input
5. **SQL Injection**: Parameterized queries via SQLite

## Next Steps

1. Add rate limiting for auth endpoints
2. Implement email verification
3. Add social login (Google, GitHub)
4. Add analytics for personalization effectiveness
5. Implement content A/B testing
