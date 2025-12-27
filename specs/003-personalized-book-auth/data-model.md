# Data Model: Personalized Docusaurus Book Platform with Auth & RAG

**Feature**: Personalized Docusaurus Book Platform with Auth & RAG
**Date**: 2025-12-24
**Status**: Active

## Entity Overview

| Entity | Purpose | Storage |
|--------|---------|---------|
| User | Authentication + profile | SQLite/PostgreSQL |
| UserSession | Active sessions | SQLite/PostgreSQL |
| UserProfile | Personalization preferences | SQLite/PostgreSQL |

## User Entity

### Description
Represents a registered reader with authentication credentials and profile preferences.

### Fields

| Field | Type | Required | Validation | Notes |
|-------|------|----------|------------|-------|
| id | UUID | Yes | Auto-generated | Primary key |
| email | String | Yes | Email format, unique | Login identifier |
| password_hash | String | Yes | bcrypt/Argon2 | Never store plaintext |
| email_verified | Boolean | No | Default: false | Future use |
| created_at | DateTime | Yes | Auto-generated | |
| last_login | DateTime | No | Nullable | |
| updated_at | DateTime | No | Auto-updated | |

### Profile Extension (UserProfile)

| Field | Type | Required | Default | Notes |
|-------|------|----------|---------|-------|
| id | UUID | Yes | Auto-generated | FK to User.id |
| software_level | Enum | Yes | Beginner | Beginner/Intermediate/Advanced |
| hardware_experience | Enum | Yes | None | None/Basic/Hands-on |
| learning_depth | Enum | Yes | Both | Conceptual/Practical/Both |
| display_name | String | No | Nullable | Optional display name |
| bio | String | No | Nullable | Optional short bio |

### Enums

```typescript
SoftwareLevel = 'Beginner' | 'Intermediate' | 'Advanced'
HardwareExperience = 'None' | 'Basic' | 'Hands-on'
LearningDepth = 'Conceptual' | 'Practical' | 'Both'
```

### Relationships

```
User (1) ──→ (1) UserProfile
User (1) ──→ (N) UserSession
```

## UserSession Entity

### Description
Tracks active authentication sessions for secure login management.

### Fields

| Field | Type | Required | Validation | Notes |
|-------|------|----------|------------|-------|
| id | UUID | Yes | Auto-generated | Primary key |
| user_id | UUID | Yes | FK to User.id | Foreign key |
| session_token | String | Yes | UUID, unique | JWT token |
| expires_at | DateTime | Yes | Future date | 30 days from creation |
| created_at | DateTime | Yes | Auto-generated | |
| ip_address | String | No | Nullable | Security logging |
| user_agent | String | No | Nullable | Security logging |

### Relationships

```
UserSession (N) ──→ (1) User
```

## UserProfile Entity (Separate Table)

### Description
Extended profile data for personalization preferences.

### Fields

| Field | Type | Required | Default | Notes |
|-------|------|----------|---------|-------|
| id | UUID | Yes | Auto-generated | Primary key (same as User.id) |
| software_level | String | Yes | 'Beginner' | See enum above |
| hardware_experience | String | Yes | 'None' | See enum above |
| learning_depth | String | Yes | 'Both' | See enum above |
| personalization_enabled | Boolean | No | true | Toggle state |
| favorite_chapters | Array[String] | No | [] | Chapter IDs for quick access |
| last_personalized_chapter | String | No | Nullable | Resume functionality |
| created_at | DateTime | Yes | Auto-generated | |
| updated_at | DateTime | No | Auto-updated | |

## SQLite Schema (Development)

```sql
-- Users table (managed by Better Auth)
CREATE TABLE IF NOT EXISTS user (
    id TEXT PRIMARY KEY,
    email TEXT NOT NULL UNIQUE,
    password_hash TEXT NOT NULL,
    email_verified INTEGER DEFAULT 0,
    created_at TEXT NOT NULL,
    last_login TEXT
);

-- User sessions (managed by Better Auth)
CREATE TABLE IF NOT EXISTS session (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL,
    token TEXT NOT NULL UNIQUE,
    expires_at TEXT NOT NULL,
    ip_address TEXT,
    user_agent TEXT,
    created_at TEXT NOT NULL,
    FOREIGN KEY (user_id) REFERENCES user(id)
);

-- Extended profile (custom table)
CREATE TABLE IF NOT EXISTS user_profile (
    id TEXT PRIMARY KEY,
    software_level TEXT NOT NULL DEFAULT 'Beginner',
    hardware_experience TEXT NOT NULL DEFAULT 'None',
    learning_depth TEXT NOT NULL DEFAULT 'Both',
    personalization_enabled INTEGER DEFAULT 1,
    favorite_chapters TEXT,  -- JSON array
    last_personalized_chapter TEXT,
    created_at TEXT NOT NULL,
    updated_at TEXT,
    FOREIGN KEY (id) REFERENCES user(id)
);

-- Indexes
CREATE INDEX idx_session_user_id ON session(user_id);
CREATE INDEX idx_user_email ON user(email);
```

## Validation Rules

### Signup Validation

```python
from pydantic import EmailStr, constr
from typing import Literal

class SignupRequest(BaseModel):
    email: EmailStr
    password: constr(min_length=8, max_length=128)
    confirm_password: str
    software_level: Literal["Beginner", "Intermediate", "Advanced"]
    hardware_experience: Literal["None", "Basic", "Hands-on"]
    learning_depth: Literal["Conceptual", "Practical", "Both"]

    @validator('confirm_password')
    def passwords_match(cls, v, values):
        if 'password' in values and v != values['password']:
            raise ValueError('passwords do not match')
        return v
```

### Profile Update Validation

```python
class ProfileUpdateRequest(BaseModel):
    software_level: Optional[Literal["Beginner", "Intermediate", "Advanced"]]
    hardware_experience: Optional[Literal["None", "Basic", "Hands-on"]]
    learning_depth: Optional[Literal["Conceptual", "Practical", "Both"]]
    display_name: Optional[constr(max_length=100)]
    bio: Optional[constr(max_length=500)]

    class Config:
        extra = "forbid"
```

## State Transitions

### User State Machine

```
┌─────────┐     ┌─────────┐     ┌─────────┐
│ New     │ ──→ │ Active  │ ──→ │ Inactive│
│ User    │     │ User    │     │ User    │
└─────────┘     └─────────┘     └─────────┘
                    │
                    └──→ │ Suspended│
                         │ (future)│
                         └─────────┘
```

### Session State

```
┌─────────┐     ┌─────────┐     ┌─────────┐
│ Valid   │ ──→ │ Expired │ ──→ │ Revoked │
│ Session │     │ Session │     │ Session │
└─────────┘     └─────────┘     └─────────┘
```

## Migrations

### Initial Migration (001_initial_auth.sql)

```sql
-- Create all tables for Phase 1
-- See SQLite schema above
```

### Profile Migration (002_add_profile.sql)

```sql
-- Add profile table if not exists
CREATE TABLE IF NOT EXISTS user_profile (
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
