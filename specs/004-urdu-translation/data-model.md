# Data Model: Urdu Translation

**Feature**: Chapter-Level Urdu Translation
**Date**: 2025-12-28
**Based On**: [spec.md](./spec.md) and [research.md](./research.md)

---

## Entity: TranslationCache

Stores translated chapter content per user with TTL-based expiration.

### Attributes

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `id` | UUID | Yes | Auto-generated | Primary key |
| `user_id` | UUID (FK) | Yes | Valid user.id | Reference to user |
| `chapter_id` | String | Yes | Non-empty | Chapter identifier |
| `original_content_hash` | String | Yes | SHA-256 | Hash of source content |
| `translated_content` | Text | Yes | Non-empty | Urdu translated markdown |
| `user_profile_snapshot` | JSON | Yes | Valid profile JSON | User level at translation time |
| `created_at` | ISO8601 | Yes | Auto-generated | Creation timestamp |
| `expires_at` | ISO8601 | Yes | Future timestamp | Cache expiration |

### Relationships

```
TranslationCache
  belongs_to → User (user_id)
  indexed_by → (user_id, chapter_id) for fast lookup
  indexed_by → (expires_at) for cleanup queries
```

### State Transitions

```
NEW → VALID (cache created, within TTL)
VALID → EXPIRED (TTL passed, auto-cleaned)
VALID → INVALIDATED (content changed, profile changed, manual clear)
```

### Validation Rules

1. Unique constraint: `(user_id, chapter_id, original_content_hash)`
2. TTL range: 24-48 hours (86400-172800 seconds)
3. Content hash must match current chapter content for cache hit

---

## Entity: TranslationRequest

Temporary storage for in-progress translation requests (prevents duplicate calls).

### Attributes

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | UUID | Yes | Request tracking ID |
| `user_id` | UUID | Yes | Requesting user |
| `chapter_id` | String | Yes | Target chapter |
| `content_hash` | String | Yes | Content being translated |
| `status` | Enum | Yes | pending\|processing\|completed\|failed |
| `created_at` | ISO8601 | Yes | Request timestamp |
| `completed_at` | ISO8601 | No | Completion timestamp |

### Status Values

| Status | Meaning |
|--------|---------|
| `pending` | Awaiting processing |
| `processing` | LLM call in progress |
| `completed` | Translation finished |
| `failed` | Error occurred |

---

## Entity: TranslationPreference

User-level preferences for translation behavior.

### Attributes

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `user_id` | UUID (PK) | - | Reference to user |
| `auto_translate` | Boolean | false | Auto-show Urdu on chapter visit |
| `remember_preference` | Boolean | true | Persist language choice per chapter |
| `preferred_depth` | String | "Both" | Default learning depth for translation |

### Integration with Existing UserProfile

```
UserProfile (existing)
  has_one → TranslationPreference (extension)
  shares user_id as primary key
```

---

## Database Schema (SQLite)

```sql
-- Extension to existing auth.db

-- Translation cache table
CREATE TABLE IF NOT EXISTS translation_cache (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL,
    chapter_id TEXT NOT NULL,
    original_content_hash TEXT NOT NULL,
    translated_content TEXT NOT NULL,
    user_profile_snapshot TEXT NOT NULL,
    created_at TEXT NOT NULL,
    expires_at TEXT NOT NULL,
    FOREIGN KEY (user_id) REFERENCES user(id),
    UNIQUE (user_id, chapter_id, original_content_hash)
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_translation_cache_user_chapter
    ON translation_cache(user_id, chapter_id);
CREATE INDEX IF NOT EXISTS idx_translation_cache_expires
    ON translation_cache(expires_at);
CREATE INDEX IF NOT EXISTS idx_translation_cache_hash
    ON translation_cache(original_content_hash);

-- In-progress request tracking
CREATE TABLE IF NOT EXISTS translation_request (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL,
    chapter_id TEXT NOT NULL,
    content_hash TEXT NOT NULL,
    status TEXT NOT NULL DEFAULT 'pending',
    created_at TEXT NOT NULL,
    completed_at TEXT,
    FOREIGN KEY (user_id) REFERENCES user(id)
);

-- Translation preferences
CREATE TABLE IF NOT EXISTS translation_preference (
    user_id TEXT PRIMARY KEY,
    auto_translate INTEGER NOT NULL DEFAULT 0,
    remember_preference INTEGER NOT NULL DEFAULT 1,
    preferred_depth TEXT NOT NULL DEFAULT 'Both',
    FOREIGN KEY (user_id) REFERENCES user(id)
);
```

---

## API Data Models (Pydantic)

```python
from pydantic import BaseModel
from datetime import datetime
from typing import Optional

class TranslationRequest(BaseModel):
    """Request to translate chapter content"""
    chapter_id: str
    content: str
    chapter_title: str

class TranslationResponse(BaseModel):
    """Translation API response"""
    success: bool
    translated_content: str
    cached: bool
    ttl_seconds: int
    language: str = "ur"

class TranslationStatusResponse(BaseModel):
    """Status check response"""
    has_translation: bool
    cached_at: Optional[datetime]
    ttl_remaining: Optional[int]
    language: str

class UserProfileSnapshot(BaseModel):
    """Snapshot of user profile at translation time"""
    software_level: str
    hardware_experience: str
    learning_depth: str

class TranslationPreferenceUpdate(BaseModel):
    """User preference update"""
    auto_translate: Optional[bool] = None
    remember_preference: Optional[bool] = None
    preferred_depth: Optional[str] = None
```

---

## Frontend State Model

```typescript
interface TranslationState {
  language: 'en' | 'ur';
  isTranslating: boolean;
  translatedContent: string | null;
  originalContent: string;
  error: string | null;
  lastTranslatedAt: datetime | null;
  cacheExpiresAt: datetime | null;
}

interface TranslationToggleProps {
  chapterId: string;
  chapterTitle: string;
  onLanguageChange?: (language: 'en' | 'ur') => void;
}
```

---

## Caching Strategy

### Cache Key Generation

```
Primary Key: translation:{user_id}:{chapter_id}
Hash Key: translation:{user_id}:{chapter_id}:{content_hash}

Example:
- User: abc123
- Chapter: chapter-1-1
- Content Hash: a1b2c3d4
- Key: translation:abc123:chapter-1-1:a1b2c3d4
```

### TTL Configuration

| Scenario | TTL | Rationale |
|----------|-----|-----------|
| Standard content | 48 hours | Balance freshness/cost |
| Frequently updated chapters | 24 hours | Content may change |
| Static reference chapters | 72 hours | Lower change likelihood |

### Invalidation Conditions

1. **Time-based**: TTL expires
2. **Content-based**: `original_content_hash` mismatch
3. **Profile-based**: User profile update detected
4. **Manual**: User clears cache for chapter

---

## Entity Relationships Diagram

```
┌─────────────────┐       ┌──────────────────────┐
│      User       │──────▶│ TranslationCache     │
│   (existing)    │       │                      │
└─────────────────┘       │ - id                 │
                          │ - chapter_id         │
                          │ - translated_content │
                          │ - expires_at         │
                          └──────────────────────┘
                                  │
                                  │ has many
                                  ▼
                    ┌─────────────────────────┐
                    │   TranslationRequest    │
                    │                         │
                    │ - status (pending/etc)  │
                    └─────────────────────────┘
                                  │
                                  │ optional 1:1
                                  ▼
                    ┌─────────────────────────┐
                    │ TranslationPreference   │
                    │ (user extension)        │
                    │                         │
                    │ - auto_translate        │
                    │ - preferred_depth       │
                    └─────────────────────────┘
```
