# Quickstart: Urdu Translation Feature

**Feature**: Chapter-Level Urdu Translation for Logged-in Users
**Date**: 2025-12-28

---

## Prerequisites

| Tool | Version | Purpose |
|------|---------|---------|
| Node.js | 18+ | Frontend development |
| Python | 3.11+ | Backend development |
| npm/pip | Latest | Package management |
| Git | Latest | Version control |

## Setup Instructions

### 1. Clone and Install Dependencies

```bash
# Clone the repository
git clone https://github.com/your-org/humanoid-robotics-Book.git
cd humanoid-robotics-Book

# Install frontend dependencies
npm install

# Install backend dependencies
cd backend
pip install -r requirements.txt
cd ..
```

### 2. Environment Configuration

Create `.env` file in `backend/` directory:

```bash
# Required
OPENROUTER_API_KEY=your_openrouter_api_key

# Optional (defaults shown)
TRANSLATION_CACHE_TTL=172800  # 48 hours in seconds
TRANSLATION_MAX_TOKENS=4000
TRANSLATION_MODEL=gpt-4o-mini
```

Create `.env` file in root directory (frontend):

```bash
# Already configured from existing setup
```

### 3. Database Setup

```bash
# Backend database is auto-initialized
cd backend
python -c "from src.db import init_db; init_db()"
cd ..
```

This creates the following tables in `backend/src/data/auth.db`:
- `translation_cache` (new)
- `translation_request` (new)
- `translation_preference` (new)

### 4. Start Development Servers

**Terminal 1 - Backend:**
```bash
cd backend
python main.py
# Server runs at http://localhost:8000
```

**Terminal 2 - Frontend:**
```bash
npm start
# Docusaurus runs at http://localhost:3000
```

---

## Development Workflow

### Running Tests

```bash
# Backend tests
cd backend
pytest tests/ -v

# Frontend tests
npm test
```

### Translation API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/translate/{chapter_id}` | Translate chapter to Urdu |
| GET | `/api/translate/{chapter_id}/status` | Check cache status |
| DELETE | `/api/translate/{chapter_id}` | Clear cached translation |
| GET | `/api/translate/preferences` | Get user preferences |
| PUT | `/api/translate/preferences` | Update preferences |

### Example API Calls

```bash
# Translate a chapter (requires authentication via cookie)
curl -X POST http://localhost:8000/api/translate/chapter-1-1 \
  -H "Content-Type: application/json" \
  -H "Origin: http://localhost:3000" \
  --cookie-jar cookies.txt \
  -d '{
    "chapter_id": "chapter-1-1",
    "chapter_title": "Introduction to ROS 2",
    "content": "# Introduction to ROS 2\n\nROS 2 is..."
  }'

# Check translation status
curl http://localhost:8000/api/translate/chapter-1-1/status \
  --cookie cookies.txt

# Clear cache
curl -X DELETE http://localhost:8000/api/translate/chapter-1-1 \
  --cookie cookies.txt
```

---

## Project Structure

```
humanoid-robotics-Book/
├── backend/
│   ├── src/
│   │   ├── api/
│   │   │   └── translation.py      # NEW: Translation endpoints
│   │   ├── services/
│   │   │   └── translation_service.py  # NEW: LLM translation logic
│   │   └── db.py                   # Updated with translation tables
│   └── main.py                     # Updated with translation routes
│
├── frontend/
│   └── src/
│       ├── components/
│       │   └── TranslationToggle/  # NEW: Toggle button component
│       ├── hooks/
│       │   └── useTranslation.js   # NEW: Translation state hook
│       └── theme/
│           └── MDXContent/
│               └── index.js        # UPDATED: Inject TranslationToggle
│
└── specs/
    └── 004-urdu-translation/
        ├── spec.md
        ├── plan.md
        ├── research.md
        ├── data-model.md
        ├── quickstart.md           # This file
        └── contracts/
            ├── openapi.yaml        # API specification
            └── frontend.yml        # Component contracts
```

---

## Key Files to Modify

### New Files (Create)

| File | Purpose |
|------|---------|
| `backend/src/api/translation.py` | Translation API endpoints |
| `backend/src/services/translation_service.py` | LLM translation logic |
| `frontend/src/components/TranslationToggle/index.js` | Toggle button component |
| `frontend/src/components/TranslationToggle/styles.module.css` | Component styles |
| `frontend/src/hooks/useTranslation.js` | Translation state hook |

### Modified Files (Update)

| File | Change |
|------|--------|
| `backend/src/db.py` | Add translation tables to schema |
| `backend/main.py` | Add translation router |
| `frontend/src/theme/MDXContent/index.js` | Inject TranslationToggle |

---

## Translation Prompt Engineering

The translation uses a profile-adaptive prompt system:

```python
# Example prompt construction
TRANSLATION_PROMPT = """
You are an expert technical translator...
Reader Profile:
- Software Level: {user.software_level}
- Hardware Experience: {user.hardware_experience}
- Learning Depth: {user.learning_depth}

Translate the following content to Urdu (Nastaliq)...

{chapter_content}
"""
```

---

## Testing the Feature

### Manual Testing Steps

1. **Start servers** (see above)
2. **Create account** or login at `/signup` or `/signin`
3. **Navigate** to a chapter (e.g., `/docs/module-1-robotic-nervous-system/chapter-1-1-introduction-ros2`)
4. **Verify** "Translate to Urdu" button appears at top
5. **Click** the button
6. **Wait** for translation (~2-5 seconds)
7. **Verify** content displays in Urdu
8. **Click** "Show English" to toggle back

### User Story Testing

#### US1: Toggle Translation
```bash
# Test steps:
1. Login to the application
2. Navigate to any chapter
3. Click "Translate to Urdu" button
4. Verify: Loading spinner appears
5. Verify: Urdu content displays after translation
6. Click "Show English"
7. Verify: Original English content returns
```

#### US2: Profile-Adapted Translation
```bash
# Test with different profiles:
1. Set profile to Beginner at /settings
2. Request translation
3. Verify: Simple explanations with clarifications

4. Set profile to Advanced at /settings
5. Request translation
6. Verify: Technical terminology preserved, concise explanations
```

#### US3: Translation Persistence
```bash
# Test cache persistence:
1. Translate a chapter to Urdu
2. Navigate to a different chapter
3. Return to the first chapter
4. Verify: Urdu content displays automatically
5. Click "Show English"
6. Navigate away and return
7. Verify: Stays in English mode
```

#### US4: Guest User Handling
```bash
# Test unauthenticated access:
1. Logout from the application
2. Visit any chapter
3. Verify: No "Translate to Urdu" button appears
4. Try direct API call (curl)
5. Verify: Returns 401 Unauthorized
```

### Automated Tests

```bash
# Run all tests
npm test && cd ../backend && pytest

# Run only translation tests
cd backend
pytest tests/test_translation.py -v
```

### Cache Management

```bash
# Run cache cleanup (for cron job)
cd backend
python -m src.services.translation_service --cleanup

# Clean up expired translations for specific user
python -m src.services.translation_service --cleanup-user USER_ID
```

---

## Troubleshooting

### Issue: Translation fails with 401

**Cause**: Not authenticated or session expired

**Solution**: Login at `/signin` and ensure cookies are being sent

### Issue: Translation times out

**Cause**: LLM service slow or unavailable

**Solution**: Check OpenRouter API key and quota

### Issue: Cached translation not showing

**Cause**: Content hash mismatch or cache expired

**Solution**: Check cache TTL, try re-translating

### Issue: Urdu text displays incorrectly

**Cause**: Font not loaded or encoding issue

**Solution**: Ensure Docusaurus loads appropriate font for Urdu script

---

## Performance Targets

| Metric | Target | How to Monitor |
|--------|--------|----------------|
| Translation time | <5s | API response time |
| Toggle time | <100ms | Client-side only |
| Cache hit rate | >70% | API response `cached` field |
| Error rate | <5% | API error responses |

---

## Next Steps

1. Complete Phase 1 design artifacts (this document)
2. Run `/sp.tasks` to generate implementation tasks
3. Execute tasks in order
4. Test all user scenarios
5. Deploy to staging environment
