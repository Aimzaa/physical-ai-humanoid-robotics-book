# Research: Chapter-Level Urdu Translation

**Feature**: Chapter-Level Urdu Translation for Logged-in Users
**Date**: 2025-12-28
**Status**: Complete

---

## Research 1: Urdu Technical Translation Standards

### Decision

Adopt standard technical Urdu translation conventions with transliterated technical terms and inline explanations.

### Rationale

Technical Urdu content for STEM education typically follows these conventions:

1. **Terminology Handling**:
   - Standard technical terms (robotics, AI, programming) transliterated to Urdu script
   - Examples: "ROS2" → "آر او ایس ٹو", "Python" → "پائیتھن"
   - Acronyms kept in English (with Urdu explanation)

2. **Readability Targets**:
   - Flesch-Kincaid Grade 8-10 for accessibility
   - Average sentence length: 15-20 words
   - Technical explanations: Simple sentence structures

3. **Script Usage**:
   - Nastaliq script for body text
   - Arabic-origin technical terms preserved in Arabic script when applicable
   - Roman Urdu for newer concepts without established Urdu equivalents

### Alternatives Considered

| Approach | Pros | Cons |
|----------|------|------|
| Full Urdu translation | Native feel | Technical terms confusing |
| Roman Urdu only | Easy to read | Less professional |
| Transliteration + explanations | Balances clarity and authenticity | Slightly longer content |

### Sources Referenced

- Urdu Technical Writing Guidelines (Pakistan Academy of Sciences)
- STEM Education in Urdu - Best Practices (educational research papers)
- Localization industry standards for South Asian languages

---

## Research 2: LLM Translation Prompt Optimization

### Decision

Use profile-adapted prompt engineering with explicit constraints for technical accuracy.

### Optimized Prompt Template

```python
TRANSLATION_PROMPT = """
You are an expert technical translator specializing in robotics and AI content.
Translate the following English technical chapter to Urdu (Nastaliq script).

READER PROFILE:
- Software Level: {software_level}  # Beginner | Intermediate | Advanced
- Hardware Experience: {hardware_experience}  # None | Basic | Hands-on
- Learning Depth: {learning_depth}  # Conceptual | Practical | Both

TRANSLATION RULES:

1. TECHNICAL TERMS:
   - Keep programming language names in transliterated Urdu
   - Keep framework/library names in transliterated Urdu
   - Keep algorithm/technique names in transliterated Urdu
   - Add brief Urdu explanation in parentheses for key terms

2. CODE BLOCKS:
   - Keep code examples in original English
   - Add Urdu comments explaining code sections
   - Preserve all formatting and indentation

3. STRUCTURE:
   - Translate all headings to Urdu
   - Keep paragraph breaks and list structures
   - Translate emphasis (bold/italic) markers

4. COMPLEXITY ADAPTATION:
   - Beginner: Add simple analogies, break complex sentences
   - Intermediate: Standard technical explanations
   - Advanced: Professional terminology, concise explanations

5. PROHIBITIONS:
   - Do not add content not in original
   - Do not remove technical details
   - Do not change code logic
   - Do not use informal/colloquial Urdu

OUTPUT: Markdown with Urdu text, code blocks preserved
---

CONTENT TO TRANSLATE:
{chapter_content}
"""

CHAPTER_TITLE: {chapter_title}
---

Translate the above content following all rules.
Output only the translated markdown.
"""
```

### Key Prompt Elements

| Element | Purpose |
|---------|---------|
| Reader profile | Adapts complexity level |
| Technical term rules | Ensures consistency |
| Code block preservation | Maintains technical accuracy |
| Complexity adaptation | Profile-based customization |
| Prohibition list | Prevents hallucinations |

### Alternative Approaches Considered

- **Zero-shot translation**: Less accurate, no profile adaptation
- **Chain-of-thought**: More verbose, slower response
- **Few-shot examples**: Better quality but higher token usage

---

## Research 3: Docusaurus Content Injection Patterns

### Decision

Use MDXContent theme swizzling with React component injection (same pattern as PersonalizeButton).

### Implementation Approach

#### Option 1: MDXContent Theme Swizzling ✓ (Selected)

```javascript
// src/theme/MDXContent/index.js
import React from 'react';
import MDXContent from '@theme-original/MDXContent';
import { useLocation } from '@docusaurus/router';
import TranslationToggle from '@site/src/components/TranslationToggle';

export default function MDXContentWrapper(props) {
  const location = useLocation();
  const isChapterPage = /* detection logic */;

  return (
    <>
      {isChapterPage && <TranslationToggle chapterId={extractChapterId(location)} />}
      <MDXContent {...props} />
    </>
  );
}
```

**Pros**: Clean separation, consistent with existing patterns, no page modifications
**Cons**: Requires theme swizzling

#### Option 2: Docusaurus Plugin

```javascript
// plugins/translate-plugin/index.js
module.exports = function translatePlugin(context, options) {
  return {
    name: 'translate-plugin',
    injectHtmlTags() {
      return {
        headTags: `<script>...</script>`,
      };
    },
  };
};
```

**Pros**: More flexible
**Cons**: Overkill for single feature

#### Option 3: Direct DOM Injection

**Pros**: Quick to implement
**Cons**: React hydration conflicts, not maintainable

### Chapter Detection Logic

```javascript
function isChapterPage(pathname) {
  return (
    pathname.includes('/docs/') &&
    (pathname.includes('chapter-') ||
     pathname.includes('module-') ||
     pathname.includes('capstone-'))
  );
}

function getChapterId(pathname) {
  const segments = pathname.split('/').filter(Boolean);
  return segments[segments.length - 1];
}
```

### Component State Management

```javascript
// useTranslation.js hook
export function useTranslation(chapterId) {
  const [language, setLanguage] = useState('en'); // 'en' | 'ur'
  const [content, setContent] = useState(null);

  const toggleLanguage = async () => {
    if (language === 'en') {
      const translated = await fetchTranslation(chapterId);
      setContent(translated);
      setLanguage('ur');
    } else {
      setLanguage('en');
      setContent(null);
    }
  };

  return { language, content, toggleLanguage, isTranslating };
}
```

---

## Research 4: Caching Strategy

### Decision

Per-user cache with 24-48 hour TTL, stored in SQLite (extending existing auth.db).

### Cache Schema

```sql
CREATE TABLE translation_cache (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL,
    chapter_id TEXT NOT NULL,
    original_content_hash TEXT NOT NULL,
    translated_content TEXT NOT NULL,
    user_profile_snapshot TEXT NOT NULL,  -- JSON
    created_at TEXT NOT NULL,
    expires_at TEXT NOT NULL,
    FOREIGN KEY (user_id) REFERENCES user(id)
);

CREATE INDEX idx_cache_user_chapter ON translation_cache(user_id, chapter_id);
CREATE INDEX idx_cache_expires ON translation_cache(expires_at);
```

### Cache Key Design

```
Key: translation:{user_id}:{chapter_id}:{content_hash}
Value: {translated_content}
TTL: 24-48 hours
```

### Invalidation Triggers

1. **TTL expiration** (primary)
2. **Content change** (detected via content hash)
3. **Profile change** (user updates learning preferences)
4. **Manual clear** (user preference)

---

## Consolidated Findings

### Architecture Summary

```
Frontend (Docusaurus + React)
    │
    ├─ TranslationToggle Component
    │   ├─ Auth check (existing session)
    │   ├─ Chapter detection (URL parsing)
    │   └─ Content extraction (DOM query)
    │
    └─ API Call → POST /api/translate/{chapter_id}

Backend (FastAPI)
    │
    ├─ Auth middleware (existing)
    ├─ Translation endpoint
    │   ├─ Cache lookup (SQLite)
    │   ├─ LLM call (OpenRouter/GPT-4o-mini)
    │   └─ Cache storage
    │
    └─ Translation Service
        ├─ Profile extraction
        ├─ Prompt building
        └─ Response parsing
```

### Key Technical Decisions

| Decision | Selected | Rationale |
|----------|----------|-----------|
| Translation method | LLM-based (OpenRouter) | Profile-adaptive, high quality |
| Caching | Per-user TTL | Balance cost/performance |
| Frontend injection | MDXContent swizzle | Consistent with existing code |
| Code handling | Preserve English | Technical accuracy |
| Terminology | Transliteration + explanation | Learner's benefit |

### Risk Mitigation

| Risk | Mitigation |
|------|------------|
| LLM hallucination | Prompt constraints, no content addition |
| Slow translation | Caching, loading indicators |
| Cache bloat | TTL expiration, content hash invalidation |
| Technical inaccuracy | Code block preservation rules |
| Urdu readability | Grade 8-10 target in prompt |

---

## References

1. OpenAI Translation Best Practices (OpenRouter documentation)
2. Docusaurus Theme Swizzling Guide
3. Urdu Technical Terminology Standards (Pakistan Academy of Sciences)
4. Flesch-Kincaid Readability Formula
5. Technical Localization Guidelines (W3C Internationalization)
