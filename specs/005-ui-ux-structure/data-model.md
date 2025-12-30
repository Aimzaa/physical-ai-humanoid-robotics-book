# Data Model: UI/UX Enhancement and Project Structure Finalization

**Feature Branch**: `005-ui-ux-structure`
**Date**: 2025-12-29

## Overview

This feature is primarily UI/UX focused with no database changes. The data model describes the frontend data structures used for rendering components.

## Entities

### Module (Frontend Data Structure)

Represents a major section of the book displayed as a card on the homepage.

```typescript
interface Module {
  id: string;           // Unique identifier (e.g., 'module-1')
  title: string;        // Full module title
  subtitle: string;     // Technology focus (e.g., 'ROS 2')
  description: string;  // Brief description (1-2 sentences)
  icon: string;         // Emoji or icon identifier
  color: string;        // Hex color for accent
  link: string;         // URL to first chapter
  chapters: number;     // Total chapter count
}
```

**Source**: Hardcoded in `src/pages/index.js` or extracted from `sidebars.js`

**Instance Data**:

| id | title | subtitle | color | chapters |
|----|-------|----------|-------|----------|
| module-1 | Module 1 — The Robotic Nervous System | ROS 2 | #3B82F6 | 6 |
| module-2 | Module 2 — The Digital Twin | Gazebo & Unity | #10B981 | 6 |
| module-3 | Module 3 — The AI-Robot Brain | NVIDIA Isaac | #8B5CF6 | 6 |
| module-4 | Module 4 — Vision-Language-Action | VLA | #F59E0B | 6 |

---

### NavigationItem (Existing - No Changes)

Defined in `sidebars.js`, represents sidebar navigation structure.

```typescript
interface NavigationCategory {
  type: 'category';
  label: string;
  items: string[];  // Doc IDs
}

interface NavigationDoc {
  type: 'doc';
  id: string;
}

type NavigationItem = NavigationCategory | NavigationDoc | string;
```

**Current Structure** (from sidebars.js):
- 1 index doc
- 4 module categories (6 chapters each)
- 1 capstone category (4 chapters)
- 1 references doc

---

### DesignToken (CSS Variables)

Design system tokens defined in `src/css/custom.css`.

```css
:root {
  /* Colors */
  --ifm-color-primary: #3B82F6;
  --ifm-color-primary-dark: #1D4ED8;
  --ifm-color-primary-light: #60A5FA;
  --color-background: #F8FAFC;
  --color-surface: #FFFFFF;
  --color-text-primary: #1E293B;
  --color-text-secondary: #64748B;

  /* Spacing */
  --spacing-xs: 4px;
  --spacing-sm: 8px;
  --spacing-md: 16px;
  --spacing-lg: 24px;
  --spacing-xl: 32px;
  --spacing-2xl: 48px;

  /* Border Radius */
  --radius-sm: 4px;
  --radius-md: 8px;
  --radius-lg: 12px;

  /* Shadows */
  --shadow-sm: 0 1px 2px rgba(0, 0, 0, 0.05);
  --shadow-md: 0 4px 6px rgba(0, 0, 0, 0.1);
  --shadow-lg: 0 10px 15px rgba(0, 0, 0, 0.1);
}
```

---

## Component Props

### HomepageHero Props

```typescript
interface HomepageHeroProps {
  title?: string;        // Default: from docusaurus.config.js
  subtitle?: string;     // Default: from docusaurus.config.js
  ctaText?: string;      // Default: "Start Learning"
  ctaLink?: string;      // Default: "/docs/index"
}
```

### ModuleCard Props

```typescript
interface ModuleCardProps {
  module: Module;        // Module data
  className?: string;    // Additional CSS classes
}
```

### ModuleGrid Props

```typescript
interface ModuleGridProps {
  modules: Module[];     // Array of modules to display
}
```

---

## File Structure (New Components)

```
src/components/
├── HomepageHero/
│   ├── index.js           # Component implementation
│   └── styles.module.css  # Scoped styles
└── ModuleCard/
    ├── index.js           # Component implementation
    └── styles.module.css  # Scoped styles

src/pages/
└── index.js               # Homepage with Hero + ModuleGrid
```

---

## Data Flow

```
docusaurus.config.js          sidebars.js
       │                           │
       ▼                           ▼
   Site Title/Tagline         Module Structure
       │                           │
       └───────────┬───────────────┘
                   ▼
            src/pages/index.js
                   │
       ┌───────────┴───────────┐
       ▼                       ▼
  HomepageHero            ModuleGrid
       │                       │
       ▼                       ▼
  Hero Section           ModuleCard × 4
       │                       │
       └───────────┬───────────┘
                   ▼
              Homepage UI
```

---

## Relationships

| Entity | Relates To | Cardinality | Description |
|--------|------------|-------------|-------------|
| Module | Chapter | 1:N | Each module contains 6 chapters |
| Module | ModuleCard | 1:1 | Each module renders as one card |
| DesignToken | All Components | 1:N | Tokens used across all UI |

---

## Validation Rules

| Entity | Field | Rule |
|--------|-------|------|
| Module | id | Required, unique, kebab-case |
| Module | title | Required, max 100 chars |
| Module | link | Required, valid internal route |
| Module | color | Required, valid hex color |
| Module | chapters | Required, positive integer |

---

## State Transitions

This feature has no dynamic state transitions. All data is static and rendered at build time.

---

## No Database Changes

This feature does not modify:
- Backend database schema
- API contracts
- Authentication/authorization
- User data storage

All changes are frontend-only, affecting React components and CSS.
