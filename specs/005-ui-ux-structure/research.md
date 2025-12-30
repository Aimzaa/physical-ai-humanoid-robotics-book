# Research: UI/UX Enhancement and Project Structure Finalization

**Feature Branch**: `005-ui-ux-structure`
**Date**: 2025-12-29

## Research Summary

This feature primarily involves UI/UX improvements to an existing Docusaurus site. No significant unknowns requiring deep research were identified. All technical decisions can be made based on:
1. Existing Docusaurus documentation
2. Current project structure analysis
3. AgentFactory design inspiration (clarified in spec)

## Decision Log

### D1: Docusaurus Custom Homepage Approach

**Decision**: Use `src/pages/index.js` to override default homepage

**Rationale**:
- Docusaurus automatically uses `src/pages/index.js` as the homepage when present
- This is the standard pattern for custom landing pages
- Allows full React component customization while preserving docs routing

**Alternatives Considered**:
- Using docs landing page only (`docs/index.md`) - Rejected: Limited customization for hero section
- Swizzling Layout component - Rejected: Over-engineering for simple hero + cards
- Using a separate landing page route - Rejected: Non-standard, confuses navigation

**Source**: [Docusaurus Pages Documentation](https://docusaurus.io/docs/creating-pages)

---

### D2: Component Styling Approach

**Decision**: Use CSS Modules (`.module.css`) for new components

**Rationale**:
- Already used throughout the project (TranslationToggle, AuthButton, pages)
- Provides scoped styles preventing conflicts
- No new dependencies required
- Consistent with existing codebase patterns

**Alternatives Considered**:
- Tailwind CSS - Rejected: Adds dependency, changes development workflow
- Styled Components - Rejected: Adds dependency, different paradigm
- Global CSS only - Rejected: Risk of style conflicts

**Source**: Project analysis - existing `styles.module.css` files

---

### D3: Color Palette Selection

**Decision**: Modern professional blues/grays palette based on Tailwind CSS color scale

**Rationale**:
- Blues convey professionalism and trust (appropriate for technical/educational content)
- Grays provide neutral backgrounds and text hierarchy
- Tailwind color scale is well-tested for accessibility contrast ratios
- Aligns with AgentFactory's clean, professional aesthetic

**Color Mapping**:
| Role | Color | Hex |
|------|-------|-----|
| Primary | Blue 500 | #3B82F6 |
| Primary Dark | Blue 700 | #1D4ED8 |
| Primary Light | Blue 400 | #60A5FA |
| Background | Slate 50 | #F8FAFC |
| Surface | White | #FFFFFF |
| Text Primary | Slate 800 | #1E293B |
| Text Secondary | Slate 500 | #64748B |

**Alternatives Considered**:
- Green-based (current Docusaurus default) - Rejected: Less professional for technical content
- Purple/violet primary - Rejected: Less common for educational platforms
- Custom brand colors - Rejected: No brand guidelines provided

---

### D4: Module Card Design Pattern

**Decision**: Card-based grid layout with icon, title, subtitle, description, and chapter count

**Rationale**:
- Common pattern on educational platforms (Coursera, AgentFactory, Udemy)
- Provides visual hierarchy and scannable content
- Icon + color coding aids quick module identification
- Grid layout works well on all screen sizes

**Card Structure**:
```
┌─────────────────────────────────────┐
│  [Icon]                             │
│  Module Title                       │
│  Subtitle (Technology)              │
│  Description text...                │
│  6 chapters →                       │
└─────────────────────────────────────┘
```

**Alternatives Considered**:
- List-based navigation - Rejected: Less engaging, harder to scan
- Accordion/collapse pattern - Rejected: Hides content, requires interaction
- Image-heavy cards - Rejected: Requires asset creation, slower loading

---

### D5: Project Structure Preservation

**Decision**: Keep Docusaurus at root level, maintain existing backend/ separation

**Rationale**:
- Standard Docusaurus convention (all docs sites keep it at root)
- Moving to `frontend/` would break build scripts, CI/CD, and conventions
- Current structure already has clear separation (backend/ is isolated)
- No actual restructuring needed - just documentation of current state

**Current Structure Analysis**:
- Root: Docusaurus files (docs/, src/, static/, package.json, docusaurus.config.js)
- backend/: FastAPI server (completely isolated)
- specs/, history/, .specify/: Documentation (preserved, not part of build)

**Alternatives Considered**:
- Move to `frontend/` - Rejected: Breaks conventions, unnecessary complexity
- Monorepo with workspaces - Rejected: Over-engineering for 2-project setup

---

### D6: Footer Cleanup Strategy

**Decision**: Remove default Docusaurus community links, keep project-relevant links only

**Rationale**:
- Current footer has Stack Overflow and Discord links pointing to Docusaurus community (irrelevant)
- Footer should link to: Book content, GitHub repo, and project-specific resources
- Maintains professional appearance without dead-end links

**Links to Remove**:
- Stack Overflow (Docusaurus tag)
- Discord (Docusaurus invite)

**Links to Keep/Update**:
- Book (/docs/index)
- GitHub (project repo)

---

### D7: Navigation Verification Approach

**Decision**: Verify existing Docusaurus navigation features work correctly (no custom implementation needed)

**Rationale**:
- Docusaurus provides sidebar, breadcrumbs, and pagination out of the box
- Current `sidebars.js` correctly defines book structure
- Only need to verify features work, not build new ones

**Features to Verify**:
1. Sidebar category expansion and current page highlighting
2. Breadcrumb display on docs pages
3. Previous/Next pagination links
4. Mobile hamburger menu functionality

---

## Unknowns Resolved

| Unknown | Resolution |
|---------|------------|
| AgentFactory patterns to adopt | Moderate: module cards, hero layout, navigation (clarified in spec) |
| Project restructuring scope | None needed - keep Docusaurus at root (clarified in spec) |
| Design direction | Modern professional with blues/grays (clarified in spec) |
| Homepage customization method | Standard src/pages/index.js override |
| Component styling method | CSS Modules (consistent with existing code) |

## Dependencies

| Dependency | Version | Purpose | Notes |
|------------|---------|---------|-------|
| Docusaurus | 3.x | Site framework | Already installed |
| React | 18.x | Components | Already installed via Docusaurus |
| @docusaurus/preset-classic | 3.x | Default theme | Already installed |

No new dependencies required for this feature.

## Best Practices Applied

1. **Docusaurus Conventions**: Following standard page/component patterns
2. **CSS Modules**: Scoped styling to prevent conflicts
3. **Semantic HTML**: Proper heading hierarchy, accessible markup
4. **Responsive Design**: Mobile-first approach with breakpoints
5. **Performance**: No heavy images, minimal JS, CSS-first animations
6. **Accessibility**: Color contrast ratios, keyboard navigation preserved
