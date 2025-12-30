# Tasks: UI/UX Enhancement and Project Structure Finalization

**Feature**: UI/UX Enhancement and Project Structure Finalization
**Branch**: `005-ui-ux-structure`
**Input**: Design documents from `/specs/005-ui-ux-structure/`
**Tests**: NOT included (not explicitly requested in feature specification)

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project verification and baseline establishment

- [x] T001 Verify current project builds successfully with `npm run build`
- [x] T002 [P] Document current footer links in docusaurus.config.js for audit
- [x] T003 [P] Create src/components/HomepageHero/ directory structure
- [x] T004 [P] Create src/components/ModuleCard/ directory structure

---

## Phase 2: Foundational (Design System Setup)

**Purpose**: Establish design system that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until design system is in place

- [x] T005 Update src/css/custom.css with new color palette variables (--ifm-color-primary: #3B82F6, etc.)
- [x] T006 Add spacing scale CSS variables to src/css/custom.css (--spacing-xs through --spacing-2xl)
- [x] T007 Add border-radius and shadow CSS variables to src/css/custom.css
- [x] T008 [P] Verify dark mode compatibility with new color variables in src/css/custom.css
- [x] T009 Run `npm start` to verify design system changes don't break existing UI

**Checkpoint**: Design system ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Enhanced Homepage Experience (Priority: P1) 🎯 MVP

**Goal**: Create a professional homepage with hero section and module cards that guides visitors to start learning

**Independent Test**: Visit homepage at `/`, verify hero section with title/subtitle/CTA button displays, 4 module cards are visible and clickable, each card navigates to correct module

### Implementation for User Story 1

- [x] T010 [P] [US1] Create HomepageHero component in src/components/HomepageHero/index.js
- [x] T011 [P] [US1] Create HomepageHero styles in src/components/HomepageHero/styles.module.css
- [x] T012 [P] [US1] Create ModuleCard component in src/components/ModuleCard/index.js
- [x] T013 [P] [US1] Create ModuleCard styles in src/components/ModuleCard/styles.module.css
- [x] T014 [US1] Create homepage with Hero and ModuleGrid in src/pages/index.js
- [x] T015 [US1] Define module data array with all 4 modules in src/pages/index.js
- [x] T016 [US1] Implement responsive grid layout for module cards in src/pages/index.js
- [x] T017 [US1] Add hover effects and transitions to ModuleCard in src/components/ModuleCard/styles.module.css
- [x] T018 [US1] Verify all 4 module card links navigate correctly to first chapter of each module
- [x] T019 [US1] Test homepage on mobile viewport (375px width) for responsiveness

**Checkpoint**: Homepage MVP complete - hero + module cards functional

---

## Phase 4: User Story 2 - Intuitive Navigation System (Priority: P1)

**Goal**: Ensure clear navigation with sidebar highlighting, breadcrumbs, and prev/next pagination

**Independent Test**: Navigate through 3+ chapters, verify sidebar highlights current page, breadcrumbs show correct path, prev/next buttons work

### Implementation for User Story 2

- [x] T020 [US2] Verify sidebar category expansion and current page highlighting in docs pages
- [x] T021 [US2] Verify breadcrumb navigation displays correctly on chapter pages
- [x] T022 [US2] Verify Previous/Next pagination buttons work on all chapter pages
- [x] T023 [US2] Test mobile hamburger menu opens and displays full navigation
- [x] T024 [US2] Navigate through Module 1 chapters (1-1 through 1-6) to verify navigation consistency
- [x] T025 [US2] Navigate through Module 4 chapters to verify navigation at end of book

**Checkpoint**: Navigation system verified and functional across all docs pages

---

## Phase 5: User Story 3 - Clean Project Structure (Priority: P2)

**Goal**: Verify and document project structure with Docusaurus at root and backend/ separated

**Independent Test**: Run `npm run build` and `cd backend && python main.py` - both should work without errors

### Implementation for User Story 3

- [x] T026 [US3] Verify Docusaurus files remain at root level (src/, docs/, static/, package.json)
- [x] T027 [US3] Verify backend/ directory contains all backend code (main.py, src/, requirements.txt)
- [x] T028 [US3] Verify documentation directories preserved (.specify/, specs/, history/)
- [x] T029 [US3] Run `npm run build` to confirm frontend builds successfully
- [x] T030 [US3] Verify backend starts correctly with `cd backend && python main.py`
- [x] T031 [US3] Document final project structure in README.md if not already documented

**Checkpoint**: Project structure verified and documented

---

## Phase 6: User Story 4 - Educational UI Components (Priority: P2)

**Goal**: Enhance module cards with icons, descriptions, and consistent styling

**Independent Test**: View module cards on homepage, verify each has icon, title, subtitle, description, and chapter count with consistent styling

### Implementation for User Story 4

- [x] T032 [P] [US4] Add emoji icons to each module in src/pages/index.js module data (🤖, 🌐, 🧠, 🎯)
- [x] T033 [P] [US4] Add module descriptions to each module in src/pages/index.js module data
- [x] T034 [US4] Style module card icon display in src/components/ModuleCard/styles.module.css
- [x] T035 [US4] Add chapter count badge to ModuleCard in src/components/ModuleCard/index.js
- [x] T036 [US4] Add subtle gradient background to module cards based on module color
- [x] T037 [US4] Verify visual consistency across all 4 module cards (spacing, typography, colors)

**Checkpoint**: Educational UI components complete with enhanced visual design

---

## Phase 7: User Story 5 - Removal of Irrelevant Elements (Priority: P3)

**Goal**: Clean up footer links and remove any irrelevant UI elements

**Independent Test**: Audit footer and navbar, verify no broken/irrelevant links, no console errors

### Implementation for User Story 5

- [x] T038 [US5] Audit docusaurus.config.js footer links for relevance
- [x] T039 [US5] Remove Stack Overflow link from footer in docusaurus.config.js (Docusaurus default)
- [x] T040 [US5] Remove Discord link from footer in docusaurus.config.js (Docusaurus default)
- [x] T041 [US5] Update footer "Docs" link to point to /docs/index in docusaurus.config.js
- [x] T042 [US5] Verify navbar items are all functional and relevant
- [x] T043 [US5] Check browser console for JavaScript errors on homepage
- [x] T044 [US5] Check browser console for JavaScript errors on a chapter page
- [x] T045 [US5] Run `npm run build` to verify no warnings about removed elements

**Checkpoint**: UI cleanup complete - no irrelevant elements remain

---

## Phase 8: Polish & Validation

**Purpose**: Final validation across all user stories

- [x] T046 Run full build validation with `npm run build`
- [x] T047 [P] Test all routes from quickstart.md route testing table
- [x] T048 [P] Verify mobile responsiveness on all key pages (homepage, chapter, signin)
- [x] T049 Check for console errors across 5+ different pages
- [x] T050 [P] Verify all documentation files preserved in .specify/, specs/, history/
- [x] T051 Update specs/005-ui-ux-structure/quickstart.md with any new testing notes

---

## Dependencies & Execution Order

### Phase Dependencies

| Phase | Depends On | Blocks |
|-------|------------|--------|
| Setup (1) | None | Foundational |
| Foundational (2) | Setup | All User Stories |
| User Stories (3-7) | Foundational | Polish |
| Polish (8) | User Stories | Release |

### User Story Dependencies

| Story | Priority | Can Start After | Dependencies |
|-------|----------|-----------------|--------------|
| US1 | P1 | Foundational (Phase 2) | None (independent) |
| US2 | P1 | Foundational (Phase 2) | None (independent) |
| US3 | P2 | Foundational (Phase 2) | None (independent) |
| US4 | P2 | US1 (needs module cards) | US1 partial |
| US5 | P3 | Foundational (Phase 2) | None (independent) |

### Within Each User Story

1. Components/styles can be created in parallel [P]
2. Integration (index.js) after components ready
3. Verification after implementation
4. Story complete when checkpoint passes

---

## Parallel Opportunities

### Within Phase 2 (Foundational)
```bash
# Can run in parallel (different CSS sections):
Task T005: Color palette variables
Task T006: Spacing scale variables
Task T007: Border-radius and shadow variables
Task T008: Dark mode verification
```

### Within User Story 1
```bash
# Can run in parallel (different files):
Task T010: HomepageHero/index.js
Task T011: HomepageHero/styles.module.css
Task T012: ModuleCard/index.js
Task T013: ModuleCard/styles.module.css
```

### Between User Stories
```bash
# Can run in parallel after Foundational complete:
US1: Homepage Enhancement (Phase 3)
US2: Navigation Verification (Phase 4)
US3: Structure Verification (Phase 5)
US5: Footer Cleanup (Phase 7)

# US4 should wait for US1 module cards to exist
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (design system)
3. Complete Phase 3: User Story 1 (homepage)
4. **STOP and VALIDATE**: Test homepage independently
5. Demo if ready - homepage with hero + module cards

### Incremental Delivery

1. Complete Setup + Foundational → Design system ready
2. Add User Story 1 → Test → Deploy/Demo (MVP!)
3. Add User Story 2 → Test → Verify navigation
4. Add User Story 3 → Test → Structure documented
5. Add User Story 4 → Test → Enhanced visuals
6. Add User Story 5 → Test → Cleanup complete
7. Add Polish → Final validation

---

## File Paths Summary

| Component | Path |
|-----------|------|
| Custom CSS | src/css/custom.css |
| Homepage | src/pages/index.js |
| HomepageHero Component | src/components/HomepageHero/index.js |
| HomepageHero Styles | src/components/HomepageHero/styles.module.css |
| ModuleCard Component | src/components/ModuleCard/index.js |
| ModuleCard Styles | src/components/ModuleCard/styles.module.css |
| Docusaurus Config | docusaurus.config.js |
| Sidebars | sidebars.js (no changes) |

---

## Total Task Count

| Phase | Tasks |
|-------|-------|
| Phase 1: Setup | 4 |
| Phase 2: Foundational | 5 |
| Phase 3: US1 (Homepage) | 10 |
| Phase 4: US2 (Navigation) | 6 |
| Phase 5: US3 (Structure) | 6 |
| Phase 6: US4 (Educational UI) | 6 |
| Phase 7: US5 (Cleanup) | 8 |
| Phase 8: Polish | 6 |
| **Total** | **51 tasks** |

**MVP Scope**: Phases 1-3 (19 tasks) - Homepage with hero and module cards
