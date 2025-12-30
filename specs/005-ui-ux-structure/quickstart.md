# Quickstart: UI/UX Enhancement and Project Structure Finalization

**Feature Branch**: `005-ui-ux-structure`
**Date**: 2025-12-29

## Prerequisites

- Node.js 18+ installed
- Git repository cloned
- On feature branch `005-ui-ux-structure`

## Quick Setup

```bash
# 1. Install dependencies (if not already done)
npm install

# 2. Start development server
npm start

# 3. Open browser to http://localhost:3000
```

## Development Workflow

### Running the Frontend

```bash
# Development mode with hot reload
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

### Running the Backend (if needed)

```bash
# In a separate terminal
cd backend
pip install -r requirements.txt
python main.py

# Backend runs at http://localhost:8000
```

## Testing Changes

### Manual Testing Checklist

#### Homepage (US1)
- [ ] Hero section displays with title and subtitle
- [ ] "Start Learning" button links to `/docs/index`
- [ ] 4 module cards visible below hero
- [ ] Module cards are clickable and navigate correctly
- [ ] Responsive layout on mobile (stack cards)

#### Navigation (US2)
- [ ] Sidebar shows all chapters
- [ ] Current chapter is highlighted
- [ ] Breadcrumbs display correctly
- [ ] Previous/Next buttons work
- [ ] Mobile hamburger menu works

#### Visual Design (US4)
- [ ] Color scheme is consistent (blues/grays)
- [ ] Typography is consistent
- [ ] Module cards have icons and descriptions
- [ ] Dark mode still works (if applicable)

#### Cleanup (US5)
- [ ] No broken links in footer
- [ ] No console errors on any page
- [ ] All navigation elements are functional

### Build Validation

```bash
# Run production build - must complete without errors
npm run build

# Check for warnings
# Expected: 0 errors, minimal warnings
```

### Route Testing

After changes, verify these routes work:

| Route | Expected Content |
|-------|------------------|
| `/` | Homepage with hero + module cards |
| `/docs/index` | Book introduction |
| `/docs/module-1-robotic-nervous-system/chapter-1-1-introduction-ros2` | Module 1, Chapter 1 |
| `/docs/module-2-digital-twin/chapter-2-1-physics-simulation` | Module 2, Chapter 1 |
| `/docs/module-3-ai-robot-brain/chapter-3-1-nvidia-isaac-platform` | Module 3, Chapter 1 |
| `/docs/module-4-vision-language-action/chapter-4-1-vla-systems` | Module 4, Chapter 1 |
| `/docs/capstone-project/autonomous-humanoid-implementation` | Capstone |
| `/signin` | Sign in page |
| `/signup` | Sign up page |
| `/settings` | Settings page (requires auth) |

## File Locations

### Files to Create

| File | Purpose |
|------|---------|
| `src/pages/index.js` | Custom homepage |
| `src/components/HomepageHero/index.js` | Hero section component |
| `src/components/HomepageHero/styles.module.css` | Hero styles |
| `src/components/ModuleCard/index.js` | Module card component |
| `src/components/ModuleCard/styles.module.css` | Module card styles |

### Files to Modify

| File | Changes |
|------|---------|
| `src/css/custom.css` | Add design system variables, color palette |
| `docusaurus.config.js` | Clean up footer links |

### Files to Preserve (Do Not Modify)

- All `docs/**/*.md` files
- `sidebars.js`
- `backend/**/*`
- `src/components/AuthButton/**`
- `src/components/ChatbotWidget/**`
- `src/components/PersonalizeButton/**`
- `src/components/TranslationToggle/**`
- `src/theme/MDXContent/**`
- `src/plugins/**`

## Common Issues

### "Module not found" Error
- Run `npm install` to ensure all dependencies are installed
- Check import paths use correct relative paths

### CSS Not Applying
- Ensure CSS module imports use `.module.css` extension
- Check class names match between JS and CSS

### Build Fails
- Clear `.docusaurus` cache: `rm -rf .docusaurus`
- Reinstall node_modules: `rm -rf node_modules && npm install`

### Hot Reload Not Working
- Restart dev server: `Ctrl+C` then `npm start`
- Check for syntax errors in modified files

## Success Criteria Validation

```bash
# 1. Build must succeed
npm run build

# 2. No console errors (check browser DevTools)
# Open http://localhost:3000 and check Console tab

# 3. All routes accessible
# Manually visit each route in testing checklist

# 4. Mobile responsive
# Use browser DevTools to test mobile viewport (375px width)
```

## Next Steps After Implementation

1. Run `npm run build` to validate
2. Test all routes manually
3. Check mobile responsiveness
4. Review console for errors
5. Create PR with `/sp.git.commit_pr`
