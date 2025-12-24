# Claude Code Context for Physical AI & Humanoid Robotics Book

## Project Overview
The Physical AI & Humanoid Robotics book is a comprehensive educational resource built using Docusaurus and deployed to GitHub Pages. The project follows spec-driven development principles using Spec-Kit Plus.

## Key Technologies
- Docusaurus v3.x (static site generator)
- React-based documentation components
- Markdown content management
- GitHub Pages hosting
- npm/node.js build tools
- GitHub Actions for CI/CD

## Project Structure
```
physical-ai-humanoid-robotics-book/
├── docs/                     # Content files organized by modules
│   ├── module-1-robotic-nervous-system/    # Module 1 content
│   ├── module-2-digital-twin/             # Module 2 content
│   ├── module-3-ai-robot-brain/           # Module 3 content
│   ├── module-4-vision-language-action/   # Module 4 content
│   └── index.md              # Book introduction
├── src/                      # Custom React components
├── static/                   # Static assets (images, etc.)
│   └── img/                  # Book diagrams and illustrations
├── docusaurus.config.js      # Docusaurus configuration
├── sidebars.js               # Navigation structure
└── package.json              # Dependencies and scripts
```

## Content Organization
- 4 main modules with 6 chapters each (24+ total chapters)
- Each chapter follows a standardized template with consistent structure
- Content organized in module-specific folders
- Navigation hierarchy: Book → Module → Chapter

## Chapter Template Structure
Each chapter must include:
- Goal and Learning Objectives
- Overview and Key Concepts
- Step-by-Step Breakdown
- Code Examples and Diagrams
- Case Study (optional)
- References and Review Questions
- Practical Exercises

## Development Workflow
1. Create new content in appropriate module directory
2. Follow standardized chapter template
3. Add new files to sidebars.js for navigation
4. Test locally with `npm start`
5. Build and validate with `npm run build`
6. Deploy to GitHub Pages

## Important Files
- `specs/002-physical-ai-book/spec.md` - Feature specification
- `specs/002-physical-ai-book/plan.md` - Implementation plan
- `specs/002-physical-ai-book/research.md` - Research findings
- `specs/002-physical-ai-book/data-model.md` - Data model
- `specs/002-physical-ai-book/quickstart.md` - Quickstart guide
- `specs/002-physical-ai-book/contracts/book-content-api.yaml` - API contract

## Quality Standards
- Maintain 6,000-10,000 total word count
- Each chapter: 500-1,200 words
- Flesch-Kincaid Grade 8-10 readability
- All technical content verified against official documentation
- Consistent formatting and structure across all chapters
- Valid, non-broken links and references

## Deployment
- Use GitHub Actions for automated deployment
- Deploy to gh-pages branch
- Automatic build and deployment on main branch updates