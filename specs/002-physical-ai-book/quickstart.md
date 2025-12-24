# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Overview
This guide provides a quick introduction to setting up and working with the Physical AI & Humanoid Robotics book project. This book is built using Docusaurus and deployed to GitHub Pages, following spec-driven development principles.

## Prerequisites
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git
- GitHub account for deployment

## Setup Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Start Development Server
```bash
npm start
```
This command starts a local development server and opens the book in your browser. Most changes are reflected live without restarting the server.

### 4. Build for Production
```bash
npm run build
```
This command generates static content in the `build` directory, which can be served using any static hosting service.

## Project Structure
```
physical-ai-humanoid-robotics-book/
├── docs/
│   ├── module-1-robotic-nervous-system/
│   ├── module-2-digital-twin/
│   ├── module-3-ai-robot-brain/
│   ├── module-4-vision-language-action/
│   └── index.md
├── src/
├── static/
│   └── img/
├── docusaurus.config.js
├── sidebars.js
└── package.json
```

## Adding New Content

### 1. Create a New Chapter
Create a new markdown file in the appropriate module directory:
```bash
# Example: Adding a chapter to Module 1
touch docs/module-1-robotic-nervous-system/new-chapter.md
```

### 2. Follow the Chapter Template
Each chapter should follow the standardized template:

```md
---
id: chapter-1-2-nodes-topics-services
title: Nodes, Topics, Services, and Actions
sidebar_label: Nodes, Topics, Services, and Actions
---

## Goal
[To be filled with the main goal of the chapter]

## Learning Objectives
- [To be filled with 3-5 specific learning objectives]
- [Students will be able to understand/practice specific concepts]

## Overview
[To be filled with a brief overview of the chapter content]

## Key Concepts
- [To be filled with main theoretical concepts]
- [Core principles and terminology]

## Step-by-Step Breakdown
[To be filled with detailed implementation steps]

## Code Examples
[Placeholder for practical code examples]

## Diagrams
[Placeholder for visual diagrams and illustrations]

## Case Study
[Optional - real-world application example]

## References
[Academic and practical references]

## Review Questions
[3-5 questions to test understanding]

## Practical Exercises
[Hands-on exercises for students to complete]
```

### 3. Update Sidebars
Add the new chapter to the `sidebars.js` file to make it appear in the navigation:

```js
module.exports = {
  book: [
    'index',
    {
      type: 'category',
      label: 'Module 1 — The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-robotic-nervous-system/chapter-1-1-introduction-ros2',
        'module-1-robotic-nervous-system/chapter-1-2-nodes-topics-services', // New chapter
        // ... other chapters
      ],
    },
    // ... other modules
  ],
};
```

## Deployment to GitHub Pages

### 1. Configure GitHub Pages
Ensure your `docusaurus.config.js` has the correct GitHub Pages configuration:

```js
module.exports = {
  // ... other config
  organizationName: 'your-org', // GitHub organization/username
  projectName: 'physical-ai-humanoid-robotics-book', // GitHub repository name
  deploymentBranch: 'gh-pages', // Branch to deploy to
  // ... rest of config
};
```

### 2. Deploy Command
```bash
npm run deploy
```
This command builds the site and deploys it to the `gh-pages` branch.

### 3. GitHub Actions (Recommended)
Set up automatic deployment using GitHub Actions. Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm
      - name: Install dependencies
        run: npm install
      - name: Build website
        run: npm run build
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

## Content Guidelines

### Writing Style
- Write in clear, concise, instructional language
- Target audience: Beginner-friendly to intermediate technical learners
- Maintain Flesch-Kincaid Grade 8-10 readability level
- Use active voice wherever possible

### Technical Content
- Verify all code examples and technical procedures
- Use official documentation as primary sources
- Include step-by-step instructions for setup and configuration
- Provide context for each technical concept

### Chapter Structure
All chapters must follow the standardized template to ensure consistency:
- Goal and learning objectives
- Overview and key concepts
- Step-by-step breakdown
- Code examples and diagrams
- References and exercises

## Testing and Validation

### Local Testing
- Run `npm start` to preview changes locally
- Verify all links work correctly
- Check that all code examples are syntactically correct
- Test navigation and search functionality

### Build Validation
- Run `npm run build` to ensure the site builds without errors
- Check for any warnings during the build process
- Verify the built site functions correctly

## Troubleshooting

### Common Issues
1. **Build errors**: Check for syntax errors in markdown files
2. **Missing navigation**: Ensure new files are added to sidebars.js
3. **Broken links**: Use relative paths for internal links
4. **Image issues**: Place images in `/static/img/` and reference with absolute paths

### Getting Help
- Check the detailed specifications in `/specs/002-physical-ai-book/spec.md`
- Review the implementation plan in `/specs/002-physical-ai-book/plan.md`
- Contact the development team if issues persist