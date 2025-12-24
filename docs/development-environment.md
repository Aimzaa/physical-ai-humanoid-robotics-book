---
id: development-environment
title: Development Environment Setup
sidebar_label: Development Environment
---

# Development Environment Setup

This guide provides instructions for setting up your development environment to work with the Physical AI & Humanoid Robotics book.

## Prerequisites

Before you begin, ensure you have the following installed on your system:

- **Node.js** (version 18 or higher)
- **npm** (comes with Node.js) or **yarn**
- **Git** for version control
- A modern text editor or IDE (VS Code, WebStorm, etc.)

### Installing Node.js

1. Visit [nodejs.org](https://nodejs.org/)
2. Download the LTS version (Long Term Support)
3. Run the installer and follow the prompts
4. Verify installation by opening a terminal and running:
   ```bash
   node --version
   npm --version
   ```

### Installing Git

1. Visit [git-scm.com](https://git-scm.com/)
2. Download and install Git for your operating system
3. Verify installation by running:
   ```bash
   git --version
   ```

## Project Setup

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

## Development Workflow

### Creating New Content

1. Create new markdown files in the appropriate module directory
2. Add the new file to `sidebars.js` to make it appear in the navigation
3. Use the chapter template as a starting point for consistency

### Building for Production

```bash
npm run build
```

This command generates static content in the `build` directory, which can be served using any static hosting service.

### Local Testing

- Run `npm start` to preview changes locally
- Verify all links work correctly
- Check that all code examples are syntactically correct
- Test navigation and search functionality

## Recommended Tools

### VS Code Extensions

- **MDX** - For syntax highlighting in MDX files
- **Docusaurus** - Provides snippets and syntax highlighting
- **Prettier** - For consistent code formatting
- **ESLint** - For JavaScript/TypeScript linting

### Browser Extensions

- **React Developer Tools** - For debugging React components
- **Docusaurus Browser Extension** - Provides additional development tools

## Troubleshooting

### Common Issues

1. **Build errors**: Check for syntax errors in markdown files
2. **Missing navigation**: Ensure new files are added to sidebars.js
3. **Broken links**: Use relative paths for internal links
4. **Image issues**: Place images in `/static/img/` and reference with absolute paths

### Getting Help

- Check the detailed specifications in the specs directory
- Review the implementation plan
- Contact the development team if issues persist

## Performance Optimization

- Use efficient image formats (SVG for diagrams, WebP for photos)
- Minimize the use of heavy JavaScript components
- Optimize code examples for readability and performance
- Regularly test build times and page load speeds