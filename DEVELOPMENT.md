# Development Guide: Physical AI & Humanoid Robotics Book

This guide provides instructions for setting up a local development environment and contributing to the Physical AI & Humanoid Robotics Book project.

## Prerequisites

- Node.js (v18 or higher)
- Python (v3.8 or higher)
- Git
- Access to OpenRouter API key
- Qdrant vector database (local or cloud)

## Local Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd humanoid-robotics-book
```

### 2. Frontend Setup (Docusaurus)
```bash
# Install Node.js dependencies
npm install

# Start the development server
npm start
```

The site will be available at http://localhost:3000/physical-ai-humanoid-robotics-book/

### 3. Backend Setup (RAG Service)
```bash
# Navigate to backend directory
cd backend

# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install -r requirements.txt
```

### 4. Environment Variables
Create a `.env` file in the `backend/` directory:
```env
QDRANT_URL=http://localhost:6333  # For local development
# QDRANT_URL=your_cloud_url  # For cloud deployment
# QDRANT_API_KEY=your_api_key  # If using cloud Qdrant
OPENROUTER_API_KEY=your_openrouter_api_key
```

### 5. Start Qdrant (Local Development)
For local development, you can run Qdrant using Docker:
```bash
docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant
```

### 6. Load Book Content to Vector Store
```bash
cd backend
python load_embeddings.py
```

### 7. Start the Backend Service
```bash
cd backend
uvicorn main:app --reload --port 8000
```

## Development Workflow

### Frontend Development
1. Make changes to markdown files in the `docs/` directory
2. The development server automatically reloads changes
3. Test the chatbot widget functionality

### Backend Development
1. Make changes to the FastAPI service in `backend/main.py`
2. The service auto-reloads with `--reload` flag
3. Test API endpoints using the development server

### Content Development
1. Follow the chapter template in `docs/chapter-template.md`
2. Ensure content follows the required structure:
   - Chapter Title
   - Goal
   - Learning Objectives
   - Overview
   - Key Concepts
   - Step-by-Step Breakdown
   - Code Examples
   - Diagrams
   - References
   - Review Questions
   - Practical Exercises

## Project Structure

```
├── docs/                           # Book content
│   ├── module-1-robotic-nervous-system/    # Module 1 chapters
│   ├── module-2-digital-twin/              # Module 2 chapters
│   ├── module-3-ai-robot-brain/            # Module 3 chapters
│   ├── module-4-vision-language-action/    # Module 4 chapters
│   ├── capstone-project/                   # Capstone chapters
│   ├── index.md                          # Book introduction
│   └── ...
├── backend/                        # RAG service
│   ├── main.py                     # FastAPI application
│   ├── process_content.py          # Content processing
│   ├── load_embeddings.py          # Embedding loading
│   └── requirements.txt            # Python dependencies
├── src/                           # Docusaurus custom components
│   ├── components/ChatbotWidget/   # Chatbot React component
│   └── plugins/ChatbotPlugin/      # Docusaurus plugin
├── static/                        # Static assets
├── specs/                         # Project specifications
├── history/                       # Project history and validation
├── docusaurus.config.js           # Docusaurus configuration
├── sidebars.js                    # Navigation configuration
└── package.json                   # Node.js dependencies
```

## API Endpoints

### Backend (http://localhost:8000)
- `GET /` - Health check
- `GET /health` - Service health
- `POST /embeddings` - Add documents to vector store
- `POST /search` - Search for relevant content
- `POST /chat` - RAG chat functionality

### Testing Endpoints
```bash
# Test health
curl http://localhost:8000/health

# Test search
curl -X POST "http://localhost:8000/search" \
  -H "Content-Type: application/json" \
  -d '{"query": "ROS 2 architecture", "top_k": 3}'

# Test chat
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{"query": "Explain ROS 2 nodes"}'
```

## Content Guidelines

### Writing Style
- Maintain Grade 8-10 readability
- Use clear, concise language
- Include practical examples
- Verify all technical content against official documentation
- Include code examples with proper syntax highlighting

### Chapter Structure
Follow the template in `docs/chapter-template.md`:
- Clear learning objectives
- Step-by-step breakdowns
- Practical exercises
- Review questions
- Proper referencing

### Code Examples
- Test all code examples before adding
- Use proper syntax highlighting
- Include comments explaining complex concepts
- Follow best practices for the respective technology

## Testing

### Frontend Testing
1. Verify all pages render correctly
2. Test navigation and search functionality
3. Check chatbot widget integration
4. Test text selection context feature

### Backend Testing
1. Test all API endpoints
2. Verify embedding loading process
3. Test retrieval accuracy
4. Validate response quality

### Content Testing
1. Verify all links and references
2. Check readability metrics
3. Validate against official documentation
4. Ensure 40%+ official documentation references

## Common Tasks

### Adding New Content
1. Create a new markdown file following the template
2. Add it to the appropriate module directory
3. Update `sidebars.js` to include the new content in navigation
4. Rebuild the site to test

### Updating RAG Content
1. Add or modify content in the `docs/` directory
2. Run the embedding loader again:
   ```bash
   cd backend
   python load_embeddings.py
   ```

### Customizing the Chatbot
1. Modify the component in `src/components/ChatbotWidget/`
2. Adjust styling in the CSS file
3. Update API integration as needed

## Troubleshooting

### Frontend Issues
- If the site doesn't build: Run `npm run build` to see detailed errors
- If chatbot doesn't appear: Check browser console for errors
- If navigation doesn't work: Verify `sidebars.js` configuration

### Backend Issues
- If Qdrant connection fails: Verify URL and API key
- If embeddings don't load: Check content processing script
- If API calls fail: Verify OpenRouter API key

### Common Fixes
```bash
# Clean Docusaurus cache
npx docusaurus clear

# Clean npm cache
npm cache clean --force

# Reinstall dependencies
rm -rf node_modules package-lock.json
npm install

# For backend, recreate virtual environment
rm -rf venv
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r backend/requirements.txt
```

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes
4. Test thoroughly
5. Commit your changes: `git commit -m 'Add feature'`
6. Push to the branch: `git push origin feature-name`
7. Submit a pull request

## Performance Tips

### For Faster Development
- Use `npm start` for development (hot reloading)
- Use `--reload` flag with uvicorn for backend auto-reload
- Run Qdrant locally for faster response times during development

### For Production
- Optimize images and assets
- Minimize embedding size for faster retrieval
- Implement caching for common queries
- Monitor and optimize API response times

## Resources

- [Docusaurus Documentation](https://docusaurus.io/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [OpenRouter API](https://openrouter.ai/docs)
- [Original Project Specification](specs/002-physical-ai-book/spec.md)
- [Implementation Plan](specs/002-physical-ai-book/plan.md)