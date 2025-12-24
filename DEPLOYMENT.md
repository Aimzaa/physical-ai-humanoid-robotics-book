# Deployment Guide: Physical AI & Humanoid Robotics Book

This guide provides instructions for deploying the Physical AI & Humanoid Robotics Book with integrated RAG chatbot to production.

## Architecture Overview

The system consists of two main components:
1. **Frontend**: Docusaurus documentation site hosted on GitHub Pages
2. **Backend**: FastAPI RAG service for AI-powered responses

## Frontend Deployment (GitHub Pages)

### Prerequisites
- GitHub repository with the book content
- GitHub Actions enabled for the repository

### Steps
1. The GitHub Actions workflow is already configured in `.github/workflows/deploy.yml`
2. When you push changes to the `main` branch, the workflow automatically builds and deploys the site to GitHub Pages
3. The site will be available at: `https://<your-username>.github.io/<repository-name>/`

### Manual Deployment
If you need to deploy manually:
```bash
npm run build
npm run deploy
```

## Backend Deployment (RAG Service)

### Option 1: Cloud Platforms (Recommended)
Deploy the FastAPI backend to a cloud platform:

#### Heroku
1. Create a Heroku account and install the Heroku CLI
2. Create a new app: `heroku create your-app-name`
3. Set environment variables:
   ```bash
   heroku config:set QDRANT_URL=your_qdrant_url
   heroku config:set QDRANT_API_KEY=your_qdrant_api_key
   heroku config:set OPENROUTER_API_KEY=your_openrouter_api_key
   ```
4. Deploy: `git push heroku main`

#### Render
1. Create a Render account
2. Create a new Web Service
3. Connect to your GitHub repository
4. Set environment variables in the dashboard
5. Configure build command: `pip install -r backend/requirements.txt`
6. Set start command: `uvicorn backend.main:app --host=0.0.0.0 --port=$PORT`

#### Railway
1. Create a Railway account
2. Import your repository
3. Set environment variables
4. Deploy automatically

### Option 2: Self-Hosting
1. Set up a server with Python 3.8+
2. Install dependencies: `pip install -r backend/requirements.txt`
3. Set environment variables
4. Run the service: `uvicorn backend.main:app --host=0.0.0.0 --port=8000`

## Environment Variables

### Frontend (Docusaurus)
Set the backend API URL as an environment variable:
```bash
REACT_APP_RAG_API_URL=https://your-backend-url.onrender.com
```

### Backend (FastAPI)
Required environment variables:
```env
QDRANT_URL=your_qdrant_instance_url
QDRANT_API_KEY=your_qdrant_api_key  # If using Qdrant Cloud
OPENROUTER_API_KEY=your_openrouter_api_key
EMBEDDING_MODEL_NAME=all-MiniLM-L6-v2  # Optional, default value
```

## Content Population

After deploying the backend, you need to populate the Qdrant vector store with book content:

1. Install backend dependencies: `pip install -r backend/requirements.txt`
2. Run the content loader:
   ```bash
   cd backend
   python load_embeddings.py
   ```
3. This will process all markdown files from `../docs` and store embeddings in Qdrant

## Frontend Configuration

Update the frontend to point to your deployed backend:

1. In your production environment, set the `REACT_APP_RAG_API_URL` environment variable to your backend URL
2. Or update the default in `src/components/ChatbotWidget/ChatbotWidget.jsx`:
   ```javascript
   const API_BASE_URL = process.env.REACT_APP_RAG_API_URL || 'https://your-backend-url.onrender.com';
   ```

## Monitoring and Maintenance

### Frontend Monitoring
- GitHub Pages provides basic monitoring
- Check site regularly for broken links or rendering issues

### Backend Monitoring
- Monitor API usage and response times
- Check logs for errors
- Monitor Qdrant storage usage (should stay under 1GB for free tier)

### Cost Management
- OpenRouter costs: Monitor token usage through their dashboard
- Qdrant costs: Monitor vector storage and query usage
- Set up billing alerts if needed

## Troubleshooting

### Frontend Issues
- If the chatbot doesn't appear, check browser console for errors
- Verify the backend URL is correct
- Check that CORS allows requests from your frontend domain

### Backend Issues
- If embedding loading fails, verify Qdrant connection
- If OpenRouter calls fail, check API key and rate limits
- Check logs for specific error messages

### Common Issues
- **CORS Errors**: Ensure backend allows requests from frontend domain
- **API Key Issues**: Verify all API keys are correctly set
- **Connection Timeouts**: Check network connectivity and service availability

## Updating Content

To update book content:
1. Edit the markdown files in the `docs/` directory
2. Commit and push changes to trigger GitHub Pages deployment
3. If content changes significantly, you may need to re-run the embedding loader

## Scaling Considerations

### For Increased Traffic
- Upgrade Qdrant plan for more queries
- Use a more robust cloud platform for the backend
- Consider caching strategies for common queries

### For Larger Content
- Monitor Qdrant storage limits
- Consider content chunking strategies
- Plan for longer embedding processing times

## Security Best Practices

- Never commit API keys to the repository
- Use environment variables for all sensitive data
- Implement rate limiting on the backend
- Regularly rotate API keys
- Monitor for unusual usage patterns

## Support and Maintenance

### Regular Tasks
- Monthly: Review usage and costs
- Quarterly: Update dependencies
- As needed: Update content and reprocess embeddings

### Contact Information
- For technical issues: Check the GitHub repository issues
- For content questions: Contact the book maintainers

## Success Metrics

Monitor these key metrics:
- Site uptime and availability
- RAG response times (should be < 3 seconds)
- User engagement with the chatbot
- Content accuracy and user satisfaction
- Cost per month for hosting and API usage

## Rollback Procedures

If deployment issues occur:
1. For frontend: Use GitHub Pages version history to rollback
2. For backend: Use your platform's rollback features
3. Always test changes in a staging environment first