# Hugging Face Deployment Guide

## Complete Steps to Deploy Backend on Hugging Face Spaces

### Step 1: Create Hugging Face Account
1. Go to https://huggingface.co
2. Sign up with your GitHub account
3. Verify your email

### Step 2: Create New Space
1. Go to https://huggingface.co/new-space
2. Fill in the details:
   - **Owner**: Your username
   - **Space name**: `physical-ai-backend`
   - **SDK**: Select **Docker** (very important!)
   - **Hardware**: CPU (free tier)
   - **Space visibility**: Public
3. Click "Create Space"

### Step 3: Link GitHub Repository
1. In your Space, go to **Settings** tab
2. Find **Repository** section
3. Click **"Link a repository"**
4. Search and select: `Aimzaa/physical-ai-humanoid-robotics-book`
5. Click **"Link Repository"**

### Step 4: Configure Dockerfile Path
In Space Settings:
- **Dockerfile path**: `backend/Dockerfile`
- **Autobuild**: ✅ Enable this

### Step 5: Add Environment Variables
In Space Settings, go to **"Variables and secrets"** and add:

| Variable Name | Value | Secret? |
|--------------|-------|---------|
| `QDRANT_URL` | Your Qdrant Cloud URL | ✅ Yes |
| `QDRANT_API_KEY` | Your Qdrant API Key | ✅ Yes |
| `OPENROUTER_API_KEY` | Your OpenRouter API Key | ✅ Yes |

**Where to get these keys:**

1. **Qdrant Cloud** (for vector database):
   - Go to https://cloud.qdrant.io
   - Sign up and create a free cluster
   - Copy the URL and API key from your cluster

2. **OpenRouter** (for LLM responses):
   - Go to https://openrouter.ai
   - Sign up and get a free API key
   - Key starts with `sk-or-v1-`

### Step 6: Wait for Build
- Hugging Face will automatically build your Docker image
- This takes 2-5 minutes first time
- Watch the "Build logs" tab for progress

### Step 7: Test Your API
Once deployed, your API will be at:
```
https://your-username-physical-ai-backend.hf.space
```

**Test endpoints:**
```bash
# Health check
curl https://your-username-physical-ai-backend.hf.space/health

# API docs
# Visit: https://your-username-physical-ai-backend.hf.space/docs
```

---

## Troubleshooting

### Build Fails
1. Check "Build logs" tab for errors
2. Common issues:
   - Missing `backend/requirements.txt`
   - Invalid Dockerfile syntax
   - Missing dependencies

### API Returns 500 Error
1. Check environment variables are set correctly
2. Verify Qdrant cluster is running
3. Check API keys are valid

### Port Issues
- Hugging Face uses port 7860 by default
- Our Dockerfile is configured for this

---

## Files Created for Deployment

| File | Purpose |
|------|---------|
| `README.md` | HF Space metadata (title, tags, colors) |
| `backend/Dockerfile` | Docker image configuration |
| `backend/.env.example` | Environment variables template |
| `.huggingface/spaces.yaml` | HF Spaces configuration |

---

## Frontend Connection

Once backend is deployed, update frontend to use the new API URL:

In `src/plugins/ChatbotPlugin/index.js`:
```javascript
const API_URL = "https://your-username-physical-ai-backend.hf.space";
```

Or set `REACT_APP_API_URL` environment variable.
