# Quickstart: Personalized Docusaurus Book with Auth

**Feature**: Personalized Docusaurus Book Platform with Auth & RAG
**Date**: 2025-12-24

This guide covers setting up authentication and personalization for your Docusaurus-based book.

## Prerequisites

- Node.js 18+ and npm
- Python 3.11+ and pip
- Git

## Backend Setup (FastAPI + Better Auth)

### 1. Create Python Virtual Environment

```bash
cd backend
python -m venv venv
# Windows
venv\Scripts\activate
# Linux/macOS
source venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install fastapi uvicorn python-dotenv pydantic
pip install better-auth better-sqlite3-adapter
pip install qdrant-client sentence-transformers openai
```

### 3. Configure Environment

Create `.env` file in `backend/`:

```env
# Auth
AUTH_SECRET=your-super-secret-key-min-32-chars
JWT_SECRET=your-jwt-secret-key

# Database
DATABASE_URL=sqlite:///./data/auth.db

# RAG (existing)
QDRANT_URL=https://your-qdrant-instance.io
QDRANT_API_KEY=your-qdrant-key
OPENROUTER_API_KEY=your-openrouter-key

# Frontend URL (for CORS)
FRONTEND_URL=https://yourusername.github.io/book
```

### 4. Initialize Database

```bash
python -m scripts.init_db
```

This creates `data/auth.db` with user and session tables.

### 5. Run Backend Server

```bash
python main.py
# Server runs on http://localhost:8000
```

## Frontend Setup (Docusaurus)

### 1. Install Docusaurus Dependencies

```bash
cd frontend  # or root if Docusaurus is at root
npm install
```

### 2. Configure CORS in Docusaurus

Docusaurus doesn't need backend CORS config since it's static. The backend's CORS must allow your GitHub Pages domain.

### 3. Create Auth Components

Create `src/components/AuthButton/index.jsx`:

```jsx
import React, { useState, useEffect } from 'react';

export default function AuthButton() {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check auth status
    fetch('http://localhost:8000/auth/me', {
      credentials: 'include',
    })
      .then(res => res.ok ? res.json() : null)
      .then(data => {
        setUser(data?.user || null);
        setLoading(false);
      })
      .catch(() => setLoading(false));
  }, []);

  if (loading) return <span>Loading...</span>;

  if (user) {
    return (
      <div className="auth-menu">
        <span>Welcome, {user.email}</span>
        <button onClick={() => signOut()}>Sign Out</button>
      </div>
    );
  }

  return (
    <div className="auth-menu">
      <a href="/signup">Sign Up</a>
      <a href="/signin">Sign In</a>
    </div>
  );
}

async function signOut() {
  await fetch('http://localhost:8000/auth/signout', {
    method: 'POST',
    credentials: 'include',
  });
  window.location.reload();
}
```

### 4. Create Signup Page

Create `src/pages/signup.jsx`:

```jsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';

export default function Signup() {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirm_password: '',
    software_level: 'Beginner',
    hardware_experience: 'None',
    learning_depth: 'Both',
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    if (formData.password !== formData.confirm_password) {
      setError('Passwords do not match');
      setLoading(false);
      return;
    }

    try {
      const res = await fetch('http://localhost:8000/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(formData),
      });

      const data = await res.json();

      if (!res.ok) {
        throw new Error(data.error || 'Signup failed');
      }

      // Redirect to book homepage
      window.location.href = '/';
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign Up">
      <div className="container" style={{ maxWidth: '500px', padding: '2rem' }}>
        <h1>Create Your Account</h1>
        <p>Get personalized content based on your experience level.</p>

        {error && (
          <div className="alert alert--danger">{error}</div>
        )}

        <form onSubmit={handleSubmit}>
          <fieldset>
            <legend>Account Information</legend>

            <label>
              Email
              <input
                type="email"
                required
                value={formData.email}
                onChange={e => setFormData({...formData, email: e.target.value})}
              />
            </label>

            <label>
              Password
              <input
                type="password"
                required
                minLength={8}
                value={formData.password}
                onChange={e => setFormData({...formData, password: e.target.value})}
              />
            </label>

            <label>
              Confirm Password
              <input
                type="password"
                required
                value={formData.confirm_password}
                onChange={e => setFormData({...formData, confirm_password: e.target.value})}
              />
            </label>
          </fieldset>

          <fieldset>
            <legend>Your Background</legend>

            <label>
              Software Level
              <select
                value={formData.software_level}
                onChange={e => setFormData({...formData, software_level: e.target.value})}
              >
                <option value="Beginner">Beginner - New to programming</option>
                <option value="Intermediate">Intermediate - Comfortable with code</option>
                <option value="Advanced">Advanced - Professional developer</option>
              </select>
            </label>

            <label>
              Hardware/Robotics Experience
              <select
                value={formData.hardware_experience}
                onChange={e => setFormData({...formData, hardware_experience: e.target.value})}
              >
                <option value="None">None - New to robotics</option>
                <option value="Basic">Basic - Some theoretical knowledge</option>
                <option value="Hands-on">Hands-on - Built robots before</option>
              </select>
            </label>

            <label>
              Learning Depth Preference
              <select
                value={formData.learning_depth}
                onChange={e => setFormData({...formData, learning_depth: e.target.value})}
              >
                <option value="Conceptual">Conceptual - Focus on theory</option>
                <option value="Practical">Practical - Focus on implementation</option>
                <option value="Both">Both - Balance of theory and practice</option>
              </select>
            </label>
          </fieldset>

          <button type="submit" disabled={loading}>
            {loading ? 'Creating Account...' : 'Create Account'}
          </button>
        </form>

        <p style={{ marginTop: '1rem' }}>
          Already have an account? <a href="/signin">Sign In</a>
        </p>
      </div>
    </Layout>
  );
}
```

### 5. Create Personalization Toggle

Create `src/components/PersonalizeButton/index.jsx`:

```jsx
import React, { useState, useEffect } from 'react';

export default function PersonalizeButton({ chapterId }) {
  const [enabled, setEnabled] = useState(false);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    // Load saved preference
    const saved = localStorage.getItem(`personalize_${chapterId}`);
    setEnabled(saved === 'true');
  }, [chapterId]);

  const togglePersonalization = async () => {
    setLoading(true);
    const newValue = !enabled;

    try {
      const res = await fetch(`http://localhost:8000/api/personalize/${chapterId}`, {
        method: 'POST',
        credentials: 'include',
      });

      if (res.ok) {
        setEnabled(newValue);
        localStorage.setItem(`personalize_${chapterId}`, String(newValue));

        // Reload page to apply changes
        if (newValue) {
          window.location.reload();
        }
      }
    } catch (err) {
      console.error('Personalization failed:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <button
      className={`button button--${enabled ? 'primary' : 'secondary'}`}
      onClick={togglePersonalization}
      disabled={loading}
    >
      {loading ? 'Applying...' : enabled ? '✓ Personalized' : 'Personalize Content'}
    </button>
  );
}
```

## Testing Locally

### 1. Start Backend

```bash
cd backend
python main.py
# Runs on http://localhost:8000
```

### 2. Start Frontend

```bash
npm start
# Runs on http://localhost:3000
```

### 3. Test Flow

1. Visit `http://localhost:3000`
2. Click "Sign Up" → Create account with profile
3. Visit any chapter
4. Click "Personalize Content"
5. Verify content adapts to your level
6. Test chatbot with personalized responses

## Deployment

### GitHub Pages (Frontend)

1. Update `docusaurus.config.js`:

```js
const config = {
  url: 'https://yourusername.github.io',
  baseUrl: '/book/',
  // ... rest of config
};
```

2. Build and deploy:

```bash
npm run build
npx docusaurus deploy
```

### Backend Deployment

Deploy to your preferred provider (Railway, Render, Fly.io):

1. Set environment variables
2. Use PostgreSQL for production
3. Update CORS to allow your GitHub Pages domain

## Verification Checklist

- [ ] Backend runs without errors
- [ ] Signup creates user with profile
- [ ] Signin returns HttpOnly cookie
- [ ] Profile page shows user data
- [ ] Personalize button toggles content
- [ ] Chatbot responds with user-aware context
- [ ] Logout clears session
- [ ] Local development works end-to-end
- [ ] Production build deploys successfully

## Common Issues

### CORS Errors

```
Access to fetch blocked by CORS policy
```

**Fix**: Ensure backend CORS includes your frontend URL:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Dev
    allow_origins=["https://username.github.io"],  # Prod
    allow_credentials=True,
)
```

### Auth Cookie Not Sent

**Fix**: Ensure `credentials: 'include'` is set on all fetch requests.

### Session Not Persisting

**Fix**: Check cookie settings match between client and server.

## Next Steps

1. Complete `/sp.tasks` to generate implementation tasks
2. Implement backend auth endpoints
3. Create Docusaurus auth components
4. Integrate personalization with chapter content
5. Extend RAG chatbot with user context
6. Test all flows
7. Deploy to production
