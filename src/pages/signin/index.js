import React, { useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

// Get API URL - works in both browser and SSR
function getApiUrl() {
  if (typeof window !== 'undefined') {
    const isProduction = window.location.hostname !== 'localhost';
    return isProduction ? 'https://your-backend-domain.com' : 'http://localhost:8000';
  }
  return 'http://localhost:8000';
}

export default function Signin() {
  const [apiUrl, setApiUrl] = useState('http://localhost:8000');
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  // Get base URLs
  const homeUrl = useBaseUrl('/');
  const signupUrl = useBaseUrl('/signup');

  React.useEffect(() => {
    setApiUrl(getApiUrl());
  }, []);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    setError('');
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const res = await fetch(`${apiUrl}/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(formData),
      });

      const data = await res.json();

      if (!res.ok) {
        // FastAPI returns error in 'detail' field
        let errorMsg = data.detail || data.error || 'Sign in failed';
        // Handle 401 Unauthorized
        if (res.status === 401) {
          errorMsg = 'Invalid email or password. Please try again.';
        }
        // Handle validation errors (array of details)
        if (Array.isArray(data.detail)) {
          errorMsg = data.detail.map(d => d.msg || d.message).join(', ');
        }
        throw new Error(errorMsg);
      }

      // Redirect to homepage with proper baseUrl
      window.location.href = homeUrl;
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign In - Physical AI Book">
      <div className={styles.container}>
        <div className={styles.card}>
          <h1 className={styles.title}>Welcome Back</h1>
          <p className={styles.subtitle}>
            Sign in to access your personalized content
          </p>

          {error && (
            <div className={styles.error}>
              ⚠️ {error}
            </div>
          )}

          <form onSubmit={handleSubmit}>
            <label className={styles.label}>
              Email
              <input
                type="email"
                name="email"
                required
                value={formData.email}
                onChange={handleChange}
                className={styles.input}
                placeholder="you@example.com"
              />
            </label>

            <label className={styles.label}>
              Password
              <input
                type="password"
                name="password"
                required
                value={formData.password}
                onChange={handleChange}
                className={styles.input}
                placeholder="Your password"
              />
            </label>

            <button
              type="submit"
              className={clsx(styles.submitButton, 'button', 'button--primary')}
              disabled={loading}
            >
              {loading ? 'Signing In...' : 'Sign In'}
            </button>
          </form>

          <p className={styles.footer}>
            Don't have an account?{' '}
            <a href={signupUrl} className={styles.link}>
              Sign Up
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
