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

const SOFTWARE_LEVELS = [
  { value: 'Beginner', label: 'Beginner', desc: 'New to programming' },
  { value: 'Intermediate', label: 'Intermediate', desc: 'Comfortable with code' },
  { value: 'Advanced', label: 'Advanced', desc: 'Professional developer' },
];

const HARDWARE_EXPERIENCE = [
  { value: 'None', label: 'None', desc: 'New to robotics' },
  { value: 'Basic', label: 'Basic', desc: 'Some theoretical knowledge' },
  { value: 'Hands-on', label: 'Hands-on', desc: 'Built robots before' },
];

const LEARNING_DEPTH = [
  { value: 'Conceptual', label: 'Conceptual', desc: 'Focus on theory' },
  { value: 'Practical', label: 'Practical', desc: 'Focus on implementation' },
  { value: 'Both', label: 'Both', desc: 'Balance of theory and practice' },
];

export default function Signup() {
  const [apiUrl, setApiUrl] = useState('http://localhost:8000');
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

  // Get base URLs
  const homeUrl = useBaseUrl('/');
  const signinUrl = useBaseUrl('/signin');

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

    if (formData.password !== formData.confirm_password) {
      setError('Passwords do not match');
      setLoading(false);
      return;
    }

    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      setLoading(false);
      return;
    }

    try {
      const res = await fetch(`${apiUrl}/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(formData),
      });

      const data = await res.json();

      if (!res.ok) {
        // FastAPI returns error in 'detail' field
        let errorMsg = data.detail || data.error || 'Signup failed';
        // Handle 409 Conflict (email already exists)
        if (res.status === 409) {
          errorMsg = 'An account with this email already exists. Please sign in instead.';
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
    <Layout title="Sign Up - Physical AI Book">
      <div className={styles.container}>
        <div className={styles.card}>
          <h1 className={styles.title}>Create Your Account</h1>
          <p className={styles.subtitle}>
            Get personalized content based on your experience level
          </p>

          {error && (
            <div className={styles.error}>
              ⚠️ {error}
            </div>
          )}

          <form onSubmit={handleSubmit}>
            <fieldset className={styles.fieldset}>
              <legend>Account Information</legend>

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
                  minLength={8}
                  value={formData.password}
                  onChange={handleChange}
                  className={styles.input}
                  placeholder="At least 8 characters"
                />
              </label>

              <label className={styles.label}>
                Confirm Password
                <input
                  type="password"
                  name="confirm_password"
                  required
                  value={formData.confirm_password}
                  onChange={handleChange}
                  className={styles.input}
                  placeholder="Confirm your password"
                />
              </label>
            </fieldset>

            <fieldset className={styles.fieldset}>
              <legend>Your Background</legend>
              <p className={styles.legendDesc}>
                This helps us personalize the content for you
              </p>

              <div className={styles.radioGroup}>
                <label className={styles.radioLabel}>
                  Software Level
                </label>
                {SOFTWARE_LEVELS.map((level) => (
                  <label key={level.value} className={styles.radioOption}>
                    <input
                      type="radio"
                      name="software_level"
                      value={level.value}
                      checked={formData.software_level === level.value}
                      onChange={handleChange}
                    />
                    <span className={styles.radioContent}>
                      <span className={styles.radioTitle}>{level.label}</span>
                      <span className={styles.radioDesc}>{level.desc}</span>
                    </span>
                  </label>
                ))}
              </div>

              <div className={styles.radioGroup}>
                <label className={styles.radioLabel}>
                  Hardware/Robotics Experience
                </label>
                {HARDWARE_EXPERIENCE.map((exp) => (
                  <label key={exp.value} className={styles.radioOption}>
                    <input
                      type="radio"
                      name="hardware_experience"
                      value={exp.value}
                      checked={formData.hardware_experience === exp.value}
                      onChange={handleChange}
                    />
                    <span className={styles.radioContent}>
                      <span className={styles.radioTitle}>{exp.label}</span>
                      <span className={styles.radioDesc}>{exp.desc}</span>
                    </span>
                  </label>
                ))}
              </div>

              <div className={styles.radioGroup}>
                <label className={styles.radioLabel}>
                  Learning Depth Preference
                </label>
                {LEARNING_DEPTH.map((depth) => (
                  <label key={depth.value} className={styles.radioOption}>
                    <input
                      type="radio"
                      name="learning_depth"
                      value={depth.value}
                      checked={formData.learning_depth === depth.value}
                      onChange={handleChange}
                    />
                    <span className={styles.radioContent}>
                      <span className={styles.radioTitle}>{depth.label}</span>
                      <span className={styles.radioDesc}>{depth.desc}</span>
                    </span>
                  </label>
                ))}
              </div>
            </fieldset>

            <button
              type="submit"
              className={clsx(styles.submitButton, 'button', 'button--primary')}
              disabled={loading}
            >
              {loading ? 'Creating Account...' : 'Create Account'}
            </button>
          </form>

          <p className={styles.footer}>
            Already have an account?{' '}
            <a href={signinUrl} className={styles.link}>
              Sign In
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
