import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
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

export default function Settings() {
  const [apiUrl, setApiUrl] = useState('http://localhost:8000');
  const [profile, setProfile] = useState(null);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState({ type: '', text: '' });

  useEffect(() => {
    setApiUrl(getApiUrl());
    fetchProfile();
  }, []);

  const fetchProfile = async () => {
    try {
      const res = await fetch(`${apiUrl}/api/profile`, {
        credentials: 'include',
      });

      if (!res.ok) {
        if (res.status === 401) {
          window.location.href = '/signin';
          return;
        }
        throw new Error('Failed to fetch profile');
      }

      const data = await res.json();
      setProfile(data.profile);
    } catch (error) {
      setMessage({ type: 'error', text: 'Failed to load profile' });
    } finally {
      setLoading(false);
    }
  };

  const handleSave = async (field, value) => {
    setSaving(true);
    setMessage({ type: '', text: '' });

    try {
      const res = await fetch(`${apiUrl}/api/profile`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ [field]: value }),
      });

      if (!res.ok) {
        throw new Error('Failed to update profile');
      }

      const data = await res.json();
      setProfile(data.profile);
      setMessage({ type: 'success', text: 'Profile updated successfully!' });

      // Clear success message after 3 seconds
      setTimeout(() => setMessage({ type: '', text: '' }), 3000);
    } catch (error) {
      setMessage({ type: 'error', text: error.message });
    } finally {
      setSaving(false);
    }
  };

  if (loading) {
    return (
      <Layout title="Settings - Physical AI Book">
        <div className={styles.container}>
          <div className={styles.loading}>Loading profile...</div>
        </div>
      </Layout>
    );
  }

  if (!profile) {
    return (
      <Layout title="Settings - Physical AI Book">
        <div className={styles.container}>
          <div className={styles.error}>Failed to load profile</div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Profile Settings - Physical AI Book">
      <div className={styles.container}>
        <h1 className={styles.title}>Profile Settings</h1>
        <p className={styles.subtitle}>
          Customize your learning experience by updating your preferences below
        </p>

        {message.text && (
          <div className={clsx(styles.message, styles[message.type])}>
            {message.type === 'success' ? '✓ ' : '⚠️ '}
            {message.text}
          </div>
        )}

        <div className={styles.section}>
          <h2 className={styles.sectionTitle}>Software Level</h2>
          <p className={styles.sectionDesc}>
            Choose your programming experience level for tailored explanations
          </p>
          <div className={styles.radioGroup}>
            {SOFTWARE_LEVELS.map((level) => (
              <button
                key={level.value}
                type="button"
                className={clsx(
                  styles.optionButton,
                  profile.software_level === level.value && styles.selected
                )}
                onClick={() => handleSave('software_level', level.value)}
                disabled={saving}
              >
                <span className={styles.optionTitle}>{level.label}</span>
                <span className={styles.optionDesc}>{level.desc}</span>
              </button>
            ))}
          </div>
        </div>

        <div className={styles.section}>
          <h2 className={styles.sectionTitle}>Hardware/Robotics Experience</h2>
          <p className={styles.sectionDesc}>
            Your hands-on experience with robotics hardware
          </p>
          <div className={styles.radioGroup}>
            {HARDWARE_EXPERIENCE.map((exp) => (
              <button
                key={exp.value}
                type="button"
                className={clsx(
                  styles.optionButton,
                  profile.hardware_experience === exp.value && styles.selected
                )}
                onClick={() => handleSave('hardware_experience', exp.value)}
                disabled={saving}
              >
                <span className={styles.optionTitle}>{exp.label}</span>
                <span className={styles.optionDesc}>{exp.desc}</span>
              </button>
            ))}
          </div>
        </div>

        <div className={styles.section}>
          <h2 className={styles.sectionTitle}>Learning Depth Preference</h2>
          <p className={styles.sectionDesc}>
            How you prefer to learn new concepts
          </p>
          <div className={styles.radioGroup}>
            {LEARNING_DEPTH.map((depth) => (
              <button
                key={depth.value}
                type="button"
                className={clsx(
                  styles.optionButton,
                  profile.learning_depth === depth.value && styles.selected
                )}
                onClick={() => handleSave('learning_depth', depth.value)}
                disabled={saving}
              >
                <span className={styles.optionTitle}>{depth.label}</span>
                <span className={styles.optionDesc}>{depth.desc}</span>
              </button>
            ))}
          </div>
        </div>

        <div className={styles.infoBox}>
          <h3>💡 How This Works</h3>
          <p>
            Your profile settings affect how content is personalized for you:
          </p>
          <ul>
            <li><strong>Software Level</strong> - Controls technical depth of explanations</li>
            <li><strong>Hardware Experience</strong> - Affects hands-on vs theoretical content</li>
            <li><strong>Learning Depth</strong> - Balances concepts with practical examples</li>
          </ul>
        </div>
      </div>
    </Layout>
  );
}
