import React, { useState, useEffect } from 'react';
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

export default function AuthButton() {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [menuOpen, setMenuOpen] = useState(false);

  // Get base URL for links
  const signinUrl = useBaseUrl('/signin');
  const signupUrl = useBaseUrl('/signup');
  const settingsUrl = useBaseUrl('/settings');

  useEffect(() => {
    const checkAuth = async () => {
      const apiUrl = getApiUrl();
      try {
        const res = await fetch(`${apiUrl}/auth/status`, {
          credentials: 'include',
        });
        const data = await res.json();
        if (data.authenticated) {
          setUser(data.user);
        }
      } catch (error) {
        // Backend not running - show guest menu
        console.log('Auth check: Backend not available');
      } finally {
        setLoading(false);
      }
    };

    checkAuth();
  }, []);

  const handleSignOut = async () => {
    const apiUrl = getApiUrl();
    try {
      await fetch(`${apiUrl}/auth/signout`, {
        method: 'POST',
        credentials: 'include',
      });
      setUser(null);
      window.location.reload();
    } catch (error) {
      console.error('Sign out failed:', error);
    }
  };

  if (loading) {
    return (
      <div className={styles.guestMenu}>
        <span className={styles.loading}>...</span>
      </div>
    );
  }

  if (user) {
    return (
      <div className={styles.userMenu}>
        <button
          className={styles.userButton}
          onClick={() => setMenuOpen(!menuOpen)}
        >
          <span className={styles.userIcon}>👤</span>
          <span className={styles.userEmail}>{user.email}</span>
          <span className={styles.dropdownArrow}>▼</span>
        </button>
        {menuOpen && (
          <div className={styles.dropdown}>
            <a href={settingsUrl} className={styles.dropdownItem}>
              ⚙️ Profile Settings
            </a>
            <button onClick={handleSignOut} className={styles.dropdownItem}>
              🚪 Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className={styles.guestMenu}>
      <a href={signinUrl} className={styles.signInLink}>
        Sign In
      </a>
      <a href={signupUrl} className={clsx(styles.signUpButton, 'button', 'button--primary')}>
        Sign Up
      </a>
    </div>
  );
}
