import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const API_URL = process.env.DOCUSAURUS_ENV === 'production'
  ? 'https://your-backend-domain.com'
  : 'http://localhost:8000';

interface User {
  id: string;
  email: string;
  profile?: {
    software_level: string;
    hardware_experience: string;
    learning_depth: string;
  };
}

export default function AuthButton(): JSX.Element {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const [menuOpen, setMenuOpen] = useState(false);

  useEffect(() => {
    checkAuth();
  }, []);

  const checkAuth = async () => {
    try {
      const res = await fetch(`${API_URL}/auth/status`, {
        credentials: 'include',
      });
      const data = await res.json();
      if (data.authenticated) {
        setUser(data.user);
      }
    } catch (error) {
      console.error('Auth check failed:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleSignOut = async () => {
    try {
      await fetch(`${API_URL}/auth/signout`, {
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
      <button className={styles.authButton} disabled>
        <span className={styles.loading}>Loading...</span>
      </button>
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
            <a href="/settings" className={styles.dropdownItem}>
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
      <a href="/signin" className={styles.signInLink}>
        Sign In
      </a>
      <a href="/signup" className={clsx(styles.signUpButton, 'button', 'button--primary')}>
        Sign Up
      </a>
    </div>
  );
}
