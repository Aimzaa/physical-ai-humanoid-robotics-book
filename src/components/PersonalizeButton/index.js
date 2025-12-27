import React, { useState, useEffect, useRef } from 'react';
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

export default function PersonalizeButton({ chapterId }) {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);
  const [personalizing, setPersonalizing] = useState(false);
  const [enabled, setEnabled] = useState(false);
  const [userLevel, setUserLevel] = useState(null);
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [error, setError] = useState(null);
  const [apiUrl, setApiUrl] = useState('http://localhost:8000');
  const contentRef = useRef(null);

  // Get URLs with baseUrl
  const signupUrl = useBaseUrl('/signup');
  const signinUrl = useBaseUrl('/signin');
  const settingsUrl = useBaseUrl('/settings');

  useEffect(() => {
    // Set API URL on client side
    setApiUrl(getApiUrl());
  }, []);

  useEffect(() => {
    // Check authentication status
    const checkAuth = async () => {
      const url = getApiUrl();
      try {
        const res = await fetch(`${url}/auth/status`, {
          credentials: 'include',
        });
        const data = await res.json();

        if (data.authenticated && data.user) {
          setIsAuthenticated(true);
          // Set user level from profile
          if (data.user.profile) {
            setUserLevel({
              software: data.user.profile.software_level,
              hardware: data.user.profile.hardware_experience,
              learning: data.user.profile.learning_depth,
            });
          }
        } else {
          setIsAuthenticated(false);
        }
      } catch (error) {
        console.log('Auth check failed:', error);
        setIsAuthenticated(false);
      } finally {
        setLoading(false);
      }
    };

    checkAuth();

    // Load saved preference and content
    if (chapterId) {
      const saved = localStorage.getItem(`personalize_${chapterId}`);
      const savedContent = localStorage.getItem(`personalized_content_${chapterId}`);
      if (saved === 'true' && savedContent) {
        setEnabled(true);
        try {
          setPersonalizedContent(JSON.parse(savedContent));
        } catch (e) {
          console.error('Failed to parse saved content');
        }
      }
    }
  }, [chapterId]);

  // Extract chapter content from the page
  const getChapterContent = () => {
    // Get the main content area (Docusaurus markdown content)
    const article = document.querySelector('article');
    if (!article) return null;

    // Get title
    const titleEl = article.querySelector('h1');
    const title = titleEl ? titleEl.textContent : chapterId;

    // Get the first few sections of content (limit to ~2000 chars for API)
    const contentElements = article.querySelectorAll('p, h2, h3, ul, ol, pre');
    let content = '';
    for (const el of contentElements) {
      if (content.length > 3000) break;
      content += el.textContent + '\n\n';
    }

    return { title, content: content.trim() };
  };

  const handlePersonalize = async () => {
    setPersonalizing(true);
    setError(null);

    try {
      const chapterData = getChapterContent();
      if (!chapterData) {
        throw new Error('Could not extract chapter content');
      }

      const res = await fetch(`${apiUrl}/api/personalize/${chapterId}/content`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          chapter_title: chapterData.title,
          chapter_content: chapterData.content,
        }),
      });

      if (!res.ok) {
        const errorData = await res.json();
        throw new Error(errorData.detail || 'Personalization failed');
      }

      const data = await res.json();

      setPersonalizedContent(data);
      setEnabled(true);
      localStorage.setItem(`personalize_${chapterId}`, 'true');
      localStorage.setItem(`personalized_content_${chapterId}`, JSON.stringify(data));

      // Scroll to personalized content
      setTimeout(() => {
        contentRef.current?.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }, 100);

    } catch (err) {
      console.error('Personalization failed:', err);
      setError(err.message);
    } finally {
      setPersonalizing(false);
    }
  };

  const handleReset = () => {
    setEnabled(false);
    setPersonalizedContent(null);
    localStorage.removeItem(`personalize_${chapterId}`);
    localStorage.removeItem(`personalized_content_${chapterId}`);
  };

  // Show loading state
  if (loading) {
    return (
      <div className={styles.container}>
        <div className={styles.loadingState}>
          <span className={styles.spinner}></span>
          <span>Loading...</span>
        </div>
      </div>
    );
  }

  // Show sign-up prompt for non-authenticated users
  if (!isAuthenticated) {
    return (
      <div className={styles.container}>
        <div className={styles.prompt}>
          <span className={styles.lockIcon}>🔒</span>
          <span>Sign in to personalize this chapter based on your experience level</span>
          <div className={styles.authLinks}>
            <a href={signinUrl} className={styles.signInLink}>
              Sign In
            </a>
            <span className={styles.authDivider}>or</span>
            <a href={signupUrl} className={styles.signUpLink}>
              Sign Up
            </a>
          </div>
        </div>
      </div>
    );
  }

  // Show personalization button and content for authenticated users
  return (
    <div className={styles.wrapper}>
      <div className={styles.container}>
        <div className={styles.buttonRow}>
          <button
            className={clsx(
              styles.button,
              enabled ? styles.enabled : styles.disabled
            )}
            onClick={handlePersonalize}
            disabled={personalizing}
          >
            {personalizing ? (
              <>
                <span className={styles.spinner}></span>
                <span>Generating personalized content...</span>
              </>
            ) : enabled ? (
              <>
                <span className={styles.checkIcon}>✓</span>
                <span>Content Personalized</span>
              </>
            ) : (
              <>
                <span className={styles.personalizeIcon}>✨</span>
                <span>Personalize This Chapter</span>
              </>
            )}
          </button>

          {enabled && (
            <button
              className={styles.resetButton}
              onClick={handleReset}
              disabled={personalizing}
              title="Show original content"
            >
              Show Original
            </button>
          )}
        </div>

        {userLevel && (
          <div className={styles.levelInfo}>
            <span className={styles.levelLabel}>Your profile:</span>
            <span className={styles.levelBadge}>{userLevel.software}</span>
            <span className={styles.levelBadge}>{userLevel.hardware} hardware</span>
            <span className={styles.levelBadge}>{userLevel.learning}</span>
            <a href={settingsUrl} className={styles.settingsLink}>
              Edit
            </a>
          </div>
        )}

        {error && (
          <div className={styles.errorNotice}>
            <span>⚠️ {error}</span>
          </div>
        )}
      </div>

      {/* Personalized Content Display */}
      {enabled && personalizedContent && (
        <div ref={contentRef} className={styles.personalizedSection}>
          <div className={styles.personalizedHeader}>
            <span className={styles.personalizedIcon}>✨</span>
            <span className={styles.personalizedTitle}>
              Personalized for {userLevel?.software || 'your'} level
            </span>
            <span className={styles.personalizedBadge}>AI Adapted</span>
          </div>
          <div
            className={styles.personalizedContent}
            dangerouslySetInnerHTML={{
              __html: formatMarkdown(personalizedContent.personalized_content)
            }}
          />
          <div className={styles.personalizedFooter}>
            <span className={styles.footerNote}>
              📝 This content has been adapted to your {userLevel?.software} software level,
              {userLevel?.hardware} hardware experience, and {userLevel?.learning} learning preference.
            </span>
            <button onClick={handleReset} className={styles.footerReset}>
              View Original Content
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

// Simple markdown to HTML converter for display
function formatMarkdown(text) {
  if (!text) return '';

  return text
    // Code blocks
    .replace(/```(\w+)?\n([\s\S]*?)```/g, '<pre class="personalized-code"><code>$2</code></pre>')
    // Inline code
    .replace(/`([^`]+)`/g, '<code class="personalized-inline-code">$1</code>')
    // Headers
    .replace(/^### (.+)$/gm, '<h4>$1</h4>')
    .replace(/^## (.+)$/gm, '<h3>$1</h3>')
    .replace(/^# (.+)$/gm, '<h2>$1</h2>')
    // Bold
    .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
    // Italic
    .replace(/\*([^*]+)\*/g, '<em>$1</em>')
    // Lists
    .replace(/^- (.+)$/gm, '<li>$1</li>')
    .replace(/(<li>.*<\/li>\n?)+/g, '<ul>$&</ul>')
    // Numbered lists
    .replace(/^\d+\. (.+)$/gm, '<li>$1</li>')
    // Paragraphs
    .replace(/\n\n/g, '</p><p>')
    .replace(/^(?!<)(.+)$/gm, '<p>$1</p>')
    // Clean up
    .replace(/<p><\/p>/g, '')
    .replace(/<p>(<h[234]>)/g, '$1')
    .replace(/(<\/h[234]>)<\/p>/g, '$1')
    .replace(/<p>(<ul>)/g, '$1')
    .replace(/(<\/ul>)<\/p>/g, '$1')
    .replace(/<p>(<pre)/g, '$1')
    .replace(/(<\/pre>)<\/p>/g, '$1');
}
