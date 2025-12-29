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

export default function TranslationToggle({ chapterId, chapterTitle }) {
  // TEMP: For testing, show button to everyone
  const [isAuthenticated, setIsAuthenticated] = useState(true); // Always true for testing
  const [loading, setLoading] = useState(false); // No loading for auth
  const [translating, setTranslating] = useState(false);
  const [language, setLanguage] = useState('en');
  const [translatedContent, setTranslatedContent] = useState(null);
  const [error, setError] = useState(null);
  const [apiUrl, setApiUrl] = useState('http://localhost:8000');
  const contentRef = useRef(null);

  // Get URLs with baseUrl
  const signupUrl = useBaseUrl('/signup');
  const signinUrl = useBaseUrl('/signin');

  useEffect(() => {
    // Set API URL on client side
    setApiUrl(getApiUrl());
    console.log('Translation API URL:', getApiUrl());
  }, []);

  useEffect(() => {
    // TEMP: Skip auth check for testing
    // Load saved language preference for this chapter
    if (chapterId) {
      const savedLang = localStorage.getItem(`translation_lang_${chapterId}`);
      if (savedLang === 'ur') {
        const cachedContent = localStorage.getItem(`translation_content_${chapterId}`);
        if (cachedContent) {
          setLanguage('ur');
          setTranslatedContent(cachedContent);
        }
      }
    }
  }, [chapterId]);

  // Extract chapter content from the page
  const getChapterContent = () => {
    const article = document.querySelector('article');
    if (!article) return null;

    // Get title from the page
    const titleEl = document.querySelector('h1');
    const title = titleEl ? titleEl.textContent : chapterTitle || chapterId;

    // Get the main content
    const contentElements = article.querySelectorAll('p, h2, h3, h4, ul, ol, pre, table');
    let content = '';
    for (const el of contentElements) {
      if (content.length > 5000) break;
      content += el.textContent + '\n\n';
    }

    return { title, content: content.trim() };
  };

  const handleTranslate = async () => {
    setTranslating(true);
    setError(null);

    try {
      const chapterData = getChapterContent();
      if (!chapterData) {
        throw new Error('Could not extract chapter content');
      }

      console.log('Translating chapter:', chapterId);
      console.log('API URL:', `${apiUrl}/api/translate/${chapterId}`);
      console.log('Content length:', chapterData.content.length);

      const res = await fetch(`${apiUrl}/api/translate/${chapterId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_title: chapterData.title,
          content: chapterData.content,
        }),
      });

      console.log('Response status:', res.status);

      if (!res.ok) {
        const errorText = await res.text();
        console.error('Error response:', errorText);
        throw new Error(`Translation failed: ${res.status} - ${errorText}`);
      }

      const data = await res.json();
      console.log('Translation success! Content length:', data.translated_content?.length);

      setTranslatedContent(data.translated_content);
      setLanguage('ur');

      // Save to localStorage for persistence
      localStorage.setItem(`translation_lang_${chapterId}`, 'ur');
      localStorage.setItem(`translation_content_${chapterId}`, data.translated_content);

      // Scroll to translated content
      setTimeout(() => {
        contentRef.current?.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }, 100);

    } catch (err) {
      console.error('Translation failed:', err);
      setError(err.message);
    } finally {
      setTranslating(false);
    }
  };

  const handleToggleBack = () => {
    setLanguage('en');
    setTranslatedContent(null);
    localStorage.setItem(`translation_lang_${chapterId}`, 'en');
    localStorage.removeItem(`translation_content_${chapterId}`);
  };

  const handleClearCache = async () => {
    try {
      const url = getApiUrl();
      await fetch(`${url}/api/translate/${chapterId}`, {
        method: 'DELETE',
        credentials: 'include',
      });

      // Clear local state and storage
      setLanguage('en');
      setTranslatedContent(null);
      localStorage.removeItem(`translation_lang_${chapterId}`);
      localStorage.removeItem(`translation_content_${chapterId}`);
    } catch (err) {
      console.error('Failed to clear cache:', err);
    }
  };

  const handleRetry = () => {
    setError(null);
    handleTranslate();
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
          <span>Sign in to translate this chapter to Urdu</span>
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

  // Show toggle buttons for authenticated users
  return (
    <div className={styles.wrapper}>
      <div className={styles.container}>
        <div className={styles.buttonRow}>
          {language === 'en' ? (
            <button
              className={clsx(styles.button, styles.translateButton)}
              onClick={handleTranslate}
              disabled={translating}
            >
              {translating ? (
                <>
                  <span className={styles.spinner}></span>
                  <span>Translating to Urdu...</span>
                </>
              ) : (
                <>
                  <span className={styles.translateIcon}>🌐</span>
                  <span>Translate to Urdu</span>
                </>
              )}
            </button>
          ) : (
            <>
              <button
                className={clsx(styles.button, styles.toggleBackButton)}
                onClick={handleToggleBack}
              >
                <span className={styles.backIcon}>←</span>
                <span>Show English</span>
              </button>
              <button
                className={clsx(styles.button, styles.clearCacheButton)}
                onClick={handleClearCache}
                title="Clear cached translation"
              >
                <span className={styles.clearIcon}>🗑️</span>
              </button>
            </>
          )}
        </div>

        {error && (
          <div className={styles.errorNotice}>
            <span>⚠️ {error}</span>
            <button onClick={handleRetry} className={styles.retryButton}>
              Retry
            </button>
          </div>
        )}
      </div>

      {/* Translated Content Display */}
      {language === 'ur' && translatedContent && (
        <div ref={contentRef} className={styles.translatedSection}>
          <div className={styles.translatedHeader}>
            <span className={styles.translatedIcon}>🌙</span>
            <span className={styles.translatedTitle}>اردو میں پڑھیں</span>
            <span className={styles.translatedBadge}>AI Translated</span>
          </div>
          <div
            className={styles.translatedContent}
            dir="rtl"
            lang="ur"
            dangerouslySetInnerHTML={{
              __html: formatMarkdown(translatedContent)
            }}
          />
          <div className={styles.translatedFooter}>
            <span className={styles.footerNote}>
              📝 Translated to Urdu. Technical terms are kept in English with Urdu explanations.
            </span>
            <button onClick={handleToggleBack} className={styles.footerToggle}>
              View Original English
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

// Simple markdown to HTML converter for Urdu content
function formatMarkdown(text) {
  if (!text) return '';

  // Escape HTML first
  let html = text
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');

  // Code blocks - keep code in English
  html = html.replace(/```(\w+)?\n([\s\S]*?)```/g, '<pre class="urdu-code"><code>$2</code></pre>');

  // Inline code
  html = html.replace(/`([^`]+)`/g, '<code class="urdu-inline-code">$1</code>');

  // Headers
  html = html.replace(/^### (.+)$/gm, '<h4>$1</h4>');
  html = html.replace(/^## (.+)$/gm, '<h3>$1</h3>');
  html = html.replace(/^# (.+)$/gm, '<h2>$1</h2>');

  // Bold
  html = html.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');

  // Italic
  html = html.replace(/\*([^*]+)\*/g, '<em>$1</em>');

  // Lists
  html = html.replace(/^- (.+)$/gm, '<li>$1</li>');
  html = html.replace(/(<li>.*<\/li>\n?)+/g, '<ul>$&</ul>');

  // Numbered lists
  html = html.replace(/^\d+\. (.+)$/gm, '<li>$1</li>');

  // Links - preserve URL, translate text
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener noreferrer">$1</a>');

  // Paragraphs - wrap remaining text
  html = html
    .replace(/\n\n/g, '</p><p>')
    .replace(/^(?!<)(.+)$/gm, '<p>$1</p>');

  // Clean up
  html = html
    .replace(/<p><\/p>/g, '')
    .replace(/<p>(<h[234]>)/g, '$1')
    .replace(/(<\/h[234]>)<\/p>/g, '$1')
    .replace(/<p>(<ul>)/g, '$1')
    .replace(/(<\/ul>)<\/p>/g, '$1')
    .replace(/<p>(<pre)/g, '$1')
    .replace(/(<\/pre>)<\/p>/g, '$1')
    .replace(/<p>(<table)/g, '$1')
    .replace(/(<\/table>)<\/p>/g, '$1')
    .replace(/<p>(<li>)/g, '$1')
    .replace(/(<\/li>)<\/p>/g, '$1');

  return html;
}
