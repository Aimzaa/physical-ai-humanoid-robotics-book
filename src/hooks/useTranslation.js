import { useState, useEffect, useCallback } from 'react';

// Get API URL - works in both browser and SSR
function getApiUrl() {
  if (typeof window !== 'undefined') {
    const isProduction = window.location.hostname !== 'localhost';
    return isProduction ? 'https://your-backend-domain.com' : 'http://localhost:8000';
  }
  return 'http://localhost:8000';
}

/**
 * Custom hook for managing chapter translation state.
 *
 * @param {string} chapterId - The unique identifier for the chapter
 * @returns {Object} Translation state and methods
 */
export default function useTranslation(chapterId) {
  const [language, setLanguage] = useState('en');
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState(null);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [cacheExpiresAt, setCacheExpiresAt] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [apiUrl, setApiUrl] = useState('http://localhost:8000');

  // Initialize API URL on client side
  useEffect(() => {
    setApiUrl(getApiUrl());
  }, []);

  // Check authentication status
  useEffect(() => {
    const checkAuth = async () => {
      const url = getApiUrl();
      try {
        const res = await fetch(`${url}/auth/status`, {
          credentials: 'include',
        });
        const data = await res.json();
        setIsAuthenticated(data.authenticated && data.user);
      } catch (err) {
        setIsAuthenticated(false);
      }
    };

    checkAuth();
  }, []);

  // Load saved preference from localStorage
  useEffect(() => {
    if (!chapterId) return;

    const savedLang = localStorage.getItem(`translation_lang_${chapterId}`);
    const savedContent = localStorage.getItem(`translation_content_${chapterId}`);

    if (savedLang === 'ur' && savedContent) {
      setLanguage('ur');
      setTranslatedContent(savedContent);
    }
  }, [chapterId]);

  // Persist language preference
  useEffect(() => {
    if (!chapterId) return;

    localStorage.setItem(`translation_lang_${chapterId}`, language);
  }, [language, chapterId]);

  /**
   * Translate the current chapter to Urdu
   * @param {string} chapterTitle - The title of the chapter
   * @param {string} content - The content to translate
   */
  const translate = useCallback(async (chapterTitle, content) => {
    if (!isAuthenticated) {
      setError('Not authenticated');
      return false;
    }

    setIsTranslating(true);
    setError(null);

    try {
      const url = getApiUrl();
      const res = await fetch(`${url}/api/translate/${chapterId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_title: chapterTitle,
          content: content,
        }),
      });

      if (!res.ok) {
        const errorData = await res.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await res.json();

      setTranslatedContent(data.translated_content);
      setLanguage('ur');
      setCacheExpiresAt(data.ttl_seconds ? Date.now() + data.ttl_seconds * 1000 : null);

      // Save to localStorage
      localStorage.setItem(`translation_content_${chapterId}`, data.translated_content);

      return true;
    } catch (err) {
      setError(err.message);
      return false;
    } finally {
      setIsTranslating(false);
    }
  }, [chapterId, isAuthenticated]);

  /**
   * Toggle back to English
   */
  const toggleToEnglish = useCallback(() => {
    setLanguage('en');
    setTranslatedContent(null);
    localStorage.removeItem(`translation_content_${chapterId}`);
  }, [chapterId]);

  /**
   * Clear the cached translation
   */
  const clearCache = useCallback(async () => {
    try {
      const url = getApiUrl();
      await fetch(`${url}/api/translate/${chapterId}`, {
        method: 'DELETE',
        credentials: 'include',
      });

      toggleToEnglish();
      setCacheExpiresAt(null);
    } catch (err) {
      setError('Failed to clear cache');
    }
  }, [chapterId, toggleToEnglish]);

  /**
   * Check if a cached translation exists
   */
  const checkCacheStatus = useCallback(async () => {
    if (!isAuthenticated) return { hasTranslation: false };

    try {
      const url = getApiUrl();
      const res = await fetch(`${url}/api/translate/${chapterId}/status`, {
        credentials: 'include',
      });

      if (!res.ok) {
        return { hasTranslation: false };
      }

      return await res.json();
    } catch (err) {
      return { hasTranslation: false };
    }
  }, [chapterId, isAuthenticated]);

  /**
   * Get user translation preferences
   */
  const getPreferences = useCallback(async () => {
    if (!isAuthenticated) return null;

    try {
      const url = getApiUrl();
      const res = await fetch(`${url}/api/translate/preferences`, {
        credentials: 'include',
      });

      if (!res.ok) {
        return null;
      }

      return await res.json();
    } catch (err) {
      return null;
    }
  }, [isAuthenticated]);

  /**
   * Update user translation preferences
   */
  const updatePreferences = useCallback(async (updates) => {
    if (!isAuthenticated) return null;

    try {
      const url = getApiUrl();
      const res = await fetch(`${url}/api/translate/preferences`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(updates),
      });

      if (!res.ok) {
        throw new Error('Failed to update preferences');
      }

      return await res.json();
    } catch (err) {
      setError(err.message);
      return null;
    }
  }, [isAuthenticated]);

  return {
    // State
    language,
    isTranslating,
    error,
    translatedContent,
    cacheExpiresAt,
    isAuthenticated,
    hasCachedTranslation: !!translatedContent,

    // Methods
    translate,
    toggleToEnglish,
    clearCache,
    checkCacheStatus,
    getPreferences,
    updatePreferences,

    // Helpers
    setLanguage,
    setError,
  };
}

/**
 * Hook to check if user is authenticated for translation
 * @returns {Object} Authentication status
 */
export function useTranslationAuth() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const checkAuth = async () => {
      const url = getApiUrl();
      try {
        const res = await fetch(`${url}/auth/status`, {
          credentials: 'include',
        });
        const data = await res.json();
        setIsAuthenticated(data.authenticated && data.user);
      } catch (err) {
        setIsAuthenticated(false);
      } finally {
        setLoading(false);
      }
    };

    checkAuth();
  }, []);

  return { isAuthenticated, loading };
}
