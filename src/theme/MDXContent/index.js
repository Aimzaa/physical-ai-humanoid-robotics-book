import React, { useState, useEffect } from 'react';
import MDXContent from '@theme-original/MDXContent';
import { useLocation } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslationToggle from '@site/src/components/TranslationToggle';

// Check if current page is a chapter (in docs folder, not index)
function isChapterPage(pathname) {
  // Match docs pages that are chapters (contain "chapter-" or "module-")
  const isDocsPage = pathname.includes('/docs/');
  const isChapter = pathname.includes('chapter-') ||
                    pathname.includes('module-') ||
                    pathname.includes('capstone-') ||
                    pathname.includes('development-environment');
  const isNotIndex = !pathname.endsWith('/docs/') && !pathname.endsWith('/docs');

  return isDocsPage && isChapter && isNotIndex;
}

// Extract chapter ID from pathname
function getChapterId(pathname) {
  // Get the last segment of the path as chapter ID
  const segments = pathname.split('/').filter(Boolean);
  const lastSegment = segments[segments.length - 1];
  // Remove any trailing slash or hash
  return lastSegment.replace(/[/#].*$/, '');
}

// Client-side only component for translation toggle
function TranslationToggleWrapper({ chapterId }) {
  const [chapterTitle, setChapterTitle] = useState(null);

  useEffect(() => {
    // Get chapter title from DOM (only runs on client)
    const titleEl = document.querySelector('h1');
    if (titleEl) {
      setChapterTitle(titleEl.textContent);
    }
  }, []);

  return <TranslationToggle chapterId={chapterId} chapterTitle={chapterTitle} />;
}

export default function MDXContentWrapper(props) {
  const location = useLocation();
  const pathname = location.pathname;

  const showPersonalizeButton = isChapterPage(pathname);
  const showTranslationToggle = isChapterPage(pathname);
  const chapterId = showTranslationToggle ? getChapterId(pathname) : null;

  return (
    <>
      {showTranslationToggle && (
        <BrowserOnly fallback={null}>
          {() => <TranslationToggleWrapper chapterId={chapterId} />}
        </BrowserOnly>
      )}
      {showPersonalizeButton && (
        <PersonalizeButton chapterId={chapterId} />
      )}
      <MDXContent {...props} />
    </>
  );
}
