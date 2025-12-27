// Auth client module - injects AuthButton into navbar
import React from 'react';
import { createRoot } from 'react-dom/client';
import AuthButton from '@site/src/components/AuthButton';

let root = null;

function injectAuthButton() {
  // Find the navbar right section
  const navbarRight = document.querySelector('.navbar__items--right');

  if (!navbarRight) {
    // Try again later
    setTimeout(injectAuthButton, 100);
    return;
  }

  // Check if already injected
  if (document.getElementById('auth-button-container')) {
    return;
  }

  // Create container
  const container = document.createElement('div');
  container.id = 'auth-button-container';
  container.className = 'navbar__item';
  navbarRight.insertBefore(container, navbarRight.firstChild);

  // Render AuthButton using React 18 createRoot
  root = createRoot(container);
  root.render(<AuthButton />);
}

// Wait for DOM to be ready
if (typeof document !== 'undefined') {
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', injectAuthButton);
  } else {
    injectAuthButton();
  }
}
