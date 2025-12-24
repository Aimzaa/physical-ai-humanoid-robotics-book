import React from 'react';
import { createRoot } from 'react-dom/client';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

let chatbotRoot = null;

function createChatbotContainer() {
  if (!ExecutionEnvironment.canUseDOM) {
    return;
  }

  // Create a container for the chatbot
  const chatbotContainer = document.createElement('div');
  chatbotContainer.id = 'chatbot-container';
  document.body.appendChild(chatbotContainer);

  // Create and render the chatbot widget
  chatbotRoot = createRoot(chatbotContainer);
  chatbotRoot.render(<ChatbotWidget />);
}

function removeChatbotContainer() {
  if (chatbotRoot) {
    chatbotRoot.unmount();
    chatbotRoot = null;
  }

  const chatbotContainer = document.getElementById('chatbot-container');
  if (chatbotContainer) {
    chatbotContainer.remove();
  }
}

// Initialize on mount
if (ExecutionEnvironment.canUseDOM) {
  // Wait for the DOM to be ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', createChatbotContainer);
  } else {
    createChatbotContainer();
  }
}

// Clean up on unmount
export function onRouteDidUpdate() {
  // Remove existing container if it exists to prevent duplicates
  removeChatbotContainer();

  // Recreate the chatbot container
  if (ExecutionEnvironment.canUseDOM) {
    setTimeout(createChatbotContainer, 0);
  }
}