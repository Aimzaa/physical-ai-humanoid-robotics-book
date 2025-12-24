import React, { useState, useRef, useEffect, useCallback } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './ChatbotWidget.css';

// Function to get current theme from document
const getCurrentTheme = () => {
  const html = document.documentElement;
  const theme = html.getAttribute('data-theme');
  return theme || 'light';
};

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [currentTheme, setCurrentTheme] = useState('light');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const { siteConfig } = useDocusaurusContext();

  // Function to update theme based on document attribute
  const updateTheme = useCallback(() => {
    const theme = getCurrentTheme();
    setCurrentTheme(theme);
  }, []);

  // Set up theme detection
  useEffect(() => {
    // Initial theme detection
    updateTheme();

    // Create a MutationObserver to watch for theme changes
    const observer = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.type === 'attributes' && mutation.attributeName === 'data-theme') {
          updateTheme();
        }
      });
    });

    // Start observing the html element
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['data-theme'],
    });

    // Also listen for Docusaurus theme change events
    const handleThemeChange = () => {
      updateTheme();
    };

    document.addEventListener('theme-change', handleThemeChange);

    return () => {
      observer.disconnect();
      document.removeEventListener('theme-change', handleThemeChange);
    };
  }, [updateTheme]);

  // Ye line change ki hai – ab process nahi use kar rahe
  const API_BASE_URL = 'http://localhost:8000';   // ← yahan hard code kar diya – error khatam

  // Function to get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText.length > 0 && selectedText.length < 500) { // Limit to 500 chars
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = inputValue.trim();
    const contextToSend = selectedText;

    // Add user message to chat
    const newMessages = [...messages, { type: 'user', content: userMessage }];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend RAG API
      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userMessage,
          user_context: contextToSend || null,
          model: "openai/gpt-3.5-turbo"
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response to chat
      setMessages(prev => [
        ...prev,
        {
          type: 'assistant',
          content: data.response,
          sources: data.sources || []
        }
      ]);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [
        ...prev,
        {
          type: 'assistant',
          content: `Sorry, I encountered an error: ${error.message}. Please try again later.`
        }
      ]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  };

  const clearChat = () => {
    setMessages([]);
  };

  return (
    <div className={`chatbot-widget ${currentTheme}`}>
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <div className="chatbot-title">
              <span>📚 Book Assistant</span>
            </div>
            <div className="chatbot-controls">
              <button
                onClick={clearChat}
                className="chatbot-button"
                title="Clear chat"
              >
                🗑️
              </button>
              <button
                onClick={toggleChat}
                className="chatbot-button"
                title="Close chat"
              >
                −
              </button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <h4>Hello! I'm your Physical AI & Humanoid Robotics Book assistant.</h4>
                <p>Ask me anything about the book content:</p>
                <ul>
                  <li>• ROS 2 architecture and concepts</li>
                  <li>• Gazebo simulation and Unity integration</li>
                  <li>• NVIDIA Isaac platform and VSLAM</li>
                  <li>• Vision-Language-Action systems</li>
                  <li>• Autonomous humanoid implementation</li>
                </ul>
                <p>Select text on the page to add context to your questions!</p>
              </div>
            ) : (
              messages.map((msg, index) => (
                <div
                  key={index}
                  className={`chatbot-message ${msg.type}`}
                >
                  <div className="chatbot-message-content">
                    {msg.type === 'assistant' && msg.sources && msg.sources.length > 0 && (
                      <div className="chatbot-sources">
                        <small>Sources: {msg.sources.slice(0, 2).map(s =>
                          s.split('/').pop().replace('.md', '').replace(/-/g, ' ')
                        ).join(', ')}</small>
                      </div>
                    )}
                    <div>{msg.content}</div>
                  </div>
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className="chatbot-context">
              <small>Using selected text as context:</small>
              <div className="chatbot-selected-text">
                "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
              </div>
            </div>
          )}

          <div className="chatbot-input-area">
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about the book content..."
              disabled={isLoading}
              rows="2"
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
              className="chatbot-send-button"
            >
              {isLoading ? ' Sending...' : '➤'}
            </button>
          </div>
        </div>
      ) : (
        <button
          className="chatbot-toggle-button"
          onClick={toggleChat}
        >
          <span>📚</span>
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;