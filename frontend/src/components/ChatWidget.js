import React, { useState, useRef, useEffect } from 'react';
import { chatApi } from '../utils/api';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Get conversation ID from the last message if it exists
      const lastMessage = messages[messages.length - 1];
      const conversationId = lastMessage?.conversationId || null;

      // Send message to backend
      const response = await chatApi.sendMessage(inputValue, conversationId);

      const botMessage = {
        id: Date.now() + 1,
        text: response.response,
        sender: 'bot',
        sources: response.sources || [],
        conversationId: response.conversation_id,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (err) {
      setError('Failed to get response. Please try again.');
      console.error('Chat error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const formatSources = (sources) => {
    if (!sources || sources.length === 0) return null;

    return (
      <div className="sources-section">
        <p className="sources-title">Sources:</p>
        <ul className="sources-list">
          {sources.slice(0, 3).map((source, index) => (
            <li key={index} className="source-item">
              {source.split('/').pop().replace('.md', '')}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Physical AI Assistant</h3>
            <button
              className="close-button"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Physical AI & Humanoid Robotics assistant.</p>
                <p>Ask me anything about the textbook content:</p>
                <ul className="suggestions">
                  <li>What is ROS 2?</li>
                  <li>Explain humanoid locomotion</li>
                  <li>How does Gazebo simulation work?</li>
                </ul>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}-message`}
                >
                  <div className="message-content">
                    {message.sender === 'bot' && (
                      <div className="bot-icon">🤖</div>
                    )}
                    <div className="message-text">
                      {message.text.split('\n').map((line, i) => (
                        <p key={i}>{line}</p>
                      ))}
                      {message.sources && formatSources(message.sources)}
                    </div>
                    {message.sender === 'user' && (
                      <div className="user-icon">👤</div>
                    )}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <div className="bot-icon">🤖</div>
                  <div className="message-text">
                    <div className="typing-indicator">
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            {error && <div className="error-message">{error}</div>}
            <div className="input-container">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about Physical AI & Robotics..."
                className="chat-input"
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={handleSendMessage}
                disabled={!inputValue.trim() || isLoading}
                className="send-button"
                aria-label="Send message"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 2L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
          </div>
        </div>
      ) : (
        <button
          className="chat-toggle-button"
          onClick={toggleChat}
          aria-label="Open chat"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H7L3 21V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
          <span>AI Assistant</span>
        </button>
      )}
    </div>
  );
};

export default ChatWidget;