/**
 * React Chat Component for RAG Chatbot
 */
import React, { useState, useEffect, useRef } from 'react';

const ChatComponent = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Function to handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim()) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API running on port 8000
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          session_id: 'session-8000-session-' + Date.now(), // Unique session ID
          selected_text: selectedText || null
        })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'bot',
          timestamp: new Date(),
          sources: data.context_sources
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        throw new Error('Failed to get response');
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
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

  const askAboutSelectedText = () => {
    if (selectedText) {
      setInputValue(`About this: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`);
    }
  };

  return (
    <div className="chat-container">
      {/* Floating button */}
      <button
        className="chat-toggle-button"
        onClick={toggleChat}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#1a73e8',
          color: 'white',
          border: 'none',
          fontSize: '24px',
          cursor: 'pointer',
          zIndex: '1000',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
        }}
      >
        💬
      </button>

      {/* Chat window */}
      {isOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '400px',
            height: '500px',
            backgroundColor: 'white',
            border: '1px solid #ddd',
            borderRadius: '8px',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            zIndex: '1000',
            display: 'flex',
            flexDirection: 'column'
          }}
        >
          {/* Header */}
          <div
            style={{
              backgroundColor: '#1a73e8',
              color: 'white',
              padding: '12px',
              borderTopLeftRadius: '8px',
              borderTopRightRadius: '8px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>Book Assistant</h3>
            <button
              onClick={toggleChat}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '18px',
                cursor: 'pointer'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages */}
          <div
            style={{
              flex: 1,
              padding: '12px',
              overflowY: 'auto',
              backgroundColor: '#f9f9f9'
            }}
          >
            {messages.length === 0 ? (
              <div style={{ textAlign: 'center', color: '#666', marginTop: '20px' }}>
                <p>Ask me about the Physical AI & Humanoid Robotics content!</p>
                {selectedText && (
                  <div style={{ marginTop: '10px' }}>
                    <p><small>Selected text: "{selectedText.substring(0, 50)}..."</small></p>
                    <button
                      onClick={askAboutSelectedText}
                      style={{
                        backgroundColor: '#3cba54',
                        color: 'white',
                        border: 'none',
                        padding: '4px 8px',
                        borderRadius: '4px',
                        cursor: 'pointer',
                        fontSize: '12px'
                      }}
                    >
                      Ask about this
                    </button>
                  </div>
                )}
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  style={{
                    marginBottom: '12px',
                    textAlign: message.sender === 'user' ? 'right' : 'left'
                  }}
                >
                  <div
                    style={{
                      display: 'inline-block',
                      padding: '8px 12px',
                      borderRadius: '18px',
                      backgroundColor: message.sender === 'user' ? '#e3f2fd' : '#f1f1f1',
                      color: '#333',
                      maxWidth: '80%'
                    }}
                  >
                    {message.text}
                    {message.sources && message.sources.length > 0 && (
                      <div style={{ fontSize: '12px', marginTop: '4px', color: '#666' }}>
                        Sources: {message.sources.join(', ')}
                      </div>
                    )}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div style={{ textAlign: 'left', marginBottom: '12px' }}>
                <div
                  style={{
                    display: 'inline-block',
                    padding: '8px 12px',
                    borderRadius: '18px',
                    backgroundColor: '#f1f1f1',
                    color: '#333'
                  }}
                >
                  Thinking...
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <div style={{ padding: '12px', borderTop: '1px solid #eee' }}>
            {selectedText && messages.length === 0 && (
              <div style={{ marginBottom: '8px', fontSize: '12px' }}>
                <span>Selected: "{selectedText.substring(0, 50)}..."</span>
                <button
                  onClick={askAboutSelectedText}
                  style={{
                    marginLeft: '8px',
                    backgroundColor: '#3cba54',
                    color: 'white',
                    border: 'none',
                    padding: '2px 6px',
                    borderRadius: '4px',
                    cursor: 'pointer',
                    fontSize: '10px'
                  }}
                >
                  Use this text
                </button>
              </div>
            )}
            <div style={{ display: 'flex' }}>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about the book content..."
                style={{
                  flex: 1,
                  padding: '8px',
                  border: '1px solid #ddd',
                  borderRadius: '4px',
                  resize: 'none',
                  minHeight: '40px',
                  maxHeight: '80px'
                }}
                rows="1"
              />
              <button
                onClick={sendMessage}
                disabled={isLoading || !inputValue.trim()}
                style={{
                  marginLeft: '8px',
                  padding: '8px 16px',
                  backgroundColor: isLoading ? '#ccc' : '#1a73e8',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: isLoading ? 'not-allowed' : 'pointer'
                }}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatComponent;