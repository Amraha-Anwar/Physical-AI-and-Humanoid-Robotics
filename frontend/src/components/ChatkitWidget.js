import React, { useState, useEffect, useRef } from 'react';
import { MessageCircle, X, Sparkles, Send } from 'lucide-react';
import styles from './ChatkitWidget.module.css';

const ChatkitWidget = () => {
  // Use environment variable if available, otherwise fallback to your Render link
  const API_BASE_URL = "https://physical-ai-and-humanoid-robotics-1.onrender.com";
  const [sessionId] = useState(() => `session-${Date.now()}-${Math.random().toString(36).substring(2, 9)}`);
  
  const [messages, setMessages] = useState([
    {
      message: "Hello! How can I help you with Humanoid Robotics today?",
      sentTime: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      sender: "Assistant",
      direction: "incoming",
    }
  ]);

  const [isTyping, setIsTyping] = useState(false);
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [inputValue, setInputValue] = useState('');
  const [askAiButton, setAskAiButton] = useState({ show: false, x: 0, y: 0, text: '' });
  
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isTyping]);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      if (selectedText.length > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        setAskAiButton({
          show: true,
          x: rect.right + 10,
          y: rect.top + window.scrollY,
          text: selectedText
        });
      } else {
        setAskAiButton({ show: false, x: 0, y: 0, text: '' });
      }
    };

    const handleClickOutside = (e) => {
      if (!e.target.closest(`.${styles.askButton}`)) {
        setAskAiButton({ show: false, x: 0, y: 0, text: '' });
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const handleAskAI = () => {
    if (askAiButton.text) {
      setIsChatOpen(true);
      const query = `Explain: ${askAiButton.text}`;
      setAskAiButton({ show: false, x: 0, y: 0, text: '' });
      
      setTimeout(() => {
        handleSend(query);
      }, 100);
    }
  };

  const handleSend = async (message) => {
    if (!message.trim()) return;

    const newMessage = {
      message,
      sentTime: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
      sender: "User",
      direction: "outgoing",
    };
    
    setMessages(prev => [...prev, newMessage]);
    setInputValue('');
    setIsTyping(true);

    try {
      // Updated the fetch URL to use the API_BASE_URL constant
      const response = await fetch(`${API_BASE_URL}/api/query`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ query_text: message, session_id: sessionId }),
      });

      if (!response.ok) throw new Error(`HTTP ${response.status}`);

      const data = await response.json();

      const assistantMessage = {
        message: data.answer || "I apologize, but I couldn't process that request.",
        sentTime: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
        sender: "Assistant",
        direction: "incoming",
      };
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error("Error sending message:", error);
      setMessages(prev => [...prev, {
        message: `Oops! Something went wrong: ${error.message}`,
        sentTime: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
        sender: "Assistant",
        direction: "incoming",
      }]);
    } finally {
      setIsTyping(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend(inputValue);
    }
  };

  const toggleChat = () => setIsChatOpen(!isChatOpen);

  return (
    <>
      {askAiButton.show && (
        <button
          className={styles.askButton}
          style={{ top: `${askAiButton.y}px`, left: `${askAiButton.x}px` }}
          onClick={handleAskAI}
        >
          <Sparkles size={16} />
          Ask AI
        </button>
      )}

      <div className={`${styles.chatWidgetContainer} ${isChatOpen ? styles.open : ''}`}>
        <button className={styles.chatTrigger} onClick={toggleChat}>
          {isChatOpen ? <X size={26} /> : <MessageCircle size={26} />}
        </button>

        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <h3 className={styles.headerTitle}>Humanoid Robotics Assistant</h3>
              <div className={styles.headerStatus}>
                <span className={styles.statusDot}></span>
                Online
              </div>
            </div>
          </div>

          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div 
                key={index} 
                className={`${styles.messageWrapper} ${msg.direction === 'outgoing' ? styles.outgoing : styles.incoming}`}
              >
                <div className={`${styles.message} ${msg.direction === 'outgoing' ? styles.user : styles.assistant}`}>
                  <div className={styles.messageContent}>{msg.message}</div>
                  <div className={styles.messageTime}>{msg.sentTime}</div>
                </div>
              </div>
            ))}

            {isTyping && (
              <div className={`${styles.messageWrapper} ${styles.incoming}`}>
                <div className={`${styles.message} ${styles.assistant} ${styles.typing}`}>
                  <div className={styles.typingIndicator}>
                    <span></span><span></span><span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.chatInputArea}>
            <div className={styles.inputWrapper}>
              <input
                type="text"
                placeholder="Type your message..."
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                className={styles.chatInput}
              />
              <button
                className={styles.sendButton}
                onClick={() => handleSend(inputValue)}
                disabled={!inputValue.trim()}
              >
                <Send size={20} />
              </button>
            </div>
          </div>
        </div>
      </div>
    </>
  );
};

export default ChatkitWidget;