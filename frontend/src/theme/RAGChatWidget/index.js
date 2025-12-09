import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
import ChatSDK from './ChatSDK';

export default function RAGChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showAskButton, setShowAskButton] = useState(false);
  const [askButtonPosition, setAskButtonPosition] = useState({ x: 0, y: 0 });
  const messagesEndRef = useRef(null);

  const [selectedContext, setSelectedContext] = useState(null);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      if (selectedText.length > 0) {
        setSelectedContext(selectedText);
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        // Position the button slightly above and to the right of the selection
        setAskButtonPosition({
          x: rect.right + window.scrollX - 40, // Offset to position correctly
          y: rect.top + window.scrollY - 40,   // Offset to position correctly
        });
        setShowAskButton(true);
      } else {
        setSelectedContext(null);
        setShowAskButton(false);
      }
    };

    window.addEventListener('mouseup', handleMouseUp);
    return () => {
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: Date.now(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await ChatSDK.sendMessage(userMessage.content, selectedContext);
      
      const assistantMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: response.answer,
        timestamp: Date.now(),
        metadata: {
          evaluation_scores: response.evaluation_scores
        }
      };
      
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error("Failed to send message:", error);
      const errorMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: "Sorry, I encountered an error connecting to the server.",
        timestamp: Date.now(),
      };
       setMessages((prev) => [...prev, errorMessage]);
    } finally {
       setIsLoading(false);
    }
  };

  const handleAskButtonClick = async () => {
    if (selectedContext) {
      setIsOpen(true); // Open the chat widget
      setShowAskButton(false); // Hide the floating button
      
      const userMessage = {
        id: Date.now().toString(),
        role: 'user',
        content: selectedContext, // Use selectedContext as query
        timestamp: Date.now(),
      };

      setMessages((prev) => [...prev, userMessage]);
      setIsLoading(true);

      try {
        const response = await ChatSDK.sendMessage("Explain this based on the selected text: " + selectedContext, selectedContext); 
        
        const assistantMessage = {
          id: Date.now().toString(),
          role: 'assistant',
          content: response.answer,
          timestamp: Date.now(),
          metadata: {
            evaluation_scores: response.evaluation_scores
          }
        };
        
        setMessages((prev) => [...prev, assistantMessage]);
      } catch (error) {
        console.error("Failed to send message:", error);
        const errorMessage = {
          id: Date.now().toString(),
          role: 'assistant',
          content: "Sorry, I encountered an error connecting to the server.",
          timestamp: Date.now(),
        };
        setMessages((prev) => [...prev, errorMessage]);
      } finally {
        setIsLoading(false);
        setSelectedContext(null); // Clear selected context after sending
      }
    }
  };

  return (
    <div className={clsx(styles.chatWidgetContainer, { [styles.open]: isOpen })}>
      {showAskButton && selectedContext && (
        <button
          className={styles.askButton}
          style={{ left: askButtonPosition.x, top: askButtonPosition.y }}
          onClick={handleAskButtonClick} // Assign the new handler
        >
          {/* You might want to add some text or icon here, e.g., 'Ask AI' */}
          Ask AI
        </button>
      )}
      {/* Trigger Button */}
      <button 
        className={styles.chatTrigger} 
        onClick={toggleChat}
        aria-label="Toggle Chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>AI Assistant</h3>
          </div>
          
          <div className={styles.chatMessages}>
            {messages.map((msg) => (
              <div key={msg.id} className={clsx(styles.message, styles[msg.role])}>
                <div className={styles.messageContent}>{msg.content}</div>
                {msg.role === 'assistant' && msg.metadata?.evaluation_scores && (
                  <div className={styles.scores}>
                    <span>Relevance: {msg.metadata.evaluation_scores.relevance?.toFixed(2)}</span>
                    <span>Faithfulness: {msg.metadata.evaluation_scores.faithfulness?.toFixed(2)}</span>
                  </div>
                )}
              </div>
            ))}
            {isLoading && <div className={styles.loadingIndicator}>Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          <form className={styles.chatInputArea} onSubmit={handleSubmit}>
            {selectedContext && (
              <div className={styles.selectedContextIndicator}>
                Using context: "{selectedContext.substring(0, 50)}..."
              </div>
            )}
            <input
              type="text"

              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading || !inputValue.trim()}>
              Send
            </button>
          </form>
        </div> // <--- This closes the `chatWindow` div
      )} 
    </div> // <--- This closes the `chatWidgetContainer` div
  ); // <--- This closes the `return` statement's parenthesis
} // <--- This closes the function definition