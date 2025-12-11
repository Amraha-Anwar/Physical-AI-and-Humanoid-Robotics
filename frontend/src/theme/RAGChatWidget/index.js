import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import { MessageCircle, X, Sparkles } from 'lucide-react';
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
    const handleMouseUp = (e) => {
      // Small delay to ensure selection is complete
      setTimeout(() => {
        const selection = window.getSelection();
        const selectedText = selection.toString().trim();

        console.log('Selected text:', selectedText); // Debug log

        if (selectedText.length > 5) { // Minimum 5 characters
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          
          console.log('Selection rect:', rect); // Debug log
          
          setSelectedContext(selectedText);
          setAskButtonPosition({
            x: rect.left + window.scrollX + (rect.width / 2) - 60,
            y: rect.top + window.scrollY - 60,
          });
          setShowAskButton(true);
        }
      }, 100);
    };

    const handleSelectionChange = () => {
      const selection = window.getSelection();
      if (!selection.toString().trim()) {
        // Don't hide immediately, let user click the button
        setTimeout(() => {
          const selection2 = window.getSelection();
          if (!selection2.toString().trim()) {
            setShowAskButton(false);
          }
        }, 200);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('selectionchange', handleSelectionChange);
    
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    const currentContext = selectedContext; // Save context before clearing

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
      const response = await ChatSDK.sendMessage(userMessage.content, currentContext);
      
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
      setSelectedContext(null); // Clear context after message is sent
    }
  };

  const handleAskButtonClick = async () => {
    if (selectedContext) {
      setIsOpen(true);
      setShowAskButton(false);
      
      const promptText = `Explain this: ${selectedContext}`;
      
      const userMessage = {
        id: Date.now().toString(),
        role: 'user',
        content: promptText,
        timestamp: Date.now(),
      };

      setMessages((prev) => [...prev, userMessage]);
      setIsLoading(true);

      try {
        const response = await ChatSDK.sendMessage(promptText, selectedContext); 
        
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
        setSelectedContext(null);
      }
    }
  };

  return (
    <div className={clsx(styles.chatWidgetContainer, { [styles.open]: isOpen })}>
      {showAskButton && selectedContext && (
        <button
          className={styles.askButton}
          style={{ left: askButtonPosition.x, top: askButtonPosition.y }}
          onClick={handleAskButtonClick}
        >
          <Sparkles size={16} />
          Ask AI
        </button>
      )}
      
      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>AI Assistant</h3>
            <button 
              className={styles.chatCloseButton}
              onClick={toggleChat}
              aria-label="Close Chat"
            >
              <X size={20} />
            </button>
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
            <div className={styles.chatInputRow}>
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
            </div>
          </form>
        </div>
      )}

      {/* Trigger Button - Only visible when chat is closed */}
      {!isOpen && (
        <button 
          className={styles.chatTrigger} 
          onClick={toggleChat}
          aria-label="Open Chat"
        >
          <MessageCircle size={28} />
        </button>
      )}
    </div>
  );
}