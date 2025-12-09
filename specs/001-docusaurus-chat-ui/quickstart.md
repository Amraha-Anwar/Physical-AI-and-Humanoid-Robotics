# P3.1 Docusaurus Chat Component: React UI and API Hookup - Quickstart Guide

This guide provides a rapid setup and testing process for the Docusaurus RAG Chat Component.

## Prerequisites

*   Node.js and npm/yarn (for Docusaurus development).
*   Your Docusaurus project running (`npm start` or `yarn start`).
*   The FastAPI backend (`P2.3`) up and running at a known URL (e.g., `http://localhost:8000`).
*   Backend RAG data ingested into Qdrant and Neon Postgres (`P2.2` completed).

## 1. Create the React Component File

Create the `RAGChatComponent.tsx` file in your Docusaurus project's component directory. A common location is `frontend/src/components/RAGChatComponent.tsx`.

## 2. Implement the Component (Skeleton)

Start with a basic React component structure:

```typescript jsx
import React, { useState, useEffect } from 'react';
import styles from './RAGChatComponent.module.css'; // For CSS Modules

interface ConversationMessage {
  id: string;
  sender: 'user' | 'bot';
  text: string;
  timestamp: Date;
  is_loading?: boolean;
}

const RAGChatComponent: React.FC = () => {
  const [messages, setMessages] = useState<ConversationMessage[]>([]);
  const [currentQuery, setCurrentQuery] = useState<string>('');
  const [highlightedText, setHighlightedText] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    // Implement selection change listener here
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        setHighlightedText(selection.toString());
      } else {
        setHighlightedText(null);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  const handleSendMessage = async () => {
    if (!currentQuery.trim() && !highlightedText) return;

    const userMessage: ConversationMessage = {
      id: Date.now().toString() + '-user',
      sender: 'user',
      text: currentQuery,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);
    setCurrentQuery(''); // Clear input

    const requestBody = {
      query: currentQuery,
      context_snippet: highlightedText, // Will be null if no text is highlighted
    };

    try {
      // Replace with your FastAPI backend URL
      const response = await fetch('http://localhost:8000/api/rag/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'API request failed');
      }

      const data = await response.json();
      const botMessage: ConversationMessage = {
        id: Date.now().toString() + '-bot',
        sender: 'bot',
        text: data.answer,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, botMessage]);
    } catch (err: any) {
      setError(err.message || 'An unknown error occurred');
      console.error('RAG API Error:', err);
    } finally {
      setIsLoading(false);
      setHighlightedText(null); // Clear highlighted text after sending
    }
  };

  return (
    <div className={styles.chatContainer}>
      <div className={styles.messagesDisplay}>
        {messages.map((msg) => (
          <div key={msg.id} className={`${styles.message} ${styles[msg.sender]}`}>
            <strong>{msg.sender === 'user' ? 'You' : 'Bot'}:</strong> {msg.text}
          </div>
        ))}
        {isLoading && <div className={styles.loading}>Bot is typing...</div>}
        {error && <div className={styles.error}>Error: {error}</div>}
      </div>
      {highlightedText && (
        <div className={styles.highlightIndicator}>
          Querying about: "{highlightedText.substring(0, 50)}..."
        </div>
      )}
      <div className={styles.inputArea}>
        <input
          type="text"
          value={currentQuery}
          onChange={(e) => setCurrentQuery(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
          placeholder="Ask a question about the book..."
          className={styles.textInput}
          disabled={isLoading}
        />
        <button onClick={handleSendMessage} disabled={isLoading || (!currentQuery.trim() && !highlightedText)}>
          Send
        </button>
      </div>
    </div>
  );
};

export default RAGChatComponent;
```

## 3. Create a CSS Module

Create `frontend/src/components/RAGChatComponent.module.css` for basic styling:

```css
.chatContainer {
  border: 1px solid #e0e0e0;
  border-radius: 8px;
  width: 100%;
  max-width: 600px;
  margin: 20px auto;
  display: flex;
  flex-direction: column;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
}

.messagesDisplay {
  flex-grow: 1;
  padding: 15px;
  overflow-y: auto;
  max-height: 400px; /* Adjust as needed */
  background-color: #f9f9f9;
}

.message {
  margin-bottom: 10px;
  padding: 8px 12px;
  border-radius: 6px;
  max-width: 80%;
  word-wrap: break-word;
}

.user {
  background-color: #e0f7fa; /* Light blue */
  align-self: flex-end;
  margin-left: auto;
}

.bot {
  background-color: #f1f8e9; /* Light green */
  align-self: flex-start;
}

.loading, .error {
  font-style: italic;
  color: #757575;
  padding: 8px 12px;
}

.error {
  color: #d32f2f; /* Red */
}

.highlightIndicator {
  background-color: #fff9c4; /* Light yellow */
  padding: 8px 15px;
  font-size: 0.9em;
  border-top: 1px solid #eee;
}

.inputArea {
  display: flex;
  padding: 10px 15px;
  border-top: 1px solid #e0e0e0;
  background-color: #fff;
}

.textInput {
  flex-grow: 1;
  padding: 8px;
  border: 1px solid #ccc;
  border-radius: 4px;
  margin-right: 10px;
  font-size: 1em;
}

.textInput:focus {
  outline: none;
  border-color: #2196f3; /* Blue */
}

.textInput:disabled {
  background-color: #f0f0f0;
}

button {
  background-color: #2196f3;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 8px 15px;
  cursor: pointer;
  font-size: 1em;
}

button:hover:not(:disabled) {
  background-color: #1976d2;
}

button:disabled {
  background-color: #bbdefb;
  cursor: not-allowed;
}
```

## 4. Embed the Component in an MDX Page

To test, embed `RAGChatComponent` in any of your Docusaurus MDX pages, e.g., `frontend/docs/intro/01-foundations.mdx`.

```mdx
---
title: Foundations of Physical AI
---

import RAGChatComponent from '@site/src/components/RAGChatComponent';

# Foundations of Physical AI

This is the content of your foundations document.

<RAGChatComponent />

More content here...
```

## 5. Verify API Connectivity (CORS Configuration)

During local development, you might encounter CORS errors if your Docusaurus (e.g., `localhost:3000`) and FastAPI (e.g., `localhost:8000`) run on different ports.

You may need to configure CORS in your FastAPI application (`backend/main.py`):

```python
from fastapi.middleware.cors import CORSMiddleware

# ... your existing app setup

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Adjust as needed for your Docusaurus dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ... your routes
```

## 6. Testing

*   **Docusaurus Dev Server**: Run `npm start` (or `yarn start`) in `frontend/` to view the component.
*   **FastAPI Dev Server**: Run `poetry run uvicorn main:app --reload` in `backend/` to run the API.
*   **General Query**: Type a question and send. Verify the bot responds.
*   **Highlight Query**: Highlight some text on the Docusaurus page, then type a related question and send. Verify the bot's response uses the highlighted text as primary context.

This quickstart will help you get the chat component integrated and test its core functionality.
