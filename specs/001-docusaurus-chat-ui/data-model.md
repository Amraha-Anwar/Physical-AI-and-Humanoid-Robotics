# P3.1 Docusaurus Chat Component: React UI and API Hookup - Data Model

This document outlines the data models and state management within the Docusaurus React chat component, focusing on its interaction with the backend RAG API (P2.3).

## 1. Frontend Component State

**Entity**: `ConversationMessage`

**Description**: Represents a single message in the chat history, either from the user or the bot.

**Attributes**:
*   `id` (string): Unique identifier for the message (e.g., UUID).
*   `sender` ('user' | 'bot'): Indicates who sent the message.
*   `text` (string): The content of the message.
*   `timestamp` (Date): When the message was sent/received.
*   `is_loading` (boolean, Optional): True if the bot response is pending (for bot messages).

**TypeScript Interface (Conceptual)**:

```typescript
interface ConversationMessage {
  id: string;
  sender: 'user' | 'bot';
  text: string;
  timestamp: Date;
  is_loading?: boolean;
}
```

**Entity**: `ChatState`

**Description**: The overall state managed by the React component.

**Attributes**:
*   `messages` (ConversationMessage[]): An array of `ConversationMessage` objects, representing the chat history.
*   `currentQuery` (string): The text currently typed by the user in the input field.
*   `highlightedText` (string | null): The text currently highlighted on the document, if any.
*   `isLoading` (boolean): Indicates if an API call is in progress.
*   `error` (string | null): Any error message to display.

## 2. API Interaction Models (Frontend Perspective)

These models correspond to the backend API contracts defined in P2.3, translated to TypeScript for frontend use.

### 2.1 Request Model: `RAGQueryRequest`

**Entity**: `RAGQueryRequest`

**Description**: The data structure sent from the frontend to the `/api/rag/query` FastAPI endpoint.

**Attributes**:
*   `query` (string): The user's question.
*   `context_snippet` (string, Optional): The highlighted text from the document, if available.

**TypeScript Interface (Conceptual)**:

```typescript
interface RAGQueryRequest {
  query: string;
  context_snippet?: string | null;
}
```

### 2.2 Response Model: `RAGQueryResponse`

**Entity**: `RAGQueryResponse`

**Description**: The data structure received from the `/api/rag/query` FastAPI endpoint.

**Attributes**:
*   `answer` (string): The contextual answer from the RAG system.
*   `sources` (SourceMetadata[], Optional): Metadata about the source chunks used for the answer.

**TypeScript Interface (Conceptual)**:

```typescript
interface RAGQueryResponse {
  answer: string;
  sources?: SourceMetadata[];
}

interface SourceMetadata {
  chunk_id: string;
  chapter_title: string;
  page_number: number;
}
```
