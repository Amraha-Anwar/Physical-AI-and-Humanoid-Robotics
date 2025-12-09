import { v4 as uuidv4 } from 'uuid';

class ChatSDK {
  constructor() {
    if (ChatSDK.instance) {
      return ChatSDK.instance;
    }

    // Only access localStorage if in browser
    this.sessionId = this.getOrCreateSessionId();
    ChatSDK.instance = this;
  }

  getOrCreateSessionId() {
    // Check if running in browser
    if (typeof window !== 'undefined') {
      let sessionId = localStorage.getItem('rag_session_id');
      if (!sessionId) {
        sessionId = uuidv4();
        localStorage.setItem('rag_session_id', sessionId);
      }
      return sessionId;
    } else {
      // Fallback for server-side: generate a temporary session ID
      return uuidv4();
    }
  }

  async sendMessage(queryText, selectedContext = null) {
    const payload = {
      query_text: queryText,
      session_id: this.sessionId,
      selected_context: selectedContext,
    };

    try {
      const response = await fetch('http://localhost:8000/api/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(`API error: ${response.status} - ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error sending message:', error);
      return { 
        answer: "Error: Could not get a response from the chatbot.", 
        session_id: this.sessionId, 
        evaluation_scores: { relevance: 0, faithfulness: 0 },
        error: error.message
      };
    }
  }
}

export default new ChatSDK();
