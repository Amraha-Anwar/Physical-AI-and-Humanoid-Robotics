# P3.3 Final Serverless Deployment: FastAPI and Docusaurus - Quickstart Guide

This guide provides a rapid setup and deployment process for the FastAPI backend and Docusaurus frontend to serverless platforms, enabling end-to-end RAG chatbot functionality in a production environment.

## Prerequisites

*   **FastAPI Backend**: The backend code (`backend/`) is complete and tested locally.
*   **Docusaurus Frontend**: The frontend code (`frontend/`) is complete and tested locally.
*   **Git Repository**: Both frontend and backend code are pushed to a Git repository (e.g., GitHub), as this is typically required by deployment platforms.
*   **Platform Accounts**: Accounts for Render (or chosen FastAPI platform) and Vercel (or chosen Docusaurus platform).
*   **Production Environment Variables**:
    *   `OPENAI_API_KEY`
    *   `NEON_DB_URL` (or `NEON_POSTGRES_CONNECTION_STRING`)
    *   `QDRANT_API_KEY`

## 1. Prepare FastAPI for Deployment (Backend)

### 1.1 Add Dockerfile

Create a `Dockerfile` in your `backend/` directory for containerization. This is often required for platforms like Render.

```dockerfile
# Use an official Python runtime as a parent image
FROM python:3.9-slim-buster

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install any needed packages specified in pyproject.toml
# Ensure poetry is installed
RUN pip install poetry
# Use poetry to install dependencies
RUN poetry install --no-dev --no-root

# Expose the port FastAPI runs on
EXPOSE 8000

# Run uvicorn with Gunicorn for production
CMD ["poetry", "run", "gunicorn", "main:app", "--workers", "4", "--worker-class", "uvicorn.workers.UvicornWorker", "--bind", "0.0.0.0:8000"]
```

### 1.2 Update `main.py` for Production CORS

In `backend/main.py`, update the `CORSMiddleware` to dynamically set allowed origins based on an environment variable, which will be the deployed frontend domain.

```python
from fastapi.middleware.cors import CORSMiddleware
import os

# ... your existing app setup

# Get allowed origins from environment variable, fallback to development origin
# In production, this will be the deployed frontend URL (e.g., https://your-docusaurus-site.vercel.app)
# During local development, it might be http://localhost:3000
allowed_origins_str = os.getenv("FRONTEND_ALLOWED_ORIGIN", "http://localhost:3000")
allowed_origins = [origin.strip() for origin in allowed_origins_str.split(',') if origin.strip()]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ... your routes
```

## 2. Deploy FastAPI to Render (Backend)

1.  **Create New Web Service on Render**:
    *   Go to Render Dashboard.
    *   Connect your Git repository.
    *   Select `Python` as the runtime.
    *   **Build Command**: `pip install poetry && poetry install --no-dev` (if not using Dockerfile, adjust as needed).
    *   **Start Command**: `poetry run gunicorn main:app --workers 4 --worker-class uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000` (if not using Dockerfile).
    *   **Environment Variables**: Securely add `OPENAI_API_KEY`, `NEON_DB_URL`, `QDRANT_API_KEY`, and `FRONTEND_ALLOWED_ORIGIN` (initially set to `*` or your development domain for testing, then update to final Vercel domain later).
    *   Choose a region and plan (e.g., Free or Starter).
2.  **Deploy**: Render will build and deploy your service. Note down the public URL of your deployed FastAPI service. This will be your `BACKEND_API_URL`.

## 3. Prepare Docusaurus for Deployment (Frontend)

### 3.1 Update `RAGChatComponent.tsx` for Production API URL

Modify `frontend/src/components/RAGChatComponent.tsx` to read the API URL from an environment variable:

```typescript jsx
// ... (previous imports and component setup)

const RAGChatComponent: React.FC = () => {
  // ... (existing state and useEffect)

  // Get API URL from environment variable, fallback to local dev
  const RAG_API_URL = process.env.RAG_API_URL || 'http://localhost:8000'; // Docusaurus build env variable

  const handleSendMessage = async () => {
    // ... (existing logic)
    try {
      const response = await fetch(`${RAG_API_URL}/api/rag/query`, { // Use the dynamic URL
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

  return ( /* ... */ );
};

export default RAGChatComponent;
```

## 4. Deploy Docusaurus to Vercel (Frontend)

1.  **Create New Project on Vercel**:
    *   Go to Vercel Dashboard.
    *   Connect your Git repository.
    *   Select the `frontend/` directory as the Root Directory.
    *   **Build & Output Settings**: Vercel should automatically detect Docusaurus.
    *   **Environment Variables**: Add `RAG_API_URL` and set its value to the `BACKEND_API_URL` obtained from Render in step 2.
2.  **Deploy**: Vercel will build and deploy your Docusaurus site. Note down the public URL of your deployed frontend. This will be your `FRONTEND_DOMAIN`.

## 5. Finalize CORS on FastAPI (Backend)

Go back to your Render service dashboard for the FastAPI backend.
*   Update the `FRONTEND_ALLOWED_ORIGIN` environment variable to your actual `FRONTEND_DOMAIN` obtained from Vercel (e.g., `https://your-docusaurus-site.vercel.app`).
*   Trigger a redeploy of the FastAPI service on Render.

## 6. Full End-to-End Verification

*   Access your deployed Docusaurus site using its `FRONTEND_DOMAIN`.
*   Navigate to the page containing the RAG chat component.
*   Ask both standard queries and highlight-to-query questions.
*   Verify that the RAG chatbot is fully functional, receiving correct contextual answers.
*   Check the browser's developer console for any CORS errors or API communication issues.

This quickstart guides you through the process of deploying and integrating both parts of your RAG chatbot application.
