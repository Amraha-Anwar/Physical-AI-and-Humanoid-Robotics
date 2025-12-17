# Quickstart: Verify RAG Runtime Fix

## Prerequisites
- Python 3.10+
- `backend/.env` file with valid keys:
  - `NEON_POSTGRES_CONNECTION_STRING`
  - `QDRANT_HOST` & `QDRANT_API_KEY`
  - `GEMINI_API_KEY`
  - `BASE_URL` (if using custom endpoint)

## Running the Backend

1. Navigate to backend directory:
   ```bash
   cd backend
   ```

2. Install dependencies (if needed):
   ```bash
   pip install -r requirements.txt
   ```

3. Start the server:
   ```bash
   uvicorn main:app --reload --port 8000
   ```
   *Verify that the server starts without crashing and logs "Application startup complete".*

## Verifying the Fix

1. Open a separate terminal.
2. Send a test request using curl:
   ```bash
   curl -X POST "http://localhost:8000/api/chat" \
     -H "Content-Type: application/json" \
     -d '{"message": "Hello"}'
   ```
3. Check the backend terminal logs. You should see:
   - "Request received..."
   - "Agent initialized..."
   - "Response generated..."
4. Verify the curl command returns a JSON response with the agent's answer.
