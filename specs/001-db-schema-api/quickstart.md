# P1.3 Database Connection: Schema Definition & API Layer - Quickstart Guide

This guide provides a rapid setup and verification process for the database connection and API layer components.

## Prerequisites

*   Python 3.9+
*   Poetry (for dependency management)
*   FastAPI development environment set up.
*   Access to a Neon Postgres database instance and its connection string.
*   Access to a Qdrant Cloud instance and its API key.

## 1. Setup Environment Variables

Create a `.env` file in the `backend/` directory with the following variables:

```
NEON_POSTGRES_CONNECTION_STRING="your_neon_postgres_connection_string"
QDRANT_HOST="your_qdrant_cloud_host"
QDRANT_API_KEY="your_qdrant_cloud_api_key"
```

## 2. Install Dependencies

Navigate to the `backend/` directory and install the required Python packages:

```bash
cd backend/
poetry install
```

## 3. Run the FastAPI Application

Start the FastAPI application. Ensure your environment variables are loaded (e.g., by running `poetry run uvicorn main:app --reload`).

```bash
poetry run uvicorn main:app --reload
```

The application will typically be available at `http://127.0.0.1:8000`.

## 4. Verify Database Connections

Once the FastAPI application is running, use a tool like `curl` or a web browser to hit the verification endpoints:

### 4.1 Verify Neon Postgres Connection

```bash
curl http://127.0.0.1:8000/db/status/neon
```

**Expected Output**: An HTTP 200 OK response, ideally with a message indicating successful connection and `rag_metadata` table access.

### 4.2 Verify Qdrant Cloud Connection

```bash
curl http://127.0.0.1:8000/db/status/qdrant
```

**Expected Output**: An HTTP 200 OK response, ideally with a message indicating successful connection and `book_vectors` collection access.

## 5. (Optional) Manual Schema Verification

*   **Neon Postgres**: Connect to your Neon Postgres database using `psql` or a GUI tool and verify that the `rag_metadata` table structure matches the defined schema (`RagChunk` model) in `data-model.md`.
*   **Qdrant Cloud**: Access your Qdrant Cloud dashboard or use the Qdrant client directly in a Python script to verify that the `book_vectors` collection exists and has the correct vector size (1536).

This quickstart will help confirm the basic setup and connectivity of the database layer.
