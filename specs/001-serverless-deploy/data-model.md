# P3.3 Final Serverless Deployment: FastAPI and Docusaurus - Data Model

This document outlines the key entities and their relationships within the serverless deployment architecture for the RAG chatbot. It focuses on configuration and access patterns for the deployed frontend and backend services.

## 1. Deployed Services

### 1.1 FastAPI Backend

**Entity**: `FastAPIService`

**Description**: The deployed instance of the FastAPI application, accessible via a public URL.

**Key Attributes**:
*   `public_url` (string): The public endpoint URL where the FastAPI service is accessible (e.g., `https://rag-backend.render.com`).
*   `platform` (string): The chosen serverless platform (e.g., Render).
*   `environment_variables` (Dictionary): A collection of securely configured production environment variables, including `NEON_DB_URL`, `QDRANT_API_KEY`, `OPENAI_API_KEY`, and potentially `FRONTEND_ALLOWED_ORIGIN`.

### 1.2 Docusaurus Frontend

**Entity**: `DocusaurusSite`

**Description**: The deployed static Docusaurus website, publicly accessible via its domain.

**Key Attributes**:
*   `public_domain` (string): The primary public domain where the Docusaurus site is hosted (e.g., `https://physical-ai-robotics.vercel.app`).
*   `platform` (string): The chosen static hosting service (e.g., Vercel).
*   `build_time_variables` (Dictionary): Configuration variables injected during the build process, primarily `RAG_API_URL` (set to `FastAPIService.public_url`).

## 2. Configuration & Linkage Models

### 2.1 Environment Variables

**Entity**: `DeploymentEnvironmentVariable`

**Description**: Key-value pairs representing sensitive credentials and configuration settings injected into the deployment platforms at runtime or build-time.

**Key Attributes**:
*   `key` (string): The name of the environment variable (e.g., `OPENAI_API_KEY`).
*   `value` (string, Secret): The actual secret or configuration value.
*   `target_service` (FastAPIService | DocusaurusSite): The service where the variable is injected.

### 2.2 Cross-Origin Resource Sharing (CORS) Policy

**Entity**: `CORSPolicy`

**Description**: A security configuration on the FastAPI backend defining which origins are permitted to make cross-origin requests.

**Key Attributes**:
*   `allowed_origins` (List[string]): A list of domains explicitly permitted to access the FastAPI backend. In production, this will primarily be `DocusaurusSite.public_domain`.

## 3. Relationships

*   `DocusaurusSite` **consumes API from** `FastAPIService`: The frontend makes API calls to the backend.
*   `FastAPIService` **connects to** Neon Postgres and Qdrant (external dependencies).
*   `DeploymentEnvironmentVariable` **configures** `FastAPIService` and `DocusaurusSite`.
*   `CORSPolicy` **protects** `FastAPIService`, with `DocusaurusSite.public_domain` as an allowed origin.
```