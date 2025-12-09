# P3.3 Final Serverless Deployment: FastAPI and Docusaurus - Implementation Plan

## Overview

This plan details the final deployment strategy for the RAG chatbot application, encompassing both the FastAPI backend and the Docusaurus frontend. It covers preparing the application for a serverless environment, configuring production-grade environment variables, finalizing CORS settings, and deploying to chosen platforms (Render for FastAPI, Vercel for Docusaurus) to ensure a fully functional, scalable, and cost-effective production system.

---

### Specification: P3.3 Final Serverless Deployment: FastAPI and Docusaurus

#### 🏗️ Architecture Sketch (Deployment Model)

*   **Frontend**: Docusaurus static build deployed to a Jamstack provider (Vercel/GitHub Pages).
*   **Backend**: FastAPI application deployed as a Serverless Container or Web Service (Render/Cloud Run).
*   **Secrets**: Environment variables (`OPENAI_API_KEY`, `NEON_DB_URL`, `QDRANT_API_KEY`) are injected securely into the Backend service at runtime.
*   **CORS**: FastAPI's `CORSMiddleware` is configured to specifically allow the deployed Frontend's domain.

#### 🧱 Section Structure (Task Breakdown)
1.  **P3.3.1 Backend Deployment Preparation**: Containerize the FastAPI application (using a minimal Dockerfile) or prepare a `render.yaml` file to define the service, dependencies, and entry point.
2.  **P3.3.2 Frontend API Configuration Update**: Modify the Docusaurus frontend code (P3.1 component) to read the production API URL from a build-time environment variable (e.g., `process.env.RAG_API_URL`).
3.  **P3.3.3 Backend Serverless Deployment**: Deploy the FastAPI service to the chosen platform (e.g., Render), securely setting all required environment variables/secrets. Obtain the final public API URL.
4.  **P3.3.4 CORS Finalization**: Update the `CORSMiddleware` in `backend/main.py` to use the production domain of the deployed Docusaurus site as the *only* allowed origin, retrieved from a deployment environment variable.
5.  **P3.3.5 Frontend Static Deployment**: Deploy the Docusaurus site to the static hosting service (e.g., Vercel), injecting the final public API URL obtained in P3.3.3 as a build-time environment variable.
6.  **P3.3.6 Full End-to-End Verification**: Run the final integrity checks against the live public URL.

#### 🔬 Research Approach
*   **Serverless Cost Management**: Confirm the specific resource limits and costs associated with the chosen serverless platforms' free tiers (e.g., Cloud Run's free quota, Render's starter plan) to ensure project sustainability.
*   **Deployment Tooling**: Verify the required configuration files (`Dockerfile`, `render.yaml`, `vercel.json`, etc.) needed for zero-downtime deployment from a Git repository.

#### ✅ Quality Validation (Acceptance Criteria)
*   **API Accessibility**: The deployed FastAPI URL returns a 200 status code for a health check or the root endpoint.
*   **RAG Functionality**: The chatbot component on the public Docusaurus site successfully executes a general RAG query and returns a valid answer.
*   **Security Validation**: Test the FastAPI endpoint from an **unauthorized origin** (different domain) to confirm the specific CORS configuration (P3.3.4) correctly blocks the request.
*   **Data Integrity**: Execute the integrity check (T4.2 from P2.2) manually on the production database to confirm the data is still intact after deployment.

---

### ✍️ Decisions Needing Documentation
| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **Backend Platform** | **Render**, AWS App Runner, GCP Cloud Run | **Render** offers the simplest Git-based setup for Python/FastAPI; Cloud Run is cheaper but requires Docker setup. | **Render** (Simplest deployment path, Git-integrated). |
| **Frontend Platform** | **Vercel**, GitHub Pages | **Vercel** provides faster builds/deploy previews and easy integration; GitHub Pages is free but often requires more manual setup/CI. | **Vercel** (For ease of integration and speed). |
| **Configuration Management** | Environment Variables, Configuration Files | Environment variables (secrets) are crucial for security but complex to manage; files are simpler for static settings. | **Environment Variables** for all secrets/URLs; **Docusaurus Build Variables** for the final API URL. |

### 🧪 Testing Strategy
1.  **CORS Isolation Test**: After P3.3.4, attempt to call the deployed FastAPI endpoint from a local development environment (http://localhost:3000) and verify the request is *denied* unless localhost is explicitly added for testing purposes.
2.  **Production E2E Test**: Execute the full E2E test suite (General Query and Highlight Query) against the final public Docusaurus URL and verify that both modes work, proving the entire system is linked correctly.
3.  **Secret Integrity Check**: Verify that the deployed environment does **not** expose any sensitive API keys or database strings in logs or network responses.

---

## Constitution Check

The plan for "P3.3 Final Serverless Deployment: FastAPI and Docusaurus" aligns exceptionally well with the project's constitution (v2.0.0).

*   **I. RAG Accuracy & Contextual Relevance**: This phase ensures that the deployed system maintains the RAG accuracy and contextual relevance in a production environment, making the chatbot reliably available to users.
*   **II. Interactive UX & Selection Integrity**: The deployment ensures that the frontend, which provides the interactive UX and "Highlight-to-Query" feature, is publicly accessible and fully functional.
*   **III. Scalability & Component Reliability**: The plan explicitly focuses on deploying to serverless platforms (Render, Vercel) that support auto-scaling, scaling down to zero during idle periods, and handling traffic bursts. This directly addresses the constitutional principle of "Scalability & Component Reliability" and adherence to free/low-cost tier limits.
*   **IV. Code Integrity & Tool Adherence**: The plan leverages chosen deployment platforms (Render for FastAPI, Vercel for Docusaurus) and best practices like containerization (Dockerfile for FastAPI) and secure environment variable management, aligning with code integrity and tool adherence principles.

**Key Standards**:
*   **Embedding Model & Chunking Strategy**: Not directly applicable to deployment, but this phase ensures the backend and its underlying RAG mechanisms are live and accessible.
*   **API Security**: The plan includes a critical step for **CORS Finalization** in production (P3.3.4), ensuring the FastAPI backend's API security is hardened to accept requests only from the deployed Docusaurus domain.
*   **Deployment**: This entire phase is dedicated to fulfilling the deployment requirement, selecting appropriate serverless/static hosting platforms.
*   **Database Schema**: This phase ensures the deployed backend correctly connects to the production Neon Postgres and Qdrant databases, thereby utilizing the established schemas.

No explicit violations of the constitution were identified. The plan effectively translates the constitutional principles and project requirements into a concrete, deployable solution.

---
