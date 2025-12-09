# P3.3 Final Serverless Deployment: FastAPI and Docusaurus - Tasks

**Feature Branch**: `001-serverless-deploy`  
**Goal**: Deploy the FastAPI RAG backend to Render and the Docusaurus frontend to Vercel, securely linking them via environment variables and public URLs, ensuring a fully functional, scalable, and cost-effective production system.

---

## Phase 1: Backend Deployment Preparation (US1 & US5)

*   - [ ] T001 [US1] Create a `Dockerfile` for the FastAPI application in the `backend/` directory, ensuring it includes poetry installation and Gunicorn for production.
*   - [ ] T002 [US1] Ensure `backend/pyproject.toml` (or `requirements.txt`) lists all production dependencies for the FastAPI application.
*   - [ ] T003 [US5] Update `backend/main.py` to configure `CORSMiddleware` to read `FRONTEND_ALLOWED_ORIGIN` from environment variables, with a safe default for local development (e.g., `http://localhost:3000`).
*   - [ ] T004 [US1] Create a `render.yaml` file in the project root defining the FastAPI web service, linking to the Dockerfile and specifying environment variables (`OPENAI_API_KEY`, `NEON_DB_URL`, `QDRANT_API_KEY`, `FRONTEND_ALLOWED_ORIGIN`).

## Phase 2: Backend Deployment (US1)

*   - [ ] T005 [US1] Deploy the FastAPI service to Render, connecting to the Git repository and selecting the `render.yaml` blueprint (or configuring directly via UI).
*   - [ ] T006 [US1] Securely configure `OPENAI_API_KEY`, `NEON_DB_URL`, `QDRANT_API_KEY`, and `FRONTEND_ALLOWED_ORIGIN` (initially set to `*` for broad testing, will be refined in T013) as environment variables in the Render dashboard.
*   - [ ] T007 [US1] Obtain and record the public URL of the deployed FastAPI service (e.g., `https://rag-backend.onrender.com`).

## Phase 3: Frontend Deployment Preparation (US2)

*   - [ ] T008 [US2] Modify `frontend/src/components/RAGChatComponent.tsx` (P3.1) to read the backend API URL from a build-time environment variable (e.g., `process.env.RAG_API_URL`) instead of a hardcoded local proxy.
*   - [ ] T009 [US2] Configure `frontend/docusaurus.config.js` or create a `vercel.json` file (if using Vercel) to correctly build the Docusaurus site and prepare for Vercel deployment.

## Phase 4: Frontend Deployment (US2)

*   - [ ] T010 [US2] Deploy the Docusaurus site to Vercel, connecting to the Git repository and specifying the `frontend/` directory as the build root.
*   - [ ] T011 [US2] Set `RAG_API_URL` as a build-time environment variable in Vercel, using the public URL obtained from Render (T007).
*   - [ ] T012 [US2] Obtain and record the public URL (domain) of the deployed Docusaurus frontend (e.g., `https://your-docusaurus-site.vercel.app`).

## Phase 5: CORS Finalization (US5)

*   - [ ] T013 [US5] Update the `FRONTEND_ALLOWED_ORIGIN` environment variable in the Render dashboard for the FastAPI service (T006) to the exact public domain of the deployed Docusaurus frontend (T012).
*   - [ ] T014 [US5] Trigger a redeploy of the FastAPI service on Render to apply the updated CORS configuration.

## Phase 6: Full End-to-End Verification (US3 & US4 & US5)

*   - [ ] T015 [US3] Access the deployed Docusaurus site (T012) and perform a full functional test of the RAG chatbot (standard query and highlight-to-query) to verify end-to-end integration.
*   - [ ] T016 [US4] Monitor Render logs and metrics during idle periods to verify the FastAPI service scales down to zero/near-zero instances and adheres to cost-efficiency.
*   - [ ] T017 [US4] Perform a load test (simulated burst of 5-10 concurrent queries) against the deployed RAG chatbot to verify proper scaling behavior without errors.
*   - [ ] T018 [US5] Attempt to make an API call to the deployed FastAPI backend (T007) from an unauthorized origin (e.g., a local script on a different port/domain) and verify the request is explicitly blocked by CORS.

## Phase 7: Polish & Cross-Cutting Concerns

*   - [ ] T019 Update project `README.md` with deployment instructions and links to the live frontend and backend services.
*   - [ ] T020 Document administrative access details for Render and Vercel dashboards.

---

## Dependencies

This section outlines the completion order of user stories.

1.  **Phase 1 (Backend Deployment Preparation)** -> **Phase 2 (Backend Deployment)** -> **Phase 3 (Frontend Deployment Preparation)** -> **Phase 4 (Frontend Deployment)** -> **Phase 5 (CORS Finalization)** -> **Phase 6 (Full End-to-End Verification)**

## Parallel Execution Opportunities

*   **Phase 1 (Backend Prep)** and **Phase 3 (Frontend Prep)** can be worked on concurrently up to the point where the public URLs are needed from each other.
*   Within **Phase 6 (Verification)**, tasks T016, T017, and T018 can be executed in parallel or iteratively as part of a continuous monitoring strategy.

## Implementation Strategy

The deployment will follow a staged approach:
1.  Prepare and deploy the backend service first, obtaining its public URL.
2.  Prepare and deploy the frontend, configuring it with the backend's URL.
3.  Finalize security configurations (CORS) on the backend using the deployed frontend's URL.
4.  Conduct comprehensive end-to-end verification and monitoring to ensure full functionality, scalability, and security.
