# P3.3 Final Serverless Deployment: FastAPI and Docusaurus - Research Findings

This document captures key research findings and decisions made during the planning phase for the final serverless deployment.

## Decisions Documented

| Decision | Options | Trade-offs | Chosen Option (Working Assumption) |
| :--- | :--- | :--- | :--- |
| **Backend Platform** | **Render**, AWS App Runner, GCP Cloud Run | **Render** offers the simplest Git-based setup for Python/FastAPI; Cloud Run is cheaper but requires Docker setup. | **Render** (Simplest deployment path, Git-integrated). |
| **Frontend Platform** | **Vercel**, GitHub Pages | **Vercel** provides faster builds/deploy previews and easy integration; GitHub Pages is free but often requires more manual setup/CI. | **Vercel** (For ease of integration and speed). |
| **Configuration Management** | Environment Variables, Configuration Files | Environment variables (secrets) are crucial for security but complex to manage; files are simpler for static settings. | **Environment Variables** for all secrets/URLs; **Docusaurus Build Variables** for the final API URL. |
