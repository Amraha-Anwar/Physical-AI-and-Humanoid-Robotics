# Operational Checklist: Environment Readiness for API Model Integration

**Purpose**: Verify system and environment readiness before starting implementation code changes.
**Created**: 2025-12-07
**Feature**: [001-api-data-models](../spec.md)

## Local Environment

- [ ] CHK001 Verify `backend/.env` exists and is loaded by the application.
- [ ] CHK002 Verify `backend/api/query.py` exists and is importable.
- [ ] CHK003 Verify `backend/query/query_service.py` exists and is importable.
- [ ] CHK004 Verify `pytest` is installed and executable in the current environment.

## External Services (Full Stack)

- [ ] CHK005 Verify connectivity to Neon Postgres (via `NEON_DB_URL` in `.env`).
- [ ] CHK006 Verify connectivity to Qdrant Cloud (via `QDRANT_URL` and `QDRANT_API_KEY` in `.env`).
- [ ] CHK007 Verify OpenAI API Key is valid and has sufficient quota (via `OPENAI_API_KEY` in `.env`).

## Baseline Validation (Safety Protocol)

- [ ] CHK008 Run `pytest test_query.py` (or existing regression suite) to confirm current tests pass (Green state).
- [ ] CHK009 Confirm no uncommitted changes in `backend/` that might conflict with new models.
