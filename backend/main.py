from dotenv import load_dotenv
import os
import sys
import logging

# Configure logging first to capture startup errors
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load env vars
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '.env'))

# T001: Pre-startup environment validation
required_vars = [
    "NEON_POSTGRES_CONNECTION_STRING",
    "QDRANT_HOST",
    "QDRANT_API_KEY",
    "GEMINI_API_KEY",
    "BASE_URL",
    "COHERE_API_KEY"
]

missing_vars = [var for var in required_vars if not os.getenv(var)]
if missing_vars:
    error_msg = f"CRITICAL: Missing required environment variables: {', '.join(missing_vars)}"
    logger.critical(error_msg)
    # Print to stderr to ensure visibility even if logging is misconfigured
    print(error_msg, file=sys.stderr)
    # We exit with error to prevent the server from starting in a broken state
    sys.exit(1)

from fastapi import FastAPI, Depends, HTTPException
from starlette.responses import JSONResponse
from psycopg2.extensions import connection as PgConnection
from dependencies import get_neon_db, setup_db_clients
from fastapi.middleware.cors import CORSMiddleware
from api.ingestion import router as ingestion_router
from api.query import router as query_router
from qdrant_client import QdrantClient
from dependencies import get_qdrant_client

app = FastAPI()

# T002: Simplified CORS middleware
origins = [
    "https://ai-and-robotics.vercel.app"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(ingestion_router, prefix="/api", tags=["Ingestion"])
app.include_router(query_router, prefix="/api", tags=["Query"])

# Add startup event handler
@app.on_event("startup")
async def startup_event():
    logger.info("FastAPI application startup event triggered.")
    await setup_db_clients()
    logger.info("Database clients setup complete.")

@app.get("/")
async def read_root():
    return {"message": "FastAPI application is running."}

@app.get("/db/status/neon")
async def get_neon_status(db_conn: PgConnection = Depends(get_neon_db)):
    """
    Endpoint to check the status of the Neon Postgres connection and table.
    """
    try:
        with db_conn.cursor() as cur:
            cur.execute("SELECT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name = 'rag_metadata');")
            table_exists = cur.fetchone()[0]

            if table_exists:
                return JSONResponse(status_code=200, content={"status": "ok", "database": "Neon Postgres", "table": "rag_metadata", "message": "Connected and table exists."})
            else:
                raise HTTPException(status_code=500, detail="Connected to Neon Postgres, but 'rag_metadata' table does not exist.")
    except Exception as e:
        logger.error(f"Error checking Neon DB status: {e}")
        raise HTTPException(status_code=503, detail=f"Failed to connect or verify Neon Postgres: {e}")

from qdrant_client import QdrantClient
from dependencies import get_qdrant_client

@app.get("/db/status/qdrant")
async def get_qdrant_status(qdrant_client: QdrantClient = Depends(get_qdrant_client)):
    """
    Endpoint to check the status of the Qdrant connection and collection.
    """
    collection_name = "book_vectors" # Assuming default collection name as used in initialize_qdrant_client
    try:
        # Attempt to get collection information to verify connection and existence
        collection_info = qdrant_client.get_collection(collection_name=collection_name)
        return JSONResponse(status_code=200, content={
            "status": "ok",
            "database": "Qdrant",
            "collection": collection_name,
            "message": "Connected and collection exists.",
            "collection_status": collection_info.status.value
        })
    except Exception as e:
        logger.error(f"Error checking Qdrant DB status: {e}")
        raise HTTPException(status_code=503, detail=f"Failed to connect or verify Qdrant: {e}")
