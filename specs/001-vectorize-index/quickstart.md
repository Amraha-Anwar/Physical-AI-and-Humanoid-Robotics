# P2.2 Vectorization & Indexing: OpenAI Embeddings and Dual-DB Ingestion - Quickstart Guide

This guide provides a rapid setup and execution process for the vectorization and indexing script.

## Prerequisites

*   Python 3.9+
*   Poetry (for dependency management)
*   **OpenAI API Key**: Set as an environment variable (`OPENAI_API_KEY`).
*   **P1.3 Database Setup**: Ensure Neon Postgres and Qdrant Cloud are provisioned and accessible, and their connection details are configured (as per P1.3).
*   **P2.1 Output**: A structured JSON file containing `RagChunk` objects (e.g., `processed_rag_chunks.json` from P2.1).

## 1. Setup Environment

Assuming the script will reside in `scripts/vectorizer/` within the project.

## 2. Install Dependencies

Navigate to the `scripts/vectorizer/` directory (or wherever the script will reside) and install the required Python packages:

```bash
cd scripts/vectorizer/
poetry init --no-interaction # If not already initialized
poetry add openai qdrant-client "psycopg2-binary" python-dotenv tenacity
# Ensure backend/models.py and backend/qdrant_client.py, backend/neon_client.py are accessible via PYTHONPATH
```

## 3. Configure Environment Variables

Create a `.env` file in the `scripts/vectorizer/` directory (or ensure system-wide) with the following variables:

```
OPENAI_API_KEY="your_openai_api_key"
NEON_POSTGRES_CONNECTION_STRING="your_neon_postgres_connection_string"
QDRANT_HOST="your_qdrant_cloud_host"
QDRANT_API_KEY="your_qdrant_cloud_api_key"
```

## 4. Run the Vectorization and Indexing Script

Once the script (e.g., `index_book.py`) is implemented, execute it from the project root or its designated script directory.

```bash
# Example command (actual command may vary based on script implementation)
python scripts/vectorizer/index_book.py --input-file processed_rag_chunks.json
```

## 5. Verify Ingestion

After execution, verify the data has been correctly ingested into both Qdrant and Neon Postgres.

### 5.1 Qdrant Verification

*   Access your Qdrant Cloud dashboard.
*   Verify that the `book_vectors` collection contains the expected number of points.
*   Query a sample point by its `chunk_id` to ensure the vector and payload are correct and have 1536 dimensions.

### 5.2 Neon Postgres Verification

*   Connect to your Neon Postgres database using `psql` or a GUI tool.
*   Query the `rag_metadata` table to verify that the expected number of rows have been inserted and that the metadata for sample `chunk_id`s is accurate.

### 5.3 Data Integrity Check

Compare the total count of points in Qdrant with the total number of rows in Neon Postgres. Both should match the number of `RagChunk` objects in your input JSON file.

```bash
# Example Python script for count verification
import json
from qdrant_client import QdrantClient
import psycopg2
import os

# Assuming client initialization from P1.3 modules
# from backend.qdrant_client import initialize_qdrant_client
# from backend.neon_client import initialize_neon_db

# Mock clients for demonstration if P1.3 modules are not yet integrated
qdrant_client = QdrantClient(host=os.getenv("QDRANT_HOST"), api_key=os.getenv("QDRANT_API_KEY"))
neon_conn = psycopg2.connect(os.getenv("NEON_POSTGRES_CONNECTION_STRING"))

# 1. Get count from input JSON
with open('processed_rag_chunks.json', 'r', encoding='utf-8') as f:
    input_data = json.load(f)
input_chunk_count = len(input_data)
print(f"Input JSON chunk count: {input_chunk_count}")

# 2. Get count from Qdrant
qdrant_count = qdrant_client.count(collection_name="book_vectors", exact=True).count
print(f"Qdrant collection count: {qdrant_count}")

# 3. Get count from Neon Postgres
with neon_conn.cursor() as cur:
    cur.execute("SELECT COUNT(*) FROM rag_metadata;")
    neon_count = cur.fetchone()[0]
print(f"Neon Postgres table count: {neon_count}")

# Verify integrity
if input_chunk_count == qdrant_count == neon_count:
    print("Data integrity check PASSED: Counts match across all sources!")
else:
    print("Data integrity check FAILED: Counts do NOT match.")
```

This quickstart will help confirm the script's basic functionality and data integrity across both databases.
