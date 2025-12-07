import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Setup configuration
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "book_vectors"

if not QDRANT_HOST or not QDRANT_API_KEY:
    print("Error: QDRANT_HOST or QDRANT_API_KEY not set in .env")
    sys.exit(1)

def delete_collection():
    print(f"Connecting to Qdrant at {QDRANT_HOST}...")
    client = QdrantClient(
        url=QDRANT_HOST,
        api_key=QDRANT_API_KEY,
        prefer_grpc=True
    )

    print(f"Checking for collection '{COLLECTION_NAME}'...")
    try:
        collections = client.get_collections()
        exists = any(c.name == COLLECTION_NAME for c in collections.collections)
        
        if exists:
            print(f"Deleting collection '{COLLECTION_NAME}'...")
            client.delete_collection(collection_name=COLLECTION_NAME)
            print("Collection deleted successfully.")
        else:
            print(f"Collection '{COLLECTION_NAME}' does not exist.")
            
    except Exception as e:
        print(f"Error during deletion: {e}")
        sys.exit(1)

if __name__ == "__main__":
    delete_collection()
