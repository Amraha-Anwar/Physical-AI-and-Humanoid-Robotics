import requests
import json
import uuid

# Configuration
API_URL = "http://localhost:8000/api/query"

# Payload for T055 Verification (includes the new selected_context field)
QUERY_PAYLOAD = {
   "query_text": "What are the common tasks?",
   "session_id": f"test-verify-context-{uuid.uuid4()}",
   "selected_context": "Supervised learning is a machine learning paradigm where the algorithm learns from a labeled dataset. This dataset includes input features and their corresponding correct output labels."
}

print("--- Sending T055 Verification Request (New API Contract) ---")
try:
    response = requests.post(API_URL, json=QUERY_PAYLOAD)
    
    print(f"Status Code: {response.status_code}")
    
    # Pretty print the JSON response
    response_json = response.json()
    print("Response Body:")
    print(json.dumps(response_json, indent=2))
    
    if response.status_code == 200 and 'evaluation_scores' in response_json:
        print("\n✅ Verification Successful. Please share the output with the instructor.")
    else:
        print("\n❌ Verification Failed. Check server logs for errors.")

except Exception as e:
    print(f"\n❌ Test Failed due to connection or parsing error: {e}")
    print("Ensure your FastAPI server is running (uvicorn main:app --reload) and the port is correct.")