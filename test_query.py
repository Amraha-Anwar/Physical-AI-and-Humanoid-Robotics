import requests
import json
import uuid

# Use a simple unique session ID for this test
SESSION_ID = f"eval-test-{uuid.uuid4()}" 
API_URL = "http://localhost:8000/api/query"

# Test Query Data
query_data = {
    "query_text": "What are the two common tasks in supervised learning?",
    "session_id": SESSION_ID
}

print("--- Sending Final Evaluation Query Request ---")
try:
    response = requests.post(API_URL, json=query_data)
    
    print(f"Status Code: {response.status_code}")
    
    # Pretty print the JSON response
    response_json = response.json()
    print("Response Body:")
    print(json.dumps(response_json, indent=2))
    
    if response.status_code == 200 and 'evaluation_scores' in response_json:
        print("\n✅ RAG Evaluation Verification Successful.")
    else:
        print("\n❌ Verification Failed. Check server logs for errors.")

except Exception as e:
    print(f"\n❌ Test Failed due to connection or parsing error: {e}")