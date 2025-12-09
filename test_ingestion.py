import requests
import json

# The sample data provided by the CLI
sample_data = {
    "document_id": "example-book-123",
    "chapters": [
        {
            "chapter_title": "Introduction to AI",
            "sections": [
                {
                    "section_title": "What is AI?",
                    "text": "Artificial intelligence (AI) is intelligence demonstrated by machines, as opposed to the natural intelligence displayed by animals including humans. Leading AI textbooks define the field as the study of 'intelligent agents': any device that perceives its environment and takes actions that maximize its chance of successfully achieving its goals. Colloquially, the term 'artificial intelligence' is often used to describe machines that mimic 'cognitive' functions that humans associate with other human minds, such.g. 'learning' and 'problem solving'."
                },
                {
                    "section_title": "History of AI",
                    "text": "The history of AI began in antiquity, with myths, stories and rumors of artificial beings endowed with intelligence or consciousness by master craftsmen. The seed of modern AI was planted by classical philosophers who attempted to describe the process of human thinking as the mechanical manipulation of symbols. This work culminated in the invention of the programmable digital computer in the 1940s: a machine based on the abstract essence of mathematical reasoning. This device, and the foundational ideas behind it, inspired a small group of scientists to begin seriously considering the possibility of building an electronic brain."
                }
            ]
        },
        {
            "chapter_title": "Machine Learning Basics",
            "sections": [
                {
                    "section_title": "Supervised Learning",
                    "text": "Supervised learning is a machine learning paradigm where the algorithm learns from a labeled dataset. This dataset includes input features and their corresponding correct output labels. The algorithm's goal is to learn a mapping function from the input to the output so that it can accurately predict the output for new, unseen inputs. Common tasks include classification (predicting a category) and regression (predicting a numerical value)."
                }
            ]
        }
    ]
}

print("--- Sending Ingestion Request ---")
try:
    response = requests.post(
        "http://localhost:8000/api/ingestion",
        json=sample_data,
        headers={"Content-Type": "application/json"}
    )

    print(f"\nStatus Code: {response.status_code}")
    print("Response Body:")
    print(json.dumps(response.json(), indent=2))

    # Check for success and prompt verification steps
    if response.status_code == 200:
        print("\n✅ API Call Successful. Proceeding to Manual DB Check.")
    else:
        print(f"\n❌ API Call Failed. Status: {response.status_code}. Please provide the full error traceback from the Uvicorn terminal.")

except requests.exceptions.ConnectionError:
    print("\n❌ Connection Failed. Ensure your FastAPI server is running at http://localhost:8000.")