"""
Qdrant Client Setup for RAG Chatbot
"""
from qdrant_client import QdrantClient
from config import config

# Initialize Qdrant client with longer timeout for large uploads
qdrant_client = QdrantClient(
    url=config.QDRANT_URL,
    api_key=config.QDRANT_API_KEY,
    timeout=300  # 5 minute timeout for operations
)

# Test the connection and get collections
try:
    collections = qdrant_client.get_collections()
    print("Connected to Qdrant successfully!")
    print(f"Available collections: {collections}")

    # List all collection names
    for collection in collections.collections:
        print(f"- {collection.name}")

except Exception as e:
    print(f"Error connecting to Qdrant: {e}")

print("\nConfiguration loaded:")
print(f"Neon DB URL: {'*' * 20} (truncated for security)")
print(f"OpenRouter Base URL: {config.OPENROUTER_BASE_URL}")
print(f"Qwen API Key: {'*' * 20} (truncated for security)")