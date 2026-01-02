"""
Simple test to verify Qdrant connection and configuration
"""
import sys
import os

# Add the parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from qdrant_client import QdrantClient
    print("[SUCCESS] Qdrant client imported successfully")

    # Test connection with the provided credentials
    QDRANT_URL = "https://a48e254c-ff19-450d-9593-2d26f17b2068.eu-west-2-0.aws.cloud.qdrant.io:6333"
    QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.APbILGTkNXjfOD2UY9S6ahan-R8l7zYElmXiKiwnps4"

    print("Attempting to connect to Qdrant...")

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    # Test the connection
    collections = qdrant_client.get_collections()
    print("[SUCCESS] Successfully connected to Qdrant!")
    print(f"Available collections: {len(collections.collections)}")

    for collection in collections.collections:
        print(f"  - {collection.name}")

    print("\n[INFO] Qdrant connection test completed successfully!")

except ImportError as e:
    print("[ERROR] Qdrant client not available in this environment")
    print(f"Error: {e}")
    print("\n[INFO] To run the full embedding pipeline, please install qdrant-client:")
    print("   pip install qdrant-client")

except Exception as e:
    print(f"[ERROR] Error connecting to Qdrant: {e}")
    print("\nThe Qdrant credentials may be invalid or the service may be unavailable.")

    # Show the configuration for reference
    print("\n[INFO] Configuration used:")
    print("URL: https://a48e254c-ff19-450d-9593-2d26f17b2068.eu-west-2-0.aws.cloud.qdrant.io:6333")
    print("API Key: [HIDDEN FOR SECURITY]")
    print("\n[INFO] Please verify your Qdrant credentials are correct.")

print("\n" + "="*60)
print("[INFO] Next Steps:")
print("1. If Qdrant connection works, you can proceed with embedding")
print("2. If not, verify your Qdrant Cloud credentials")
print("3. To run the full system, install all dependencies with:")
print("   pip install -r requirements.txt")
print("="*60)