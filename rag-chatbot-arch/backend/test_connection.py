"""
Test script to verify Qdrant connection
"""
from qdrant_setup import qdrant_client

def test_qdrant_connection():
    """
    Test the connection to Qdrant
    """
    try:
        # Get collection information
        collections = qdrant_client.get_collections()
        print("✅ Successfully connected to Qdrant!")
        print(f"📊 Available collections: {len(collections.collections)}")

        for collection in collections.collections:
            print(f"   - {collection.name}")

            # Get collection info
            try:
                info = qdrant_client.get_collection(collection.name)
                print(f"     Points: {info.points_count}, Vector size: {info.config.params.vectors.size}")
            except Exception as e:
                print(f"     Could not get details: {e}")

        return True
    except Exception as e:
        print(f"❌ Failed to connect to Qdrant: {e}")
        return False

if __name__ == "__main__":
    print("Testing Qdrant connection...")
    success = test_qdrant_connection()

    if success:
        print("\n🎉 Qdrant connection test passed!")
    else:
        print("\n💥 Qdrant connection test failed!")