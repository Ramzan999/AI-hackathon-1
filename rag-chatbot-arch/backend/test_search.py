"""
Test script to verify the search functionality
"""
from search_service import SearchService
from qdrant_setup import qdrant_client

def test_search():
    print("Testing search functionality...")

    # Create search service instance
    search_service = SearchService()

    # Test a simple search
    query = "humanoid robotics"
    print(f"Searching for: '{query}'")

    results = search_service.semantic_search(query, top_k=5)

    print(f"Found {len(results)} results")

    if results:
        for i, result in enumerate(results):
            print(f"\nResult {i+1}:")
            print(f"  Score: {result['score']}")
            print(f"  Section: {result['section_title']}")
            print(f"  Content preview: {result['content'][:100]}...")
    else:
        print("No results found. Checking if collection has data...")

        # Check collection info
        collection_info = qdrant_client.get_collection("book_content")
        print(f"Collection points count: {collection_info.points_count}")

        # Try to get a sample point to verify the data
        if collection_info.points_count > 0:
            sample_points = qdrant_client.scroll(
                collection_name="book_content",
                limit=1,
                with_payload=True,
                with_vectors=False
            )
            points, _ = sample_points
            if points:
                point = points[0]
                print(f"Sample point ID: {point.id}")
                print(f"Sample payload keys: {list(point.payload.keys())}")
                print(f"Sample content preview: {str(point.payload.get('content', ''))[:100]}...")

if __name__ == "__main__":
    test_search()