"""
Search Service for RAG Chatbot
Handles semantic search queries against embedded book content
"""
from typing import List, Dict, Any
from qdrant_client.http import models
from qdrant_setup import qdrant_client

class SearchService:
    def __init__(self, collection_name: str = "book_content"):
        self.collection_name = collection_name

    def semantic_search(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Perform semantic search on the embedded book content
        """
        try:
            # Import the embedding model to convert query to vector
            from sentence_transformers import SentenceTransformer
            model = SentenceTransformer('all-MiniLM-L6-v2')  # Same model used for embeddings

            # Convert query text to embedding vector
            query_vector = model.encode([query])[0].tolist()

            # Search in Qdrant using the vector - using the query_points method which should be available
            search_results = qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                with_payload=True
            )

            results = []
            for hit in search_results.points:
                results.append({
                    'id': hit.id,
                    'content': hit.payload.get('content', ''),
                    'source_file': hit.payload.get('source_file', ''),
                    'section_title': hit.payload.get('section_title', ''),
                    'score': hit.score,
                    'metadata': hit.payload.get('metadata', {})
                })

            return results

        except Exception as e:
            print(f"Error in semantic search: {e}")
            import traceback
            traceback.print_exc()
            return []

    def search_with_selected_text(self, query: str, selected_text: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search considering both the query and selected text as context
        """
        # Combine query and selected text for better context
        if selected_text:
            combined_query = f"{query} Context: {selected_text}"
        else:
            combined_query = query

        return self.semantic_search(combined_query, top_k)

    def get_relevant_context(self, query: str, selected_text: str = None, top_k: int = 3) -> str:
        """
        Get relevant context as a string for LLM consumption
        """
        results = self.search_with_selected_text(query, selected_text, top_k)

        context_parts = []
        for result in results:
            context_parts.append(f"Section: {result['section_title']}\nContent: {result['content'][:500]}...")  # Limit content length

        return "\n\n".join(context_parts)

    def find_by_source(self, source_file: str) -> List[Dict[str, Any]]:
        """
        Find all content from a specific source file
        """
        try:
            search_results = qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_file",
                            match=models.MatchValue(value=source_file)
                        )
                    ]
                ),
                limit=100,
                with_payload=True
            )

            results = []
            points, next_page = search_results
            for point in points:
                results.append({
                    'id': point.id,
                    'content': point.payload.get('content', ''),
                    'source_file': point.payload.get('source_file', ''),
                    'section_title': point.payload.get('section_title', ''),
                    'metadata': point.payload.get('metadata', {})
                })

            return results

        except Exception as e:
            print(f"Error finding by source: {e}")
            return []

# Global instance
search_service = SearchService()

def get_search_service() -> SearchService:
    """Get the global search service instance"""
    return search_service