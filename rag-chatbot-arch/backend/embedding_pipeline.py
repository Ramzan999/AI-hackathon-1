"""
Embedding Pipeline for RAG Chatbot
This script processes book content and creates embeddings for Qdrant storage
"""
import asyncio
import hashlib
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import os
import json
import glob

from qdrant_setup import qdrant_client

class BookEmbedder:
    def __init__(self):
        # Initialize the embedding model (using a multilingual model that works well with technical content)
        self.model = SentenceTransformer('all-MiniLM-L6-v2')  # Lightweight model, can be changed to a Qwen-based model
        self.collection_name = "book_content"

    def create_collection(self):
        """Create Qdrant collection for book content"""
        try:
            # Delete collection if it exists (for re-indexing)
            qdrant_client.delete_collection(self.collection_name)
            print(f"Deleted existing collection: {self.collection_name}")
        except:
            pass  # Collection doesn't exist, which is fine

        # Create new collection
        qdrant_client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=384,  # Size of all-MiniLM-L6-v2 embeddings, change if using different model
                distance=models.Distance.COSINE
            )
        )
        print(f"Created collection: {self.collection_name}")

    def extract_content_from_docs(self, docs_dir: str) -> List[Dict[str, Any]]:
        """Extract content from Docusaurus docs directory"""
        content_list = []

        # Find all markdown files in docs directory
        for md_file in glob.glob(f"{docs_dir}/**/*.md", recursive=True):
            if "node_modules" in md_file or ".git" in md_file:
                continue

            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Basic markdown parsing to extract sections
                sections = self._split_content(content, md_file)

                for i, section in enumerate(sections):
                    content_list.append({
                        'id': hashlib.md5(f"{md_file}_{i}".encode()).hexdigest(),
                        'content': section['text'],
                        'source_file': md_file,
                        'section_title': section['title'],
                        'metadata': {
                            'file_path': md_file,
                            'section': section['title'],
                            'char_count': len(section['text'])
                        }
                    })
            except Exception as e:
                print(f"Error processing {md_file}: {e}")

        return content_list

    def _split_content(self, content: str, filename: str) -> List[Dict[str, str]]:
        """Split content into sections based on headers"""
        sections = []
        lines = content.split('\n')

        current_title = "Introduction"
        current_content = []

        for line in lines:
            # Check if this is a header line (starts with #)
            if line.strip().startswith('#'):
                # Save the previous section if it exists
                if current_content and any(c.strip() for c in current_content):
                    sections.append({
                        'title': current_title.strip('# ').strip(),
                        'text': '\n'.join(current_content).strip()
                    })

                # Start a new section
                current_title = line.strip('# ').strip()
                current_content = []
            else:
                current_content.append(line)

        # Add the last section
        if current_content and any(c.strip() for c in current_content):
            sections.append({
                'title': current_title.strip('# ').strip(),
                'text': '\n'.join(current_content).strip()
            })

        # Filter out very short sections
        sections = [s for s in sections if len(s['text']) > 50]

        return sections

    def create_embeddings(self, content_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Create embeddings for content list"""
        print(f"Creating embeddings for {len(content_list)} content sections...")

        # Extract just the text content for embedding
        texts = [item['content'] for item in content_list]

        # Create embeddings
        embeddings = self.model.encode(texts)

        # Combine content with embeddings
        result = []
        for i, item in enumerate(content_list):
            result.append({
                'id': item['id'],
                'content': item['content'],
                'source_file': item['source_file'],
                'section_title': item['section_title'],
                'vector': embeddings[i].tolist(),  # Convert numpy array to list
                'metadata': item['metadata']
            })

        print(f"Created embeddings for {len(result)} sections")
        return result

    def store_embeddings(self, embeddings_list: List[Dict[str, Any]]):
        """Store embeddings in Qdrant"""
        print(f"Storing {len(embeddings_list)} embeddings in Qdrant...")

        # Prepare points for Qdrant
        points = []
        for item in embeddings_list:
            points.append(
                models.PointStruct(
                    id=item['id'],
                    vector=item['vector'],
                    payload={
                        'content': item['content'],
                        'source_file': item['source_file'],
                        'section_title': item['section_title'],
                        'metadata': item['metadata']
                    }
                )
            )

        # Upload in smaller batches with timeout handling
        batch_size = 25  # Reduced batch size to avoid timeouts
        import time
        import random

        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            try:
                # Set a longer timeout for the Qdrant client for this operation
                from qdrant_client.http import models as rest
                from qdrant_client import QdrantClient

                # Use the http API directly with proper timeout handling
                qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=batch
                )
                print(f"Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")

                # Add a small delay between batches to avoid overwhelming the server
                time.sleep(0.5 + random.uniform(0, 0.5))  # 0.5-1 second delay
            except Exception as e:
                print(f"Error uploading batch {i//batch_size + 1}: {str(e)}")
                print("Retrying batch...")
                try:
                    time.sleep(5)  # Wait 5 seconds before retry
                    qdrant_client.upsert(
                        collection_name=self.collection_name,
                        points=batch
                    )
                    print(f"Retried and uploaded batch {i//batch_size + 1}")
                except Exception as retry_error:
                    print(f"Retry failed for batch {i//batch_size + 1}: {str(retry_error)}")
                    print("Continuing with next batch...")

        print(f"Successfully stored {len(embeddings_list)} embeddings in Qdrant")

    def process_book_content(self, docs_dir: str):
        """Complete pipeline: extract content -> create embeddings -> store in Qdrant"""
        print("Starting book content embedding pipeline...")

        # Create collection
        self.create_collection()

        # Extract content
        content_list = self.extract_content_from_docs(docs_dir)
        print(f"Extracted {len(content_list)} content sections from {docs_dir}")

        # Create embeddings
        embeddings_list = self.create_embeddings(content_list)

        # Store in Qdrant
        self.store_embeddings(embeddings_list)

        print("Embedding pipeline completed successfully!")

# Function to run the embedding pipeline
def run_embedding_pipeline():
    """Run the complete embedding pipeline"""
    # Path to the docs directory
    docs_dir = "../../../docs"  # Relative to the rag-chatbot-arch/backend directory

    embedder = BookEmbedder()
    embedder.process_book_content(docs_dir)

if __name__ == "__main__":
    print("Starting the RAG Chatbot embedding pipeline...")
    run_embedding_pipeline()