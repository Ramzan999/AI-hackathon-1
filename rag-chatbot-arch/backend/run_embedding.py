"""
Script to run the complete embedding pipeline for the RAG Chatbot
"""
import os
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description="Run the RAG Chatbot embedding pipeline")
    parser.add_argument("--docs-dir", default="../../../docs", help="Path to the docs directory")
    parser.add_argument("--collection-name", default="book_content", help="Qdrant collection name")

    args = parser.parse_args()

    print("Starting RAG Chatbot embedding pipeline...")
    print(f"Using docs directory: {args.docs_dir}")
    print(f"Using collection name: {args.collection_name}")

    # Import and run the embedding pipeline
    from embedding_pipeline import BookEmbedder

    embedder = BookEmbedder()
    embedder.collection_name = args.collection_name
    embedder.process_book_content(args.docs_dir)

    print("Embedding pipeline completed successfully!")

if __name__ == "__main__":
    main()