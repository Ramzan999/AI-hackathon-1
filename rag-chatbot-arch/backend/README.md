# RAG Chatbot Backend

Backend implementation for the Physical AI & Humanoid Robotics Book RAG Chatbot using FastAPI.

## Architecture Overview
- FastAPI application with async support
- Qdrant vector database for content embeddings
- Neon Postgres for session and chat history
- OpenRouter integration for Qwen models
- Complete RAG (Retrieval-Augmented Generation) pipeline

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Copy the environment file and add your API keys:
```bash
cp .env.example .env
```

3. Update the `.env` file with your actual API keys:
   - `QDRANT_API_KEY` - Your Qdrant Cloud API key
   - `OPENROUTER_API_KEY` - Your OpenRouter API key

## Embedding Pipeline

Before running the chatbot, you need to embed the book content into Qdrant:

```bash
python run_embedding.py
```

Optional arguments:
- `--docs-dir` - Path to the docs directory (default: "../../../docs")
- `--collection-name` - Qdrant collection name (default: "book_content")

Example:
```bash
python run_embedding.py --docs-dir "../../../docs" --collection-name "book_content"
```

## Running the Application

1. Run the application:
```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

2. Test the connection:
```bash
python test_connection.py
```

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /api/chat` - Chat endpoint with RAG capabilities
- `POST /api/search` - Semantic search
- `POST /api/context` - Context processing

## Configuration

The backend is configured using environment variables defined in `.env`:
- `QDRANT_URL` - Qdrant cloud instance URL
- `QDRANT_API_KEY` - Qdrant API key
- `NEON_DATABASE_URL` - Neon Postgres connection string
- `OPENROUTER_API_KEY` - OpenRouter API key
- `OPENROUTER_BASE_URL` - OpenRouter base URL
- `QWEN_API_KEY` - Qwen-specific API key
- `CHAT_MODEL` - LLM model to use (default: qwen/qwen-2.5-72b-instruct)
- `EMBEDDING_MODEL` - Embedding model to use
- `HOST` - Host for the server (default: 0.0.0.0)
- `PORT` - Port for the server (default: 8000)

## Components

- `main.py` - Main FastAPI application with RAG implementation
- `qdrant_setup.py` - Qdrant client configuration
- `config.py` - Application configuration
- `embedding_pipeline.py` - Content embedding pipeline
- `search_service.py` - Semantic search service
- `run_embedding.py` - Script to run the embedding pipeline
- `test_connection.py` - Test script for Qdrant connection