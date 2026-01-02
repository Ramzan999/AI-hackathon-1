# RAG Chatbot Architecture

Complete implementation for the RAG Chatbot integration with the Physical AI & Humanoid Robotics Docusaurus book.

## Structure
- `frontend/` - Frontend components and documentation
- `backend/` - Backend services, API, and embedding pipeline
- `docs/` - Integration guides and documentation

## Overview
This architecture implements a complete Retrieval-Augmented Generation (RAG) chatbot that allows users to ask questions about the book content with support for selected text context. The system uses:

- **Frontend**: React-based floating chat component integrated with Docusaurus
- **Backend**: FastAPI application with asyncio support
- **Vector DB**: Qdrant Cloud for content embeddings
- **Database**: Neon Postgres for session and history management
- **LLM**: OpenRouter with Qwen/qwen-2.5-72b-instruct
- **Embeddings**: Sentence Transformer model for semantic search

## Features
- 🤖 Context-aware chat with book content
- 🔍 Semantic search through book content
- ✍️ Text selection integration ("Ask about this" feature)
- 📚 Complete embedding pipeline for book content
- 🌐 Docusaurus integration ready
- 📱 Responsive floating chat interface

## Getting Started

### Prerequisites
- Python 3.8+
- Access to Qdrant Cloud
- OpenRouter API key with Qwen model access

### Quick Start

1. **Setup Backend**:
   ```bash
   cd rag-chatbot-arch/backend
   pip install -r requirements.txt
   cp .env.example .env
   # Edit .env with your API keys
   ```

2. **Run the Complete System**:
   ```bash
   # From the rag-chatbot-arch directory
   python run_system.py
   ```

   Or run components separately:

   a. **Index Book Content** (one-time setup):
   ```bash
   cd backend
   python run_embedding.py
   ```

   b. **Start Backend Server**:
   ```bash
   cd backend
   uvicorn main:app --reload
   ```

### Backend Setup (Detailed)
1. Navigate to the `backend/` directory
2. Install dependencies: `pip install -r requirements.txt`
3. Configure environment variables in `.env`
4. Run the embedding pipeline: `python run_embedding.py`
5. Start the application: `uvicorn main:app --reload`

### Frontend Integration
1. Navigate to the `frontend/` directory
2. Review the `ChatComponent.jsx` for integration details
3. Integrate with Docusaurus using theme wrapper
4. The component provides text selection and context awareness

### API Endpoints
- `GET /` - Health check
- `POST /api/chat` - Chat with RAG capabilities
- `POST /api/search` - Semantic search
- `POST /api/context` - Context processing

## Architecture Components

### Backend
- **FastAPI** - Web framework with async support
- **Qdrant Client** - Vector database for semantic search
- **OpenRouter** - LLM integration with Qwen models
- **Embedding Pipeline** - Process and index book content
- **Search Service** - Semantic search capabilities

### Frontend
- **React Component** - Floating chat interface
- **Text Selection** - Capture and use selected text as context
- **Session Management** - Maintain conversation history

## Configuration
All configuration is managed through environment variables in the `.env` file:
- Qdrant connection settings
- OpenRouter API keys and model selection
- Database connection strings
- Server settings

## Full Documentation
1. Review the [Integration Guide](docs/integration-guide.md)
2. Examine [Frontend Architecture](frontend/architecture.md)
3. Study [Backend Architecture](backend/architecture.md)
4. Check backend README for detailed setup instructions