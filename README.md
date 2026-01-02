# Physical AI & Humanoid Robotics Book with RAG Chatbot

This repository contains a comprehensive documentation site for Physical AI & Humanoid Robotics with an integrated RAG (Retrieval-Augmented Generation) chatbot system.

## Project Structure

- **Frontend**: Docusaurus-based documentation site with interactive chat functionality
- **Backend**: FastAPI server with RAG capabilities for answering questions about the robotics content
- **Database**: Qdrant vector database for semantic search
- **AI Integration**: OpenRouter API with Qwen model for intelligent responses

## Setup Instructions

### Prerequisites

- Node.js (for frontend)
- Python 3.8+ (for backend)
- Git

### Frontend Setup

1. Install dependencies:
   ```bash
   npm install
   ```

2. Start the development server:
   ```bash
   npm start
   ```
   The site will be available at `http://localhost:3000/robotics-book/`

### Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd rag-chatbot-arch/backend
   ```

2. Create a virtual environment (recommended):
   ```bash
   python -m venv venv
   # On Windows:
   venv\Scripts\activate
   # On macOS/Linux:
   source venv/bin/activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Create environment file:
   ```bash
   cp .env.example .env
   ```

5. Update the `.env` file with your actual API keys:
   ```
   QDRANT_URL=your_actual_qdrant_url
   QDRANT_API_KEY=your_actual_qdrant_api_key
   NEON_DATABASE_URL=your_actual_neon_database_url
   OPENROUTER_API_KEY=your_actual_openrouter_api_key
   QWEN_API_KEY=your_actual_qwen_api_key
   ```

6. Run the embedding pipeline to index the book content:
   ```bash
   python run_embedding.py
   ```

7. Start the backend server:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

## Security Notice

⚠️ **Important**: This project uses API keys for various services. These are not included in the repository for security reasons. You must provide your own API keys in the `.env` file.

Supported services include:
- Qdrant Cloud (vector database)
- Neon Postgres (database)
- OpenRouter (AI models)
- Qwen API

## Features

- Interactive documentation site with Docusaurus
- AI-powered chatbot that can answer questions about the robotics content
- Text selection integration with "Ask about this" functionality
- Semantic search capabilities
- Real-time communication between frontend and backend

## Architecture

The system consists of:
- Frontend: React-based Docusaurus site with custom chat component
- Backend: FastAPI server with RAG pipeline
- Vector Database: Qdrant for semantic search
- LLM Integration: OpenRouter with Qwen models
- Content: Book content processed into searchable embeddings

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
