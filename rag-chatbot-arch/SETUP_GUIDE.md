# RAG Chatbot Setup Guide

This guide provides detailed instructions for setting up and running the RAG Chatbot system for the Physical AI & Humanoid Robotics book.

## API Keys and Configuration

The system requires the following API keys and configuration:

### Qdrant Configuration
- **URL**: `your-qdrant-url`
- **API Key**: `your-qdrant-api-key`

### Neon Database URL
- `your-neon-database-url`

### OpenRouter Configuration
- **API Key**: `your-openrouter-api-key`
- **Base URL**: `https://openrouter.ai/api/v1`

### Qwen API Key
- `your-qwen-api-key`

⚠️ **Important Security Notice**: These API keys should be stored securely and never committed to version control. In a production environment, use environment variables or a secure secrets management system.

## Required Dependencies

The system requires the following Python packages:

- `fastapi==0.104.1`
- `uvicorn[standard]==0.24.0`
- `qdrant-client==1.7.3`
- `sqlalchemy==2.0.23`
- `asyncpg==0.29.0`
- `pydantic==2.5.0`
- `python-dotenv==1.0.0`
- `openai==1.3.5`
- `tiktoken==0.5.2`
- `python-multipart==0.0.6`
- `cryptography==41.0.8`
- `psycopg2-binary==2.9.9`
- `sentence-transformers==2.7.0`
- `transformers==4.35.0`
- `torch==2.1.0`
- `numpy==1.24.3`

## Setup Instructions

### 1. Environment Setup
```bash
# Create a virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
cd rag-chatbot-arch/backend
pip install -r requirements.txt
```

### 2. Configuration
1. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

2. Update the `.env` file with your actual API keys:
   ```
   QDRANT_URL=https://a48e254c-ff19-450d-9593-2d26f17b2068.eu-west-2-0.aws.cloud.qdrant.io:6333
   QDRANT_API_KEY=your_actual_qdrant_api_key
   NEON_DATABASE_URL=your_neon_database_url
   OPENROUTER_API_KEY=your_openrouter_api_key
   QWEN_API_KEY=your_qwen_api_key
   ```

### 3. Embedding Pipeline
Run the embedding pipeline to index the book content:

```bash
python run_embedding.py
```

This will:
- Extract content from the docs directory
- Create embeddings using sentence transformers
- Store the embeddings in Qdrant for semantic search

### 4. Running the Backend Server
```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 5. Running the Complete System
Use the automated script to run the entire system:

```bash
# From the rag-chatbot-arch directory
python run_system.py
```

## Frontend Integration

The frontend component (`frontend/ChatComponent.jsx`) can be integrated with Docusaurus:

1. Create a custom theme component in `src/theme/ChatWrapper/index.js`
2. Wrap the main layout with the chat component
3. The component will automatically detect text selection and provide context

## API Endpoints

- `GET /` - Health check
- `POST /api/chat` - Chat with RAG capabilities
- `POST /api/search` - Semantic search
- `POST /api/context` - Context processing

## Troubleshooting

### Common Issues:

1. **Package Installation Issues**:
   - If you encounter issues with installing certain packages (especially torch, numpy), try:
     ```bash
     pip install --upgrade pip
     pip install --user [package-name]
     ```

2. **Qdrant Connection Issues**:
   - Verify your Qdrant URL and API key are correct
   - Check that your Qdrant Cloud instance is running
   - Ensure proper network connectivity

3. **Embedding Pipeline Issues**:
   - Make sure the docs directory path is correct
   - Verify sufficient memory for embedding large documents

4. **API Connection Issues**:
   - Check that your OpenRouter API key has access to the required models
   - Verify your Qwen model access

## Security Considerations

- Store API keys in environment variables, not in code
- Use HTTPS for all API communications
- Implement proper rate limiting
- Validate and sanitize all user inputs
- Regularly rotate API keys

## Performance Optimization

- Use batching for embedding large documents
- Implement caching for frequently accessed content
- Optimize vector search parameters
- Monitor and optimize database queries