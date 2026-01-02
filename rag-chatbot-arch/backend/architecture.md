# Backend Architecture: RAG Chatbot for Docusaurus Book

## Overview
This document outlines the backend architecture for the RAG Chatbot integration into the Physical AI & Humanoid Robotics Docusaurus book. The backend is built with FastAPI and handles query processing, vector retrieval, and LLM integration.

## Components

### 1. FastAPI Application
- **Purpose**: Handle HTTP requests and coordinate all backend services
- **Features**:
  - Asynchronous request handling
  - RESTful API endpoints
  - Request validation and error handling
  - CORS configuration for frontend integration
  - Rate limiting to prevent abuse

### 2. RAG Service
- **Purpose**: Coordinate retrieval and generation processes
- **Features**:
  - Query processing and cleaning
  - Vector search and context retrieval
  - Context combination from multiple sources
  - Response generation orchestration

### 3. Vector Database Integration (Qdrant)
- **Purpose**: Store and retrieve book content embeddings
- **Features**:
  - 1024-dimensional Qwen embeddings
  - Semantic search capabilities
  - Metadata storage for content sources
  - Efficient similarity search algorithms

### 4. Database Service (Neon Postgres)
- **Purpose**: Store user sessions, chat history, and metadata
- **Features**:
  - User session management
  - Chat history persistence
  - Selected text context storage
  - Conversation threading

### 5. LLM Service (OpenRouter)
- **Purpose**: Generate responses based on retrieved context
- **Features**:
  - Integration with OpenRouter API
  - Qwen/qwen-2.5-72b-instruct model
  - Context-aware response generation
  - Token usage tracking

### 6. Embedding Service (Qwen)
- **Purpose**: Generate text embeddings for RAG process
- **Features**:
  - 1024-dimensional vector generation
  - Batch processing capabilities
  - Integration with OpenRouter or HuggingFace
  - Consistent embedding format

## API Endpoints

### Chat Endpoints
- `POST /api/chat` - Process user queries and return responses
- `POST /api/chat/stream` - Stream responses in real-time
- `GET /api/chat/history` - Retrieve conversation history
- `DELETE /api/chat/history` - Clear conversation history

### Context Endpoints
- `POST /api/context` - Process selected text and prepare context
- `POST /api/search` - Perform semantic search on book content

### Session Endpoints
- `POST /api/session` - Create new user session
- `GET /api/session/{session_id}` - Retrieve session data
- `PUT /api/session/{session_id}` - Update session data

## Technical Specifications

### FastAPI Features
- **Async Support**: Full asynchronous request handling
- **Pydantic Models**: Request/response validation
- **Dependency Injection**: Service management
- **Middleware**: Authentication, logging, error handling

### Database Schema
- **Sessions Table**: User session data
- **Conversations Table**: Chat history
- **Messages Table**: Individual messages
- **Contexts Table**: Selected text and metadata

### Performance Considerations
- **Caching**: Redis for frequently accessed content
- **Connection Pooling**: Efficient database connections
- **Background Tasks**: Asynchronous processing for heavy operations
- **Load Balancing**: Horizontal scaling support

## Security Measures
- **Rate Limiting**: Prevent API abuse
- **Authentication**: Session-based user identification
- **Input Validation**: Sanitize all user inputs
- **API Key Management**: Secure LLM and database access
- **Data Encryption**: Sensitive data protection

## Deployment Configuration
- **Environment Variables**: Secure configuration management
- **Health Checks**: Service availability monitoring
- **Logging**: Comprehensive request and error logging
- **Monitoring**: Performance and usage metrics