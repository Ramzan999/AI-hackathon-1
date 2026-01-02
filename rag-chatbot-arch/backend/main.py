"""
FastAPI Backend for RAG Chatbot
"""
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Optional
import asyncio
import os
from openai import OpenAI

from config import config  # pyright: ignore[reportMissingImports]

# Try to import Qdrant components, but handle gracefully if not available
try:
    from qdrant_setup import qdrant_client
    from search_service import get_search_service
    QDRANT_AVAILABLE = True
except ImportError:
    qdrant_client = None
    get_search_service = None
    QDRANT_AVAILABLE = False
    print("Warning: Qdrant client not available. Some features will be disabled.")

# Initialize OpenAI client for OpenRouter
openai_client = OpenAI(
    api_key=config.OPENROUTER_API_KEY,
    base_url=config.OPENROUTER_BASE_URL
)

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Physical AI & Humanoid Robotics Book RAG Chatbot",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class ChatRequest(BaseModel):
    message: str
    session_id: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    session_id: str
    context_sources: List[str]

class ContextRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class SearchResponse(BaseModel):
    results: List[Dict]

@app.get("/")
async def root():
    return {"message": "RAG Chatbot Backend API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy", "qdrant": "connected" if QDRANT_AVAILABLE else "disconnected"}

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Process a chat message and return a response using RAG
    """
    try:
        if QDRANT_AVAILABLE:
            # Get search service and retrieve relevant context from Qdrant
            search_service = get_search_service()
            context = search_service.get_relevant_context(
                query=request.message,
                selected_text=request.selected_text,
                top_k=3
            )

            # Prepare the prompt for the LLM with context
            if context:
                prompt = f"""
                You are an assistant for the Physical AI & Humanoid Robotics book.
                Use the following context to answer the user's question.
                If the context doesn't contain relevant information, say so.

                Context:
                {context}

                User's question: {request.message}

                Answer:
                """
            else:
                prompt = f"""
                You are an assistant for the Physical AI & Humanoid Robotics book.
                The user asked: {request.message}
                Since no relevant context was found, answer based on general knowledge about robotics and AI.
                If the question is about the book content specifically and you don't have context, please say so.

                Answer:
                """
        else:
            # Qdrant not available, use general knowledge
            prompt = f"""
            You are an assistant for the Physical AI & Humanoid Robotics book.
            The user asked: {request.message}
            NOTE: The RAG system is not currently available, so this is a general response.
            The system is running but cannot access the book content for context.

            Answer:
            """

        # Generate response using OpenRouter with Qwen model
        response = openai_client.chat.completions.create(
            model=config.CHAT_MODEL,
            messages=[
                {"role": "system", "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics book. Answer questions accurately. If RAG is not available, acknowledge this limitation and answer based on general knowledge."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.7
        )

        # Extract the response
        llm_response = response.choices[0].message.content

        # Set context sources based on availability
        context_sources = []
        if QDRANT_AVAILABLE:
            try:
                search_results = search_service.search_with_selected_text(
                    request.message,
                    request.selected_text,
                    top_k=3
                )
                context_sources = [f"{result['section_title']} ({result['source_file']})" for result in search_results]
            except:
                context_sources = ["RAG system temporarily unavailable"]
        else:
            context_sources = ["RAG system not available - using general knowledge"]

        return ChatResponse(
            response=llm_response,
            session_id=request.session_id,
            context_sources=context_sources
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.post("/api/search", response_model=SearchResponse)
async def search(request: ContextRequest):
    """
    Perform semantic search on the book content
    """
    try:
        if not QDRANT_AVAILABLE:
            return SearchResponse(results=[{
                "error": "Search functionality is not available - Qdrant client not installed",
                "query": request.query,
                "selected_text": request.selected_text
            }])

        search_service = get_search_service()

        # Perform semantic search
        results = search_service.search_with_selected_text(
            query=request.query,
            selected_text=request.selected_text,
            top_k=5
        )

        return SearchResponse(results=results)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error performing search: {str(e)}")

@app.post("/api/context")
async def process_context(request: ContextRequest):
    """
    Process selected text and prepare context for chat
    """
    try:
        if not QDRANT_AVAILABLE:
            return {
                "message": "Context processing not available - Qdrant client not installed",
                "query": request.query,
                "has_selected_text": request.selected_text is not None,
                "context_preview": "RAG system unavailable - general responses only"
            }

        search_service = get_search_service()

        # Get relevant context based on the query and selected text
        context = search_service.get_relevant_context(
            query=request.query,
            selected_text=request.selected_text,
            top_k=3
        )

        return {
            "message": "Context processed successfully",
            "query": request.query,
            "has_selected_text": request.selected_text is not None,
            "context_preview": context[:200] + "..." if len(context) > 200 else context
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing context: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=config.HOST, port=config.PORT)