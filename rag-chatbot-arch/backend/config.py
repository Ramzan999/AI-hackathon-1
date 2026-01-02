"""
Configuration module for RAG Chatbot
Loads configuration from environment variables
"""
from pydantic_settings import BaseSettings
from typing import Optional
import os

# Create .env file if it doesn't exist, using .env.example as template
if not os.path.exists(".env"):
    if os.path.exists(".env.example"):
        with open(".env.example", "r") as src:
            with open(".env", "w") as dst:
                dst.write(src.read())
        print("Created .env file from .env.example")

class Config(BaseSettings):
    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_PORT: int = 6333

    # Neon Postgres Configuration
    NEON_DATABASE_URL: str

    # OpenRouter Configuration
    OPENROUTER_API_KEY: str
    OPENROUTER_BASE_URL: str = "https://openrouter.ai/api/v1"

    # Qwen-specific configuration
    QWEN_API_KEY: str

    # Model configuration
    CHAT_MODEL: str = "qwen/qwen-2.5-72b-instruct"
    EMBEDDING_MODEL: str = "qwen"

    # Collection names
    CONTENT_COLLECTION: str = "book_content"

    # Server configuration
    HOST: str = "0.0.0.0"
    PORT: int = 8000

    # CORS configuration
    FRONTEND_URL: str = "http://localhost:3000"

# Create config instance
config = Config()