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
    QDRANT_URL: str = "https://a48e254c-ff19-450d-9593-2d26f17b2068.eu-west-2-0.aws.cloud.qdrant.io:6333"
    QDRANT_API_KEY: str = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.APbILGTkNXjfOD2UY9S6ahan-R8l7zYElmXiKiwnps4"
    QDRANT_PORT: int = 6333

    # Neon Postgres Configuration
    NEON_DATABASE_URL: str = "postgresql://neondb_owner:npg_8Qutm4cvrOCB@ep-rough-thunder-abh7clnj-pooler.eu-west-2.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

    # OpenRouter Configuration
    OPENROUTER_API_KEY: str = "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"
    OPENROUTER_BASE_URL: str = "https://openrouter.ai/api/v1"

    # Qwen-specific configuration
    QWEN_API_KEY: str = "sk-9363cb6844664b54ba6c09e8b6cce733"

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