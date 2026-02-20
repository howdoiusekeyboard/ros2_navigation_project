"""
Configuration management using Pydantic Settings
Loads from environment variables and .env file
"""

from pydantic_settings import BaseSettings
from pydantic import Field
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    # API Keys
    openai_api_key: str = Field(..., env="OPENAI_API_KEY")
    gemini_api_key: str = Field(..., env="GEMINI_API_KEY")
    
    # Speech-to-text (OpenAI)
    # Recommended models (2025): gpt-4o-transcribe, gpt-4o-mini-transcribe
    # Fallback/legacy: whisper-1
    openai_stt_model: str = Field("gpt-4o-mini-transcribe", env="OPENAI_STT_MODEL")

    # Database
    database_url: str = Field(
        "sqlite:///./robot_voice_control.db",  # Default to SQLite for simplicity
        env="DATABASE_URL"
    )

    # ROS2
    ros_domain_id: int = Field(0, env="ROS_DOMAIN_ID")

    # Server
    host: str = Field("0.0.0.0", env="HOST")
    port: int = Field(8000, env="PORT")
    debug: bool = Field(True, env="DEBUG")

    # CORS
    cors_origins: List[str] = Field(
        ["http://localhost:5173", "http://localhost:3000"],
        env="CORS_ORIGINS"
    )

    # Logging
    log_level: str = Field("INFO", env="LOG_LEVEL")

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


# Global settings instance
settings = Settings()
