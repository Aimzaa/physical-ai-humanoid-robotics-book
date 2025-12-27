# Chatbot request/response schemas

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ChatMessage(BaseModel):
    """A single chat message."""
    role: str  # "user" or "assistant"
    content: str


class ChatRequest(BaseModel):
    """Request to chat with the RAG assistant."""
    query: str = Field(..., description="User's question")
    model: str = Field(default="openai/gpt-3.5-turbo", description="LLM model to use")
    conversation_history: Optional[List[ChatMessage]] = None
    chapter_context: Optional[str] = None


class UserContextApplied(BaseModel):
    """Information about user context applied to the response."""
    software_level: str
    hardware_experience: str
    learning_depth: str


class ChatResponse(BaseModel):
    """Response from the chatbot."""
    success: bool = True
    response: str = Field(..., description="The generated response")
    sources: List[str] = Field(default_factory=list, description="Source chapters used")
    tokens_used: int = Field(default=0, description="Total tokens used")
    user_context_applied: Optional[UserContextApplied] = None
