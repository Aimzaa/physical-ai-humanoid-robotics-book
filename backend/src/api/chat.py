# Chatbot API endpoints with user context

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fastapi import APIRouter, HTTPException, status, Request
from fastapi.responses import JSONResponse

from schemas.chat import ChatRequest, ChatResponse
from services.chatbot_service import chat_with_context
from middleware.auth import optional_session

router = APIRouter(prefix="/api", tags=["Chatbot"])


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest, http_request: Request):
    """
    Chat with the RAG assistant. Responses are tailored to user profile.

    User context (software level, hardware experience, learning depth)
    is automatically applied to the response.
    """
    # Get session if available (optional)
    session = await optional_session(http_request)
    user_id = session.user_id if session else None
    is_guest = not session

    # Convert conversation history if provided
    history = None
    if request.conversation_history:
        history = [{"role": msg.role, "content": msg.content} for msg in request.conversation_history]

    # Call chatbot service with user context
    result = await chat_with_context(
        query=request.query,
        conversation_history=history,
        user_id=user_id,
        model=request.model
    )

    if not result.get("success", False):
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=result.get("response", "Chat failed")
        )

    return ChatResponse(
        success=True,
        response=result["response"],
        sources=result.get("sources", []),
        tokens_used=result.get("tokens_used", 0),
        user_context_applied=result.get("user_context_applied")
    )
