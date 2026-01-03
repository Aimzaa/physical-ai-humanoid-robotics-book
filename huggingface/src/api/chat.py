from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import os
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

# We'll use the main.py's qdrant_client and embedding_model
# These will be imported when needed

class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    history: Optional[List[ChatMessage]] = []
    model: str = "openai/gpt-3.5-turbo"

class ChatResponse(BaseModel):
    response: str
    sources: List[str]
    tokens_used: int

class SearchRequest(BaseModel):
    query: str
    top_k: int = 5

class SearchResult(BaseModel):
    content: str
    source: str
    section: str
    score: float

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """RAG Chat endpoint - asks questions about the book content."""
    try:
        # Import from main - this is a workaround for the circular import
        import main
        openai = __import__('openai')

        if not main.OPENROUTER_API_KEY:
            raise HTTPException(status_code=500, detail="OPENROUTER_API_KEY not configured")

        if not main.qdrant_client:
            raise HTTPException(status_code=500, detail="Qdrant client not initialized")

        # Search for relevant documents
        search_results = await main.search_documents(main.QueryRequest(
            query=request.message,
            top_k=5
        ))

        # Build context from search results
        context_texts = [result["content"] for result in search_results.get("results", [])]
        context = "\n\n".join(context_texts)

        # Build messages with history
        messages = [
            {"role": "system", "content": """You are an assistant for the Physical AI & Humanoid Robotics Book.
Answer questions based on the provided context from the book. Be accurate and cite sources when possible.
If you don't know the answer, say so clearly."""}
        ]

        # Add chat history
        for msg in request.history:
            messages.append({"role": msg.role, "content": msg.content})

        # Add context and user message
        full_prompt = f"""Context from the book:
{context}

User's question: {request.message}

Please provide an accurate answer based on the book content, and cite which section/source your answer comes from."""

        messages.append({"role": "user", "content": full_prompt})

        # Call OpenRouter
        openai.api_key = main.OPENROUTER_API_KEY
        openai.base_url = "https://openrouter.ai/api/v1/"

        response = openai.chat.completions.create(
            model=request.model.replace("openai/", ""),
            messages=messages,
            temperature=0.3,
            max_tokens=1000
        )

        answer = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0

        # Get unique sources
        sources = list(set([result["source"] for result in search_results.get("results", [])]))

        return ChatResponse(
            response=answer,
            sources=sources,
            tokens_used=tokens_used
        )
    except ImportError as e:
        logger.error(f"Import error: {e}")
        raise HTTPException(status_code=500, detail="Required packages not installed")
    except Exception as e:
        logger.error(f"Chat error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Chat failed: {str(e)}")

@router.post("/search", response_model=List[SearchResult])
async def search(request: SearchRequest):
    """Search the book content."""
    try:
        import main

        if not main.qdrant_client:
            raise HTTPException(status_code=500, detail="Qdrant client not initialized")

        results = await main.search_documents(main.QueryRequest(
            query=request.query,
            top_k=request.top_k
        ))

        return [
            SearchResult(
                content=r["content"],
                source=r["source"],
                section=r["section"],
                score=r["score"]
            )
            for r in results.get("results", [])
        ]
    except Exception as e:
        logger.error(f"Search error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Search failed: {str(e)}")
