from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import logging
from contextlib import asynccontextmanager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration from environment variables
QDRANT_URL = os.getenv("QDRANT_URL", "")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY", "")
EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL_NAME", "all-MiniLM-L6-v2")
USE_QWEN_EMBEDDINGS = False

# Initialize global variables
qdrant_client = None
embedding_model = None

# Import required libraries for RAG
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    from sentence_transformers import SentenceTransformer
    import openai
    import requests
except ImportError as e:
    logger.warning(f"Some packages may not be installed: {e}")

# Import API routes
from src.api.auth import router as auth_router
from src.api.profile import router as profile_router
from src.api.chat import router as chat_router
from src.middleware.cors import configure_cors

class Document(BaseModel):
    content: str
    source: str
    section: str

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5
    user_context: Optional[str] = None

class OldChatRequest(BaseModel):
    query: str
    user_context: Optional[str] = None
    model: str = "openai/gpt-3.5-turbo"

class OldChatResponse(BaseModel):
    response: str
    sources: List[str]
    tokens_used: int

class TranslationRequest(BaseModel):
    chapter_id: str
    chapter_title: str
    content: str

class TranslationResponse(BaseModel):
    success: bool
    translated_content: Optional[str] = None
    cached: bool = False
    language: str = "ur"

def get_embedding_vector(text: str) -> list:
    if USE_QWEN_EMBEDDINGS:
        if not os.getenv("QWEN_API_KEY"):
            raise ValueError("QWEN_API_KEY not set for embeddings")
        import requests
        headers = {
            "Authorization": f"Bearer {os.getenv('QWEN_API_KEY')}",
            "Content-Type": "application/json"
        }
        payload = {
            "model": "text-embedding-v2",
            "input": {"texts": [text]}
        }
        response = requests.post(
            "https://dashscope.aliyuncs.com/api/v1/services/embeddings/text-embedding",
            headers=headers, json=payload
        )
        if response.status_code != 200:
            raise Exception(f"Qwen embedding API request failed: {response.status_code}")
        result = response.json()
        return result['output']['embeddings'][0]['embedding']
    else:
        return embedding_model.encode(text).tolist()

@asynccontextmanager
async def lifespan(app: FastAPI):
    global qdrant_client, embedding_model

    # Initialize Qdrant for RAG
    if QDRANT_API_KEY and QDRANT_URL:
        qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=False
        )
        logger.info("Connected to Qdrant cloud")
    else:
        qdrant_client = QdrantClient(host="localhost", port=6333)
        logger.warning("No QDRANT_URL/API_KEY set, using local Qdrant")

    # Only load local embedding model if not using Qwen
    if not USE_QWEN_EMBEDDINGS and embedding_model is None:
        try:
            embedding_model = SentenceTransformer(EMBEDDING_MODEL_NAME)
            logger.info(f"Loaded embedding model: {EMBEDDING_MODEL_NAME}")
        except Exception as e:
            logger.warning(f"Could not load embedding model: {e}")

    # Check if collection exists
    if qdrant_client:
        try:
            vector_size = 1536 if USE_QWEN_EMBEDDINGS else 384
            collections = qdrant_client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if "book_content" not in collection_names:
                qdrant_client.create_collection(
                    collection_name="book_content",
                    vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE)
                )
                logger.info(f"Created Qdrant collection 'book_content' with {vector_size} dimensions")
            else:
                logger.info("Using existing Qdrant collection 'book_content'")
        except Exception as e:
            logger.warning(f"Could not check/create Qdrant collection: {e}")

    yield

# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book API",
    description="API for the Physical AI & Humanoid Robotics Book with RAG Chatbot and Translation",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS
configure_cors(app)

# Include routers
app.include_router(auth_router, prefix="/api/auth")
app.include_router(profile_router, prefix="/api/profile")
app.include_router(chat_router, prefix="/api")

# Root endpoint
@app.get("/")
def read_root():
    return {
        "message": "Physical AI & Humanoid Robotics Book API",
        "status": "ready",
        "version": "1.0.0",
        "features": ["RAG Chatbot", "User Authentication", "Urdu Translation"]
    }

@app.get("/health")
def health_check():
    return {
        "status": "healthy",
        "qdrant_connected": QDRANT_API_KEY is not None,
        "openrouter_configured": OPENROUTER_API_KEY is not None
    }

# Translation endpoint
from deep_translator import GoogleTranslator
from concurrent.futures import ThreadPoolExecutor
import re

to_urdu_translator = GoogleTranslator(source='auto', target='ur')
executor = ThreadPoolExecutor(max_workers=2)
MAX_CHARS = 3000

def clean_text_for_translation(text: str):
    code_blocks = []
    pattern = r'```[\s\S]*?```'
    def replace_code(match):
        code_blocks.append(match.group(0))
        return f'__CODE_BLOCK_{len(code_blocks)-1}__'
    cleaned = re.sub(pattern, replace_code, text)
    inline_codes = []
    pattern2 = r'`[^`]+`'
    def replace_inline(match):
        inline_codes.append(match.group(0))
        return f'__INLINE_CODE_{len(inline_codes)-1}__'
    cleaned = re.sub(pattern2, replace_inline, cleaned)
    return cleaned, code_blocks, inline_codes

def restore_code_blocks(text: str, code_blocks: list, inline_codes: list):
    for i, block in enumerate(code_blocks):
        text = text.replace(f'__CODE_BLOCK_{i}__', block)
    for i, code in enumerate(inline_codes):
        text = text.replace(f'__INLINE_CODE_{i}__', code)
    return text

def translate_long_text_sync(text: str) -> str:
    cleaned_text, code_blocks, inline_codes = clean_text_for_translation(text)
    if len(cleaned_text) <= MAX_CHARS:
        translated = to_urdu_translator.translate(cleaned_text)
        return restore_code_blocks(translated, code_blocks, inline_codes)
    paragraphs = cleaned_text.split('\n\n')
    chunks = []
    current_chunk = ""
    for para in paragraphs:
        para = para.strip()
        if not para:
            continue
        if len(current_chunk) + len(para) + 2 <= MAX_CHARS:
            current_chunk += para + "\n\n"
        else:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            if len(para) > MAX_CHARS:
                para = para[:MAX_CHARS - 100] + "..."
            current_chunk = para + "\n\n"
    if current_chunk.strip():
        chunks.append(current_chunk.strip())
    if not chunks:
        return restore_code_blocks(
            to_urdu_translator.translate(text[:MAX_CHARS] + "..."),
            code_blocks, inline_codes
        )
    translated_chunks = []
    for i, chunk in enumerate(chunks):
        try:
            translated = to_urdu_translator.translate(chunk)
            translated_chunks.append(translated)
        except Exception as e:
            logger.error(f"Error translating chunk {i}: {e}")
            translated_chunks.append(f"[Translation error in chunk {i+1}]")
    translated = '\n\n'.join(translated_chunks)
    return restore_code_blocks(translated, code_blocks, inline_codes)

@app.post("/api/translate/{chapter_id}", response_model=TranslationResponse)
async def translate_chapter(chapter_id: str, request: TranslationRequest):
    """Translate chapter content to Urdu using deep-translator."""
    import asyncio
    try:
        loop = asyncio.get_event_loop()
        translated = await loop.run_in_executor(
            executor,
            translate_long_text_sync,
            request.content
        )
        return TranslationResponse(
            success=True,
            translated_content=translated,
            cached=False,
            language="ur",
        )
    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")

@app.post("/api/embeddings")
async def add_embeddings(request: EmbeddingRequest):
    try:
        points = []
        for i, doc in enumerate(request.documents):
            embedding = get_embedding_vector(doc.content)
            points.append(
                models.PointStruct(
                    id=i,
                    vector=embedding,
                    payload={"content": doc.content, "source": doc.source, "section": doc.section}
                )
            )
        qdrant_client.upsert(collection_name="book_content", points=points)
        return {"message": f"Successfully added {len(request.documents)} documents", "count": len(request.documents)}
    except Exception as e:
        logger.error(f"Error adding embeddings: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error adding embeddings: {str(e)}")

@app.post("/api/search")
async def search_documents(query: QueryRequest):
    try:
        query_embedding = get_embedding_vector(query.query)
        search_result = qdrant_client.query_points(
            collection_name="book_content",
            query=query_embedding,
            limit=query.top_k
        )
        results = []
        for point in search_result.points:
            results.append({
                "content": point.payload["content"],
                "source": point.payload["source"],
                "section": point.payload["section"],
                "score": point.score
            })
        return {"results": results}
    except Exception as e:
        logger.error(f"Error searching documents: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error searching documents: {str(e)}")

@app.post("/api/chat", response_model=OldChatResponse)
async def chat_endpoint(request: OldChatRequest):
    try:
        if not OPENROUTER_API_KEY:
            raise HTTPException(status_code=500, detail="OPENROUTER_API_KEY not set")
        openai.api_key = OPENROUTER_API_KEY
        openai.base_url = "https://openrouter.ai/api/v1/"
        search_request = QueryRequest(query=request.query, top_k=5, user_context=request.user_context)
        search_results = await search_documents(search_request)
        context_texts = [result["content"] for result in search_results["results"]]
        context = "\n\n".join(context_texts)
        user_context = request.user_context if request.user_context else ""
        full_context = f"""
        You are an assistant for the Physical AI & Humanoid Robotics Book.
        Answer questions based on the provided context from the book.
        Context from the book:
        {context}
        User's selected text (if any):
        {user_context}
        User's question: {request.query}
        Please provide an accurate answer based on the book content, and cite sources when possible.
        """
        response = openai.chat.completions.create(
            model=request.model.replace("openai/", ""),
            messages=[
                {"role": "system", "content": "You are an assistant for the Physical AI & Humanoid Robotics Book."},
                {"role": "user", "content": full_context}
            ],
            temperature=0.3,
            max_tokens=1000
        )
        answer = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0
        sources = list(set([result["source"] for result in search_results["results"]]))
        return OldChatResponse(response=answer, sources=sources, tokens_used=tokens_used)
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error in chat endpoint: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", "7860"))
    uvicorn.run(app, host="0.0.0.0", port=port)
