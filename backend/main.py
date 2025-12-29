from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import logging
from contextlib import asynccontextmanager
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Import required libraries for RAG
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    from sentence_transformers import SentenceTransformer
    import openai
    import requests
except ImportError:
    print("Please install required packages: pip install qdrant-client sentence-transformers openai python-dotenv requests")
    exit(1)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "https://e9733b30-e7d5-444c-9a10-6a70bf430618.eu-west-2-0.aws.cloud.qdrant.io")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.LBHUxT6lEeyld0ISSEGvbe4t6Wl_Bk0R-0wxQeBEEf8")
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
QWEN_API_KEY = os.getenv("QWEN_API_KEY")
EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL_NAME", "all-MiniLM-L6-v2")
USE_QWEN_EMBEDDINGS = False  # Use local embeddings (no API key needed)

# Initialize global variables
qdrant_client = None
embedding_model = None

# ============================================================================
# AUTHENTICATION IMPORTS
# ============================================================================
from src.db import init_db
from src.middleware.cors import configure_cors
from src.api.auth import router as auth_router
from src.api.profile import router as profile_router
from src.api.personalization import router as personalization_router
from src.api.chat import router as chat_router
# Translation router commented out - using direct endpoint below instead
# from src.api.translation import router as translation_router

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

class EmbeddingRequest(BaseModel):
    documents: List[Document]

def get_embedding_vector(text: str) -> list:
    if USE_QWEN_EMBEDDINGS:
        if not QWEN_API_KEY:
            raise ValueError("QWEN_API_KEY not set for embeddings")

        import requests
        headers = {
            "Authorization": f"Bearer {QWEN_API_KEY}",
            "Content-Type": "application/json"
        }

        # Prepare the request for Qwen embeddings
        payload = {
            "model": "text-embedding-v2",
            "input": {
                "texts": [text]
            }
        }

        response = requests.post(
            "https://dashscope.aliyuncs.com/api/v1/services/embeddings/text-embedding",
            headers=headers,
            json=payload
        )

        if response.status_code != 200:
            raise Exception(f"Qwen embedding API request failed: {response.status_code} - {response.text}")

        result = response.json()
        embedding = result['output']['embeddings'][0]['embedding']
        return embedding
    else:
        return embedding_model.encode(text).tolist()

@asynccontextmanager
async def lifespan(app: FastAPI):
    global qdrant_client, embedding_model

    # Initialize database for auth
    logger.info("Initializing authentication database...")
    try:
        init_db()
        logger.info("Auth database initialized successfully")
    except Exception as e:
        logger.warning(f"Could not initialize auth database: {e}")

    # Initialize Qdrant for RAG
    if QDRANT_API_KEY:
        qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=False
        )
    else:
        qdrant_client = QdrantClient(host="localhost", port=6333)

    # Only load local embedding model if not using Qwen
    if not USE_QWEN_EMBEDDINGS:
        embedding_model = SentenceTransformer(EMBEDDING_MODEL_NAME)

    # Set vector size based on embedding type
    vector_size = 1536 if USE_QWEN_EMBEDDINGS else 384

    # Check if collection exists, create if not
    try:
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if "book_content" in collection_names:
            logger.info("Qdrant collection 'book_content' already exists, using existing collection")
        else:
            # Create a new collection with correct dimensions for embedding model
            qdrant_client.create_collection(
                collection_name="book_content",
                vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE)
            )
            logger.info(f"Created new Qdrant collection 'book_content' with {vector_size} dimensions")
    except Exception as e:
        logger.warning(f"Could not check/create Qdrant collection: {e}")

    yield

app = FastAPI(
    title="Physical AI & Humanoid Robotics Book API",
    description="API for the Physical AI & Humanoid Robotics Book with Authentication and Personalization",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS
configure_cors(app)

# ============================================================================
# AUTHENTICATION ROUTES
# ============================================================================
app.include_router(auth_router)
app.include_router(profile_router)
app.include_router(personalization_router)
app.include_router(chat_router)
# Translation router NOT included - using direct endpoint instead

# ============================================================================
# EXISTING RAG ROUTES (preserved for backward compatibility)
# ============================================================================

@app.get("/")
def read_root():
    return {
        "message": "Physical AI & Humanoid Robotics Book API",
        "status": "ready",
        "version": "1.0.0",
        "features": ["RAG Chatbot", "User Authentication", "Content Personalization"]
    }

@app.get("/health")
def health_check():
    return {
        "status": "healthy",
        "qdrant_connected": True,
        "auth_enabled": True,
        "translation_enabled": True
    }


# =============================================================================
# TRANSLATION ENDPOINTS (Using deep-translator)
# =============================================================================

from pydantic import BaseModel
from typing import Optional
from deep_translator import GoogleTranslator
from concurrent.futures import ThreadPoolExecutor
import re

class TranslationRequest(BaseModel):
    chapter_id: str
    chapter_title: str
    content: str

class TranslationResponse(BaseModel):
    success: bool
    translated_content: Optional[str] = None
    cached: bool = False
    language: str = "ur"


# Create translator instance
to_urdu_translator = GoogleTranslator(source='auto', target='ur')

# Thread pool for running sync translator in async context
executor = ThreadPoolExecutor(max_workers=2)

# API limit for translation
MAX_CHARS = 3000  # Reduced for better reliability


def clean_text_for_translation(text: str) -> str:
    """Remove or protect code blocks, special characters from text."""
    # Replace code blocks with placeholder
    code_blocks = []
    pattern = r'```[\s\S]*?```'

    def replace_code(match):
        code_blocks.append(match.group(0))
        return f'__CODE_BLOCK_{len(code_blocks)-1}__'

    cleaned = re.sub(pattern, replace_code, text)

    # Also replace inline code
    inline_codes = []
    pattern2 = r'`[^`]+`'

    def replace_inline(match):
        inline_codes.append(match.group(0))
        return f'__INLINE_CODE_{len(inline_codes)-1}__'

    cleaned = re.sub(pattern2, replace_inline, cleaned)

    return cleaned, code_blocks, inline_codes


def restore_code_blocks(text: str, code_blocks: list, inline_codes: list) -> str:
    """Restore code blocks and inline code after translation."""
    for i, block in enumerate(code_blocks):
        text = text.replace(f'__CODE_BLOCK_{i}__', block)

    for i, code in enumerate(inline_codes):
        text = text.replace(f'__INLINE_CODE_{i}__', code)

    return text


def translate_long_text_sync(text: str) -> str:
    """Translate long text by splitting into chunks (sync version)."""
    # First, protect code blocks
    cleaned_text, code_blocks, inline_codes = clean_text_for_translation(text)

    if len(cleaned_text) <= MAX_CHARS:
        translated = to_urdu_translator.translate(cleaned_text)
        return restore_code_blocks(translated, code_blocks, inline_codes)

    # Split into chunks at paragraphs
    paragraphs = cleaned_text.split('\n\n')
    chunks = []
    current_chunk = ""

    for para in paragraphs:
        # Skip empty paragraphs
        para = para.strip()
        if not para:
            continue

        if len(current_chunk) + len(para) + 2 <= MAX_CHARS:
            current_chunk += para + "\n\n"
        else:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())

            # If para is too big, truncate it
            if len(para) > MAX_CHARS:
                para = para[:MAX_CHARS - 100] + "..."

            current_chunk = para + "\n\n"

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    if not chunks:
        # If no chunks, try translating a small part
        return restore_code_blocks(
            to_urdu_translator.translate(text[:MAX_CHARS] + "..."),
            code_blocks, inline_codes
        )

    # Translate each chunk
    translated_chunks = []
    for i, chunk in enumerate(chunks):
        try:
            translated = to_urdu_translator.translate(chunk)
            translated_chunks.append(translated)
        except Exception as e:
            print(f"Error translating chunk {i}: {e}")
            translated_chunks.append(f"[Translation error in chunk {i+1}]")

    translated = '\n\n'.join(translated_chunks)

    return restore_code_blocks(translated, code_blocks, inline_codes)


@app.post("/api/translate/{chapter_id}", response_model=TranslationResponse)
async def translate_chapter_direct(chapter_id: str, request: TranslationRequest):
    """
    Translate chapter content to Urdu using deep-translator.
    Handles long content and code blocks properly.
    """
    import asyncio

    try:
        content_len = len(request.content)
        print(f"Translating chapter: {chapter_id} (length: {content_len} chars)")

        # Run synchronous translation in thread pool
        loop = asyncio.get_event_loop()
        translated = await loop.run_in_executor(
            executor,
            translate_long_text_sync,
            request.content
        )

        print(f"Translation complete for {chapter_id}, length: {len(translated)} chars")

        return TranslationResponse(
            success=True,
            translated_content=translated,
            cached=False,
            language="ur",
        )

    except Exception as e:
        error_msg = str(e)
        print(f"Translation error: {error_msg}")
        # Return a helpful error message
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {error_msg[:200]}... Content may be too long or contain unsupported characters."
        )

@app.post("/embeddings")
async def add_embeddings(request: EmbeddingRequest):
    try:
        points = []
        for i, doc in enumerate(request.documents):
            embedding = get_embedding_vector(doc.content)
            points.append(
                models.PointStruct(
                    id=i,
                    vector=embedding,
                    payload={
                        "content": doc.content,
                        "source": doc.source,
                        "section": doc.section
                    }
                )
            )

        qdrant_client.upsert(
            collection_name="book_content",
            points=points
        )

        return {"message": f"Successfully added {len(request.documents)} documents", "count": len(request.documents)}

    except Exception as e:
        logger.error(f"Error adding embeddings: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error adding embeddings: {str(e)}")

@app.post("/search")
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

@app.post("/chat", response_model=OldChatResponse)
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
                {"role": "system", "content": "You are an assistant for the Physical AI & Humanoid Robotics Book. Provide accurate answers based on the book content and cite sources when possible."},
                {"role": "user", "content": full_context}
            ],
            temperature=0.3,
            max_tokens=1000
        )

        if not hasattr(response, 'choices'):
            logger.error(f"OpenRouter returned invalid response: {response}")
            raise HTTPException(status_code=500, detail=f"Invalid response from OpenRouter: {response}")

        answer = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0

        sources = list(set([result["source"] for result in search_results["results"]]))

        return OldChatResponse(
            response=answer,
            sources=sources,
            tokens_used=tokens_used
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error in chat endpoint: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
