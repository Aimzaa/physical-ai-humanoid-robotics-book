from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import logging
from contextlib import asynccontextmanager
from dotenv import load_dotenv  
load_dotenv()  # Load environment variables from .env file

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

class Document(BaseModel):
    content: str
    source: str
    section: str

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5
    user_context: Optional[str] = None

class ChatRequest(BaseModel):
    query: str
    user_context: Optional[str] = None
    model: str = "openai/gpt-3.5-turbo"

class ChatResponse(BaseModel):
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
        # Using DashScope embedding API
        payload = {
            "model": "text-embedding-v2",  # or text-embedding-v1 depending on your preference
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

    # Delete the old collection if it exists to ensure correct dimensions
    try:
        qdrant_client.delete_collection("book_content")
        logger.info("Deleted existing Qdrant collection 'book_content'")
    except Exception as e:
        logger.info("Collection 'book_content' does not exist, will create a new one")

    # Create a new collection with correct dimensions for embedding model
    qdrant_client.create_collection(
        collection_name="book_content",
        vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE)
    )
    logger.info(f"Created new Qdrant collection 'book_content' with {vector_size} dimensions")

    yield

app = FastAPI(
    title="Physical AI & Humanoid Robotics Book RAG API",
    description="API for Retrieval-Augmented Generation for the Physical AI & Humanoid Robotics Book",
    version="1.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics Book RAG API", "status": "ready"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "qdrant_connected": True}

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

        # Naye Qdrant client ke liye sahi tarika
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

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
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
            model=request.model.replace("openai/", ""),  # Remove "openai/" prefix for OpenRouter
            messages=[
                {"role": "system", "content": "You are an assistant for the Physical AI & Humanoid Robotics Book. Provide accurate answers based on the book content and cite sources when possible."},
                {"role": "user", "content": full_context}
            ],
            temperature=0.3,
            max_tokens=1000
        )

        # Debug: Check if response is a valid object
        if not hasattr(response, 'choices'):
            logger.error(f"OpenRouter returned invalid response: {response}")
            raise HTTPException(status_code=500, detail=f"Invalid response from OpenRouter: {response}")

        answer = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0

        sources = list(set([result["source"] for result in search_results["results"]]))

        return ChatResponse(
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