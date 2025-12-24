# RAG Backend Service for Physical AI & Humanoid Robotics Book

This backend service provides Retrieval-Augmented Generation (RAG) functionality for the Physical AI & Humanoid Robotics Book, allowing users to ask questions about the book content and get AI-powered responses with relevant citations.

## Architecture

The RAG system consists of:
- **FastAPI**: Web framework for the backend API
- **Qdrant**: Vector database for storing document embeddings
- **Sentence Transformers**: For generating text embeddings
- **OpenRouter API**: For LLM-powered responses

## Setup

### Prerequisites

1. Python 3.8+
2. Qdrant vector database (can run locally or use cloud service)
3. OpenRouter API key for LLM access

### Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys and configuration
```

### Environment Variables

Create a `.env` file with the following variables:

```env
QDRANT_URL=http://localhost:6333  # Your Qdrant instance URL
QDRANT_API_KEY=your_qdrant_api_key  # Your Qdrant API key (if using cloud)
OPENROUTER_API_KEY=your_openrouter_api_key  # Your OpenRouter API key
EMBEDDING_MODEL_NAME=all-MiniLM-L6-v2  # Model for generating embeddings
```

## Usage

### 1. Load Book Content into Vector Store

First, you need to process the book content and load it into the Qdrant vector store:

```bash
python load_embeddings.py
```

This will:
- Parse all markdown files from the `../docs` directory
- Chunk the content into manageable pieces
- Generate embeddings for each chunk
- Store them in Qdrant for fast retrieval

### 2. Start the Backend Service

```bash
python main.py
```

Or using uvicorn:
```bash
uvicorn main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`

### 3. API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /embeddings` - Add documents to the vector store
- `POST /search` - Search for relevant documents
- `POST /chat` - Main RAG endpoint for chat functionality

### 4. Frontend Integration

The frontend chatbot widget is integrated into the Docusaurus site and will automatically connect to the backend service.

## API Usage Examples

### Chat Endpoint
```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain ROS 2 architecture",
    "user_context": "Optional user-selected text to include as context"
  }'
```

### Search Endpoint
```bash
curl -X POST "http://localhost:8000/search" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is a ROS 2 node?",
    "top_k": 5
  }'
```

## Development

For development, you can run the service with auto-reload:

```bash
uvicorn main:app --reload --port 8000
```

## Deployment

### Local Development
1. Run Qdrant locally: `docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant`
2. Load embeddings: `python load_embeddings.py`
3. Start backend: `uvicorn main:app --port 8000`

### Production Deployment
1. Deploy Qdrant to a cloud provider or use Qdrant Cloud
2. Deploy the FastAPI app to a cloud platform (Heroku, AWS, GCP, etc.)
3. Set environment variables with production configuration
4. Run the content loading script once to populate the vector store

## Testing

To test the RAG functionality:

1. Load book content: `python load_embeddings.py`
2. Test search: `curl -X POST "http://localhost:8000/search" -H "Content-Type: application/json" -d '{"query": "ROS 2"}'`
3. Test chat: `curl -X POST "http://localhost:8000/chat" -H "Content-Type: application/json" -d '{"query": "What is ROS 2?"}'`

## Troubleshooting

- If you get embedding errors, ensure the sentence-transformers model is downloaded
- If Qdrant connection fails, verify the URL and API key
- If OpenRouter calls fail, verify your API key and check rate limits
- Check the logs for detailed error messages

## Cost Considerations

- Qdrant: Free tier supports up to 1GB storage (sufficient for this book)
- OpenRouter: Costs vary by model and usage (typically $0.10-10/1M tokens)
- Estimated monthly costs for educational usage: $10-100