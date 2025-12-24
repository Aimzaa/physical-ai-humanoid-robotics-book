# Physical AI & Humanoid Robotics Book RAG API

This project implements a Retrieval-Augmented Generation (RAG) system for the Physical AI & Humanoid Robotics Book using high-quality embeddings and LLMs.

## Features

- Qwen embeddings (1536 dimensions) for high-quality semantic search
- OpenRouter integration for powerful LLM responses (GPT-4, Claude, Mistral, etc.)
- Qdrant vector database for efficient similarity search
- FastAPI backend with Docusaurus frontend

## Setup Instructions

### 1. API Key Setup

#### Get Qwen/DashScope API Key:
1. Go to [DashScope Console](https://dashscope.console.aliyun.com/)
2. Create an account or log in
3. Navigate to "API Keys" section
4. Create a new API key
5. Copy the API key

#### Get OpenRouter API Key:
1. Go to [OpenRouter](https://openrouter.ai/)
2. Sign up or log in
3. Go to "Keys" section in your dashboard
4. Create a new API key
5. Copy the API key

### 2. Environment Configuration

1. Create a `.env` file in the project root:
   ```bash
   cp .env.example .env
   ```

2. Edit the `.env` file with your API keys:
   ```env
   QWEN_API_KEY=your_actual_qwen_api_key_here
   OPENROUTER_API_KEY=your_actual_openrouter_api_key_here
   ```

### 3. Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install required dependencies:
   ```bash
   pip install fastapi uvicorn python-dotenv qdrant-client sentence-transformers openai requests
   ```

3. Start the backend server:
   ```bash
   python main.py
   ```
   The server will run on `http://localhost:8000`

### 4. Upload Book Content

1. Run the embedding upload script:
   ```bash
   python upload_embeddings.py
   ```
   This will:
   - Read all markdown files from the `docs/` folder
   - Chunk the content into smaller pieces
   - Generate Qwen embeddings (1536 dimensions) for each chunk
   - Upload embeddings to your Qdrant collection

### 5. Frontend Setup (Docusaurus)

1. In a new terminal, navigate to the project root
2. Install dependencies:
   ```bash
   npm install
   ```
3. Start the Docusaurus development server:
   ```bash
   npm start
   ```
   The site will be available at `http://localhost:3000`

### 6. Testing the Chatbot

1. Open your browser and go to `http://localhost:3000`
2. Use the chatbot widget to ask questions about the book content
3. The chatbot will use:
   - Qwen embeddings (1536 dimensions) for semantic search
   - OpenRouter models for high-quality responses

## Configuration

- `USE_QWEN_EMBEDDINGS`: Set to `True` to use Qwen embeddings (1536 dimensions), `False` for local model (384 dimensions)
- The system automatically handles vector dimension differences in Qdrant

## Troubleshooting

- If you get embedding dimension errors, make sure your Qdrant collection is recreated with the correct dimensions (1536 for Qwen, 384 for local model)
- Ensure both API keys are correctly set in your `.env` file
- Check that your Qdrant instance is accessible and the API key is valid