---
title: Physical AI Backend API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
sdk_version: "3.11"
short_description: FastAPI backend for Physical AI & Humanoid Robotics Book
pinned: false
license: mit
tags:
- fastapi
- python
- api
- rag
- chatbot
- translation
---

# Physical AI & Humanoid Robotics Book - Backend API

FastAPI backend for the Physical AI & Humanoid Robotics Book, deployed on Hugging Face Spaces.

## Features

- **RAG Chatbot**: Question answering based on book content using Qdrant vector database
- **User Authentication**: Sign up, login, JWT-based authentication
- **Urdu Translation**: Translate chapter content to Urdu
- **Reading Progress**: Track user's reading progress

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | API info |
| `/health` | GET | Health check |
| `/api/auth/signup` | POST | User registration |
| `/api/auth/login` | POST | User login |
| `/api/translate/{chapter_id}` | POST | Translate chapter to Urdu |
| `/api/chat` | POST | RAG chatbot |

## API Documentation

Visit `/docs` for interactive Swagger UI documentation.

## Environment Variables

Set these in Space settings under "Variables and secrets":

| Variable | Description | Required |
|----------|-------------|----------|
| `QDRANT_URL` | Qdrant Cloud URL | Yes |
| `QDRANT_API_KEY` | Qdrant Cloud API Key | Yes |
| `OPENROUTER_API_KEY` | OpenRouter API Key for LLM | Optional |

## Local Development

```bash
cd huggingface
pip install -r requirements.txt
python main.py
```

## Deploy to Hugging Face

```bash
pip install huggingface_hub
huggingface-cli login
cd huggingface
huggingface-cli upload space YourUsername/physical-ai-backend . --repo-type space
```

## License

MIT
