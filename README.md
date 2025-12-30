---
title: Physical AI Backend API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
sdk_version: 3.11
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

This is the FastAPI backend for the Physical AI & Humanoid Robotics Book, deployed on Hugging Face Spaces.

## Features

- **RAG Chatbot**: Question answering based on book content using Qdrant vector database
- **User Authentication**: Sign up, login, JWT-based authentication
- **Content Personalization**: Track reading progress and preferences
- **Urdu Translation**: Translate chapter content to Urdu

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | API info |
| `/health` | GET | Health check |
| `/api/auth/signup` | POST | User registration |
| `/api/auth/login` | POST | User login |
| `/api/translate/{chapter_id}` | POST | Translate chapter to Urdu |
| `/chat` | POST | RAG chatbot |

## API Documentation

Visit `/docs` for interactive Swagger UI documentation.

## Environment Variables

Set these in Space settings:

| Variable | Description | Required |
|----------|-------------|----------|
| `QDRANT_URL` | Qdrant Cloud URL | Yes |
| `QDRANT_API_KEY` | Qdrant Cloud API Key | Yes |
| `OPENROUTER_API_KEY` | OpenRouter API Key for LLM | Optional |

## Local Development

```bash
cd backend
pip install -r requirements.txt
python main.py
```

## License

MIT
