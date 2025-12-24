import asyncio
import os
from pathlib import Path
from typing import List
from main import Document, qdrant_client, embedding_model, EMBEDDING_MODEL_NAME
from process_content import process_book_content
from qdrant_client.http import models

async def load_book_content_to_qdrant():
    """Load all book content into Qdrant vector store"""
    print("Starting to load book content to Qdrant...")

    # Process all book content
    documents = process_book_content()
    print(f"Processed {len(documents)} document chunks from book content")

    # Initialize Qdrant client if not already done
    if qdrant_client is None:
        from qdrant_client import QdrantClient
        from sentence_transformers import SentenceTransformer

        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, prefer_grpc=True)
        else:
            client = QdrantClient(host="localhost", port=6333)
    else:
        client = qdrant_client

    # Initialize embedding model if not already done
    if embedding_model is None:
        from sentence_transformers import SentenceTransformer
        emb_model = SentenceTransformer(EMBEDDING_MODEL_NAME)
    else:
        emb_model = embedding_model

    # Create collection if it doesn't exist
    try:
        client.get_collection("book_content")
        print("Found existing Qdrant collection 'book_content'")
        # Clear existing content
        client.delete_collection("book_content")
        print("Cleared existing collection")
    except:
        print("Creating new Qdrant collection 'book_content'")

    client.create_collection(
        collection_name="book_content",
        vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
    )

    # Prepare documents for Qdrant in batches
    batch_size = 100
    total_docs = len(documents)

    print(f"Starting to upload {total_docs} documents in batches of {batch_size}...")

    for i in range(0, total_docs, batch_size):
        batch_docs = documents[i:i + batch_size]

        # Prepare points for this batch
        points = []
        for j, doc in enumerate(batch_docs):
            # Generate embedding
            embedding = emb_model.encode(doc.content).tolist()

            # Create point for Qdrant with unique ID
            point_id = i + j  # Ensure unique ID across all batches
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": doc.content,
                        "source": doc.source,
                        "section": doc.section
                    }
                )
            )

        # Upload batch to Qdrant
        client.upsert(
            collection_name="book_content",
            points=points
        )

        print(f"Uploaded batch {i//batch_size + 1}/{(total_docs-1)//batch_size + 1}: {len(batch_docs)} documents")

    print(f"Successfully uploaded all {total_docs} documents to Qdrant!")
    print("Book content is now available for RAG queries.")

def test_retrieval():
    """Test the retrieval functionality"""
    print("\nTesting retrieval functionality...")

    # Test query
    test_query = "What is ROS 2 and how does it work?"

    # Generate embedding for query
    from sentence_transformers import SentenceTransformer
    emb_model = SentenceTransformer(EMBEDDING_MODEL_NAME)
    query_embedding = emb_model.encode(test_query).tolist()

    # Search in Qdrant
    search_results = qdrant_client.search(
        collection_name="book_content",
        query_vector=query_embedding,
        limit=3
    )

    print(f"\nTest query: '{test_query}'")
    print("Top 3 results:")
    for i, result in enumerate(search_results, 1):
        print(f"\n{i}. Score: {result.score:.3f}")
        print(f"   Section: {result.payload['section']}")
        print(f"   Source: {result.payload['source']}")
        print(f"   Content preview: {result.payload['content'][:200]}...")

if __name__ == "__main__":
    # Run the loading process
    asyncio.run(load_book_content_to_qdrant())

    # Test retrieval
    test_retrieval()