import os
import requests
import time
from pathlib import Path

def read_markdown_files(docs_path):
    """Read all markdown files from the docs folder"""
    markdown_files = []
    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.endswith('.md'):
                file_path = os.path.join(root, file)
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    # Store relative path for source
                    relative_path = os.path.relpath(file_path, docs_path)
                    markdown_files.append({
                        'content': content,
                        'source': relative_path,
                        'section': os.path.splitext(file)[0]  # Remove .md extension
                    })
    return markdown_files

def chunk_text(text, max_chunk_size=800, overlap=100):
    """Split text into overlapping chunks"""
    chunks = []
    sentences = text.split('. ')

    current_chunk = ""
    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        # Add period back to sentence except for the last one
        if not sentence.endswith('.'):
            sentence += '. '
        else:
            sentence += ' '

        # Check if adding this sentence would exceed the chunk size
        if len(current_chunk) + len(sentence) <= max_chunk_size:
            current_chunk += sentence
        else:
            # If current chunk is not empty, save it
            if current_chunk.strip():
                chunks.append(current_chunk.strip())

            # Start a new chunk
            # If the sentence is longer than max_chunk_size, split it by characters
            if len(sentence) > max_chunk_size:
                # Split long sentence into smaller parts
                for i in range(0, len(sentence), max_chunk_size - overlap):
                    chunk = sentence[i:i + max_chunk_size - overlap]
                    if chunk.strip():
                        chunks.append(chunk.strip())
                current_chunk = ""
            else:
                current_chunk = sentence

    # Add the last chunk if it exists
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    # Also handle cases where we have very long chunks
    final_chunks = []
    for chunk in chunks:
        if len(chunk) > max_chunk_size:
            # Split long chunks into smaller ones with overlap
            for i in range(0, len(chunk), max_chunk_size - overlap):
                sub_chunk = chunk[i:i + max_chunk_size - overlap]
                if sub_chunk.strip():
                    final_chunks.append(sub_chunk.strip())
        else:
            final_chunks.append(chunk)

    return final_chunks

def upload_embeddings(documents, api_url):
    """Upload documents to the embeddings endpoint"""
    try:
        response = requests.post(
            f"{api_url}/embeddings",
            json={"documents": documents},
            headers={"Content-Type": "application/json"}
        )
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error uploading embeddings: {e}")
        return None

def main():
    # Configuration
    DOCS_PATH = "docs"
    API_URL = "http://localhost:8000"  # Updated to use port 8000

    print("Reading markdown files from docs folder...")
    markdown_files = read_markdown_files(DOCS_PATH)

    print(f"Found {len(markdown_files)} markdown files")

    all_documents = []

    for file_info in markdown_files:
        print(f"Processing {file_info['source']}...")

        # Split content into chunks
        chunks = chunk_text(file_info['content'])

        print(f"  -> Created {len(chunks)} chunks")

        # Create document objects for each chunk
        for i, chunk in enumerate(chunks):
            document = {
                "content": chunk,
                "source": file_info['source'],
                "section": f"{file_info['section']}_chunk_{i+1}"
            }
            all_documents.append(document)

    print(f"\nTotal documents to upload: {len(all_documents)}")

    # Upload in batches to avoid overwhelming the server
    batch_size = 10
    total_uploaded = 0

    for i in range(0, len(all_documents), batch_size):
        batch = all_documents[i:i + batch_size]
        print(f"Uploading batch {i//batch_size + 1} ({len(batch)} documents)...")

        result = upload_embeddings(batch, API_URL)

        if result:
            print(f"  -> Successfully uploaded {len(batch)} documents")
            total_uploaded += len(batch)
        else:
            print(f"  -> Failed to upload batch {i//batch_size + 1}")

        # Add a small delay between batches to avoid overwhelming the server
        time.sleep(1)

    print(f"\nUpload complete! Total documents uploaded: {total_uploaded}")
    print(f"Total documents processed: {len(all_documents)}")

if __name__ == "__main__":
    main()