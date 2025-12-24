import os
import re
from pathlib import Path
from typing import List
from main import Document

def extract_content_from_markdown(file_path: str) -> List[Document]:
    """Extract content from a markdown file, breaking it into chunks"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract title from the first heading
    title_match = re.match(r'^# (.+)', content, re.MULTILINE)
    title = title_match.group(1) if title_match else Path(file_path).stem

    # Split content into chunks (by sections)
    sections = re.split(r'\n## ', content)

    documents = []

    # Process the first part (before first ##)
    if sections:
        first_part = sections[0].strip()
        if first_part:
            # Clean up the markdown
            clean_content = re.sub(r'^# .+\n*', '', first_part, flags=re.MULTILINE)
            clean_content = clean_content.strip()
            if len(clean_content) > 50:  # Only add if substantial content
                documents.append(Document(
                    content=clean_content,
                    source=file_path,
                    section=title
                ))

    # Process the remaining sections
    for i, section in enumerate(sections[1:], 1):
        if section.strip():
            # Add the section heading back
            section_title = section.split('\n')[0].strip()
            section_content = f"## {section}"
            clean_content = re.sub(r'\n+', '\n', section_content).strip()

            if len(clean_content) > 50:  # Only add if substantial content
                documents.append(Document(
                    content=clean_content,
                    source=file_path,
                    section=f"{title} - {section_title}"
                ))

    return documents

def chunk_text(text: str, max_length: int = 1000) -> List[str]:
    """Split text into chunks of approximately max_length characters"""
    if len(text) <= max_length:
        return [text]

    chunks = []
    paragraphs = text.split('\n\n')

    current_chunk = ""
    for paragraph in paragraphs:
        if len(current_chunk + paragraph) <= max_length:
            current_chunk += paragraph + "\n\n"
        else:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            current_chunk = paragraph + "\n\n"

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks

def process_book_content(docs_dir: str = "../docs") -> List[Document]:
    """Process all markdown files in the docs directory"""
    docs_path = Path(docs_dir)
    all_documents = []

    # Find all markdown files
    md_files = list(docs_path.rglob("*.md"))

    print(f"Found {len(md_files)} markdown files to process")

    for md_file in md_files:
        print(f"Processing {md_file}")

        try:
            # Extract content from the markdown file
            file_docs = extract_content_from_markdown(str(md_file))

            # Further chunk if needed
            for doc in file_docs:
                if len(doc.content) > 1000:
                    # Chunk large documents
                    chunks = chunk_text(doc.content, 1000)
                    for i, chunk in enumerate(chunks):
                        all_documents.append(Document(
                            content=chunk,
                            source=str(md_file),
                            section=f"{doc.section} - Chunk {i+1}"
                        ))
                else:
                    all_documents.append(doc)

        except Exception as e:
            print(f"Error processing {md_file}: {e}")
            continue

    print(f"Total documents created: {len(all_documents)}")
    return all_documents

def save_documents_to_file(documents: List[Document], output_file: str = "documents.json"):
    """Save documents to a JSON file for debugging"""
    import json

    docs_dict = [doc.dict() for doc in documents]
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(docs_dict, f, indent=2, ensure_ascii=False)

    print(f"Documents saved to {output_file}")

if __name__ == "__main__":
    # Process all book content
    documents = process_book_content()

    # Save to file for inspection
    save_documents_to_file(documents)

    # Print summary
    print(f"\nSummary:")
    print(f"- Total documents: {len(documents)}")
    print(f"- Average content length: {sum(len(doc.content) for doc in documents) / len(documents):.0f} chars")
    print(f"- Sources: {len(set(doc.source for doc in documents))} unique files")