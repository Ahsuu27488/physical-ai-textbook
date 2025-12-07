import uuid
import os
import re
from pathlib import Path
from typing import List, Dict, Any
from markdown import markdown
from bs4 import BeautifulSoup
import tiktoken
from pydantic import BaseModel
from app.models import DocumentChunk
from app.vector_db import QdrantManager
import openai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Set OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")


class MarkdownProcessor:
    """Process Markdown files and extract content."""

    def __init__(self):
        self.encoding = tiktoken.encoding_for_model("gpt-4")

    def read_markdown_file(self, file_path: Path) -> Dict[str, Any]:
        """Read a Markdown file and return its content and metadata."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract title from first heading
        title_match = re.search(r'^#\s+(.+)', content, re.MULTILINE)
        title = title_match.group(1) if title_match else file_path.stem

        # Extract content without frontmatter if present
        content_without_frontmatter = self._remove_frontmatter(content)

        return {
            "title": title,
            "content": content_without_frontmatter,
            "path": str(file_path),
            "file_name": file_path.name
        }

    def _remove_frontmatter(self, content: str) -> str:
        """Remove YAML frontmatter from content if present."""
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                return parts[2].strip()
        return content

    def extract_text_from_markdown(self, markdown_content: str) -> str:
        """Convert Markdown to plain text."""
        # Convert markdown to HTML
        html = markdown(markdown_content)
        # Extract text from HTML
        soup = BeautifulSoup(html, 'html.parser')
        return soup.get_text()

    def chunk_text(self, text: str, max_tokens: int = 500) -> List[str]:
        """Chunk text into pieces with a maximum number of tokens."""
        # Split text into sentences or paragraphs
        paragraphs = re.split(r'\n\s*\n', text)

        chunks = []
        current_chunk = ""
        current_token_count = 0

        for paragraph in paragraphs:
            # Estimate token count
            paragraph_tokens = len(self.encoding.encode(paragraph))

            # If a single paragraph is too large, split it into sentences
            if paragraph_tokens > max_tokens:
                sentences = re.split(r'[.!?]+', paragraph)
                temp_chunk = ""

                for sentence in sentences:
                    sentence = sentence.strip()
                    if not sentence:
                        continue

                    sentence_tokens = len(self.encoding.encode(sentence))

                    if len(temp_chunk) > 0 and current_token_count + sentence_tokens > max_tokens:
                        if temp_chunk.strip():
                            chunks.append(temp_chunk.strip())
                        temp_chunk = sentence + ". "
                        current_token_count = sentence_tokens
                    else:
                        temp_chunk += sentence + ". "
                        current_token_count += sentence_tokens

                if temp_chunk.strip():
                    chunks.append(temp_chunk.strip())
            else:
                # Check if adding this paragraph would exceed the token limit
                if current_token_count + paragraph_tokens > max_tokens and current_chunk:
                    chunks.append(current_chunk.strip())
                    current_chunk = paragraph
                    current_token_count = paragraph_tokens
                else:
                    current_chunk += "\n\n" + paragraph if current_chunk else paragraph
                    current_token_count += paragraph_tokens

        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks


class ETLProcessor:
    """ETL processor for ingesting Markdown documentation into the RAG system."""

    def __init__(self, source_dir: str, max_chunk_tokens: int = 500):
        self.source_dir = Path(source_dir)
        self.processor = MarkdownProcessor()
        self.max_chunk_tokens = max_chunk_tokens

    def extract(self) -> List[Dict[str, Any]]:
        """Extract all Markdown files from the source directory."""
        markdown_files = list(self.source_dir.rglob("*.md"))
        documents = []

        for file_path in markdown_files:
            try:
                doc_data = self.processor.read_markdown_file(file_path)
                documents.append(doc_data)
            except Exception as e:
                print(f"Error processing file {file_path}: {str(e)}")

        return documents

    def transform(self, documents: List[Dict[str, Any]]) -> List[DocumentChunk]:
        """Transform documents into chunked format suitable for RAG."""
        chunks = []

        for doc in documents:
            text_content = self.processor.extract_text_from_markdown(doc["content"])
            doc_chunks = self.processor.chunk_text(text_content, self.max_chunk_tokens)

            for i, chunk_text in enumerate(doc_chunks):
                # FIX: Generate a valid UUID based on the filename and index
                # We use uuid5 to ensure the ID is deterministic (same content = same ID)
                unique_str = f"{doc['file_name']}_{i}"
                chunk_uuid = str(uuid.uuid5(uuid.NAMESPACE_DNS, unique_str))

                chunk = DocumentChunk(
                    id=chunk_uuid,  # <--- UPDATED to use UUID
                    title=doc["title"],
                    content=chunk_text,
                    source_path=doc["path"],
                    chunk_index=i,
                    total_chunks=len(doc_chunks)
                )
                chunks.append(chunk)

        return chunks

    def load(self, chunks: List[DocumentChunk]) -> int:
        """Load chunks into the vector database in batches to avoid timeouts."""
        qdrant_manager = QdrantManager()
        batch_size = 20  # Process 20 chunks at a time
        total_loaded = 0
        
        print(f"Starting upload of {len(chunks)} chunks in batches of {batch_size}...")

        # Loop through chunks in batches
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i : i + batch_size]
            print(f"Uploading batch {i//batch_size + 1} ({len(batch)} chunks)...")
            
            try:
                # Upload the current batch
                success = qdrant_manager.add_document_chunks(batch)
                if success:
                    total_loaded += len(batch)
                else:
                    print(f"Failed to upload batch starting at index {i}")
            except Exception as e:
                print(f"Error uploading batch {i}: {str(e)}")

        return total_loaded

    def run_etl(self) -> Dict[str, Any]:
        """Run the complete ETL process."""
        print("Starting ETL process...")
        print(f"Source directory: {self.source_dir}")

        # Extract
        print("Extracting documents...")
        documents = self.extract()
        print(f"Extracted {len(documents)} documents")

        # Transform
        print("Transforming documents into chunks...")
        chunks = self.transform(documents)
        print(f"Transformed into {len(chunks)} chunks")

        # Load
        print("Loading chunks to vector database...")
        loaded_count = self.load(chunks)
        print(f"Loaded {loaded_count} chunks to database")

        return {
            "documents_processed": len(documents),
            "chunks_created": len(chunks),
            "chunks_loaded": loaded_count
        }


# Example usage
if __name__ == "__main__":
    # This would typically be run as a standalone script or as part of an initialization process
    etl = ETLProcessor(source_dir="../../frontend/docs")
    result = etl.run_etl()
    print(f"ETL completed: {result}")