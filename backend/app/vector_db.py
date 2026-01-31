import os
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv
from app.models import DocumentChunk, DocumentSearchResult
import logging

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)


class QdrantManager:
    """Manages connection and operations with Qdrant vector database."""

    def __init__(self, skip_init: bool = False):
        # Get Qdrant configuration from environment variables
        self.url = os.getenv("QDRANT_URL", "").strip()
        self.api_key = os.getenv("QDRANT_API_KEY", "").strip()
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_docs")

        # Vector size - using OpenAI's text-embedding-ada-002 which produces 1536-dimensional vectors
        self.vector_size = 1536

        self.client = None
        self._initialized = False

        # Skip initialization if requested (for lazy loading)
        if not skip_init:
            self._ensure_initialized()

    def _ensure_initialized(self):
        """Lazy initialization of Qdrant client."""
        if self._initialized:
            return True

        # Skip if Qdrant URL is not configured
        if not self.url:
            logger.warning("QDRANT_URL not set - Qdrant features will be disabled")
            return False

        try:
            # Initialize Qdrant client
            if self.api_key:
                self.client = QdrantClient(
                    url=self.url,
                    api_key=self.api_key,
                    timeout=60
                )
            else:
                # For local development without API key
                self.client = QdrantClient(
                    url=self.url,
                    timeout=60
                )

            # Initialize the collection if it doesn't exist
            self._initialize_collection()
            self._initialized = True
            return True
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant: {str(e)}")
            self.client = None
            return False

    def _initialize_collection(self):
        """Initialize the Qdrant collection if it doesn't exist."""
        if not self.client:
            return

        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create a new collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=self.vector_size, distance=Distance.COSINE),
                )
                logger.info(f"Created new collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error initializing collection: {str(e)}")
            # Don't raise - allow app to start without Qdrant

    def add_document_chunks(self, chunks: List[DocumentChunk]) -> bool:
        """Add document chunks to the Qdrant collection."""
        if not self._ensure_initialized():
            logger.error("Qdrant not initialized - cannot add documents")
            return False

        try:
            points = []
            for chunk in chunks:
                # Generate embedding for the content
                vector = self._generate_embedding(chunk.content)

                point = PointStruct(
                    id=chunk.id,
                    vector=vector,
                    payload={
                        "title": chunk.title,
                        "content": chunk.content,
                        "source_path": chunk.source_path,
                        "chunk_index": chunk.chunk_index,
                        "total_chunks": chunk.total_chunks,
                        "created_at": chunk.created_at.isoformat() if chunk.created_at else None
                    }
                )
                points.append(point)

            # Upload points to the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully added {len(points)} document chunks to collection")
            return True
        except Exception as e:
            logger.error(f"Error adding document chunks: {str(e)}")
            return False

    def search_documents(self, query_vector: List[float], top_k: int = 5) -> List[DocumentSearchResult]:
        """Search for relevant documents based on the query vector."""
        if not self._ensure_initialized():
            logger.error("Qdrant not initialized - cannot search documents")
            return []

        try:
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k
            )

            results = []
            for hit in search_results:
                result = DocumentSearchResult(
                    id=hit.id,
                    title=hit.payload.get("title", ""),
                    content=hit.payload.get("content", ""),
                    source_path=hit.payload.get("source_path", ""),
                    score=hit.score,
                    metadata={
                        "chunk_index": hit.payload.get("chunk_index"),
                        "total_chunks": hit.payload.get("total_chunks"),
                        "created_at": hit.payload.get("created_at")
                    } if hit.payload else None
                )
                results.append(result)

            logger.info(f"Found {len(results)} relevant documents")
            return results
        except Exception as e:
            logger.error(f"Error searching documents: {str(e)}")
            return []

    def _generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for the given text using OpenAI API."""
        from openai import OpenAI
        import os

        # Initialize OpenAI client
        client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Truncate text if it's too long for the embedding model
        max_length = 8192  # Maximum for text-embedding-ada-002
        if len(text) > max_length:
            text = text[:max_length]

        try:
            response = client.embeddings.create(
                input=text,
                model="text-embedding-ada-002"
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}")
            # Return a zero vector in case of error
            return [0.0] * self.vector_size

    def generate_query_embedding(self, query: str) -> List[float]:
        """Generate embedding for the search query."""
        return self._generate_embedding(query)

    def delete_collection(self) -> bool:
        """Delete the entire collection (use with caution)."""
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Error deleting collection: {str(e)}")
            return False