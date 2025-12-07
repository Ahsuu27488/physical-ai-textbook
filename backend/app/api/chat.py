from fastapi import APIRouter, HTTPException, Depends
from typing import List
import logging
from app.models import ChatRequest, ChatResponse, DocumentSearchRequest, DocumentSearchResponse
from app.vector_db import QdrantManager
from openai import OpenAI
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Initialize OpenAI client
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Initialize logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# Initialize Qdrant manager
qdrant_manager = QdrantManager()


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest):
    """
    Chat endpoint that processes user messages and returns AI-generated responses
    with context from the Physical AI & Humanoid Robotics textbook.
    """
    try:
        # Generate embedding for the user's query
        query_vector = qdrant_manager.generate_query_embedding(chat_request.message)

        # Search for relevant documents in the vector database
        search_results = qdrant_manager.search_documents(
            query_vector=query_vector,
            top_k=chat_request.context_window or 5
        )

        # Prepare context from search results
        context_texts = [result.content for result in search_results]
        context = "\n\n".join(context_texts)

        # Prepare the system message with context
        system_message = f"""
        You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
        Use the following context to answer the user's question:

        {context}

        If the context doesn't contain relevant information, say so.
        Provide helpful, accurate responses based on the textbook content.
        """

        # Prepare messages for OpenAI API
        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": chat_request.message}
        ]

        # Call OpenAI API to generate response
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            max_tokens=500,
            temperature=0.7
        )

        # Extract the AI's response
        ai_response = response.choices[0].message.content

        # Extract source documents for attribution
        sources = [result.source_path for result in search_results]

        # Generate a conversation ID (in a real app, this would be stored in a database)
        conversation_id = chat_request.conversation_id or "temp_conversation"

        # Log the interaction
        logger.info(f"Chat request processed: {chat_request.message[:50]}...")

        return ChatResponse(
            response=ai_response,
            conversation_id=conversation_id,
            sources=sources,
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@router.post("/search", response_model=DocumentSearchResponse)
async def search_endpoint(search_request: DocumentSearchRequest):
    """
    Search endpoint that allows users to search for specific documents in the textbook.
    """
    try:
        # Generate embedding for the search query
        query_vector = qdrant_manager.generate_query_embedding(search_request.query)

        # Search for relevant documents in the vector database
        search_results = qdrant_manager.search_documents(
            query_vector=query_vector,
            top_k=search_request.top_k
        )

        # Log the search
        logger.info(f"Search request processed: {search_request.query}")

        return DocumentSearchResponse(
            query=search_request.query,
            results=search_results
        )

    except Exception as e:
        logger.error(f"Error in search endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing search request: {str(e)}")


@router.get("/documents/health")
async def documents_health():
    """
    Health check for the document database.
    """
    try:
        # Try to get collection info to verify connection
        collection_info = qdrant_manager.client.get_collection(qdrant_manager.collection_name)

        return {
            "status": "healthy",
            "collection": qdrant_manager.collection_name,
            "vectors_count": collection_info.points_count,
            "indexed_vectors_count": collection_info.indexed_vectors_count
        }
    except Exception as e:
        logger.error(f"Error in documents health check: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Document database health check failed: {str(e)}")