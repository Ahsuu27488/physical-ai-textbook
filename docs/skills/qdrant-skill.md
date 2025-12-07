# Qdrant Cloud Python Client Skill

## Installation

### Basic installation:
```bash
pip install qdrant-client
```

### With FastEmbed support (for local embeddings):
```bash
pip install qdrant-client[fastembed]
```

### With GPU support for FastEmbed:
```bash
pip install 'qdrant-client[fastembed-gpu]'
```

## Cloud Connection Setup

### Initialize client for Qdrant Cloud:
```python
from qdrant_client import QdrantClient

# Using cloud instance
client = QdrantClient(
    url="https://xyz-example.cloud-region.cloud-provider.cloud.qdrant.io",
    api_key="YOUR_API_KEY",
)

# Alternative format for cloud connection
client = QdrantClient(
    "xyz-example.cloud-region.cloud-provider.cloud.qdrant.io",
    api_key="YOUR_API_KEY",
)
```

### Initialize client for local instance:
```python
from qdrant_client import QdrantClient

# For local development
client = QdrantClient(host="localhost", port=6333)

# For in-memory instance (useful for testing)
client = QdrantClient(":memory:")
```

## Basic Usage Examples

### Create a collection:
```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

client = QdrantClient(
    url="https://xyz-example.cloud-region.cloud-provider.cloud.qdrant.io",
    api_key="YOUR_API_KEY",
)

# Create a collection
client.create_collection(
    collection_name="my_collection",
    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
)
```

### Upsert points (add vectors with payloads):
```python
# Upsert vectors with payloads
client.upsert(
    collection_name="my_collection",
    points=[
        models.PointStruct(
            id=1,
            vector=[0.05, 0.61, 0.76, 0.74],
            payload={"city": "Berlin", "country": "Germany"}
        ),
        models.PointStruct(
            id=2,
            vector=[0.19, 0.81, 0.75, 0.11],
            payload={"city": "London", "country": "England"}
        )
    ]
)
```

### Search for similar vectors:
```python
# Search for similar vectors
search_result = client.search(
    collection_name="my_collection",
    query_vector=[0.2, 0.1, 0.9, 0.7],
    limit=5,
    with_payload=True
)

for hit in search_result:
    print(f"ID: {hit.id}, Score: {hit.score}, Payload: {hit.payload}")
```

### Using FastEmbed for automatic embeddings:
```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Initialize client with FastEmbed support
client = QdrantClient(
    url="https://xyz-example.cloud-region.cloud-provider.cloud.qdrant.io",
    api_key="YOUR_API_KEY",
)

# Create collection
client.create_collection(
    collection_name="demo_collection",
    vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
)

# With fastembed, you can upsert texts directly without pre-computing embeddings
client.add(
    collection_name="demo_collection",
    documents=[
        "Qdrant is a vector search engine",
        "Embeddings are high-dimensional vectors",
        "FastEmbed provides fast local embeddings"
    ],
    ids=[1, 2, 3]
)

# Search using text (FastEmbed will convert to vector automatically)
hits = client.query(
    collection_name="demo_collection",
    query_text="vector search engine",
    limit=5
)

for hit in hits:
    print(f"ID: {hit.id}, Score: {hit.score}, Document: {hit.document}")
```

### Configure multiprocessing for gRPC client (to avoid UNAVAILABLE errors):
```python
import multiprocessing
from qdrant_client import QdrantClient

# Set multiprocessing start method to avoid gRPC errors
multiprocessing.set_start_method("forkserver")  # or "spawn"

client = QdrantClient(
    url="https://xyz-example.cloud-region.cloud-provider.cloud.qdrant.io",
    api_key="YOUR_API_KEY",
)
```

## Environment Variables (.env):
```
QDRANT_URL=https://xyz-example.cloud-region.cloud-provider.cloud.qdrant.io
QDRANT_API_KEY=your_api_key_here
```

### Using environment variables:
```python
import os
from qdrant_client import QdrantClient

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)
```