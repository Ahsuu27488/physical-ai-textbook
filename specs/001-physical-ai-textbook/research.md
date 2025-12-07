# Research Summary: Physical AI & Humanoid Robotics Textbook Platform

## Decision: Docusaurus v3.9.2 for Frontend
**Rationale**: Docusaurus is a proven static site generator optimized for documentation sites like textbooks. It provides built-in features for navigation, search, and responsive design. The v3.9.2 version offers modern React-based architecture with plugin ecosystem support.
**Alternatives considered**:
- Next.js with custom documentation setup (more complex, requires more custom code)
- Gatsby (slower development, less documentation-focused)
- VuePress (less ecosystem support for React components)

## Decision: FastAPI for Backend
**Rationale**: FastAPI provides excellent performance, automatic API documentation, and strong typing support. It's ideal for the RAG (Retrieval Augmented Generation) system and integrates well with the AI/ML ecosystem.
**Alternatives considered**:
- Flask (less performant, less modern features)
- Express.js (Node.js-based, but Python ecosystem preferred for AI tools)
- Django (heavier framework than needed)

## Decision: BetterAuth for Authentication
**Rationale**: BetterAuth provides framework-agnostic authentication with support for social logins, email/password, and session management. It integrates well with modern frontend frameworks and provides security best practices out of the box.
**Alternatives considered**:
- NextAuth.js (framework-specific, only for Next.js)
- Auth.js (similar to BetterAuth but BetterAuth has better documentation)
- Custom authentication (security risks, more development time)

## Decision: Neon PostgreSQL for User Data
**Rationale**: Neon is a serverless PostgreSQL platform that offers autoscaling, branching, and instant restore capabilities. It provides familiar PostgreSQL interface with modern cloud features and good integration with BetterAuth.
**Alternatives considered**:
- Supabase (also Postgres-based, but Neon has better serverless features)
- PlanetScale (MySQL-based, not preferred for this use case)
- SQLite (not suitable for production multi-user application)

## Decision: Qdrant for Vector Storage
**Rationale**: Qdrant is a high-performance vector database specifically designed for similarity search. It has excellent Python client support and is well-suited for RAG applications. It supports metadata filtering and has good performance characteristics.
**Alternatives considered**:
- Pinecone (managed but proprietary)
- Weaviate (similar capabilities but Qdrant has better open-source support)
- Chroma (lightweight but less scalable)

## Decision: OpenAI Agents SDK for AI Functionality
**Rationale**: OpenAI Agents SDK provides a framework for building multi-agent workflows with built-in tools for context management, handoffs, and tracing. It's specifically designed for agentic AI applications.
**Alternatives considered**:
- LangChain (more general-purpose but more complex)
- LlamaIndex (good for RAG but less agent-focused)
- Direct OpenAI API (less abstraction, more custom code needed)

## Decision: GitHub Pages for Deployment
**Rationale**: GitHub Pages provides free, reliable hosting for static sites with custom domain support. It integrates well with GitHub workflows and provides CDN distribution. Perfect for Docusaurus-generated static content.
**Alternatives considered**:
- Vercel (good but costs apply for advanced features)
- Netlify (similar to Vercel, but GitHub Pages is free and integrated)
- AWS S3/CloudFront (more complex setup, costs involved)

## Decision: Dark Academic UI Theme
**Rationale**: A dark theme with blues and neon tones provides better readability for long-form educational content, reduces eye strain during extended study sessions, and creates a modern technology-focused aesthetic appropriate for AI/robotics content.
**Alternatives considered**:
- Light theme (traditional but causes more eye strain)
- Solarized themes (specific color schemes but less customizable)