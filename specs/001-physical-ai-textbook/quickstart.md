# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Platform

## Prerequisites
- Node.js 20+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- npm or yarn package manager
- Git

## Environment Setup
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd physical-ai-textbook
   ```

2. Create environment files:
   ```bash
   cp .env.example .env
   ```

3. Configure environment variables:
   - `DATABASE_URL`: Neon PostgreSQL connection string
   - `QDRANT_URL`: Qdrant cloud instance URL
   - `QDRANT_API_KEY`: Qdrant API key
   - `OPENAI_API_KEY`: OpenAI API key for agents SDK
   - `NEXT_PUBLIC_BASE_URL`: Base URL for the application

## Frontend Setup (Docusaurus)
1. Navigate to frontend directory:
   ```bash
   cd frontend  # or root directory if monorepo
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start development server:
   ```bash
   npm run start
   ```
   The site will be available at http://localhost:3000

4. Build for production:
   ```bash
   npm run build
   ```

## Backend Setup (FastAPI)
1. Navigate to backend directory:
   ```bash
   cd backend  # or if in monorepo structure
   ```

2. Create virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Start development server:
   ```bash
   uvicorn main:app --reload
   ```
   The API will be available at http://localhost:8000

5. View API documentation at http://localhost:8000/docs

## Milestone Implementation Order

### Milestone 1: Docusaurus Skeleton
1. Initialize Docusaurus project
2. Configure basic theme and navigation
3. Set up GitHub Pages deployment
4. Verify site builds and deploys successfully

### Milestone 2: Content Integration
1. Add textbook markdown files to Docusaurus
2. Configure sidebar navigation
3. Implement search functionality
4. Verify all content displays correctly

### Milestone 3: Backend RAG System
1. Set up FastAPI server
2. Integrate OpenAI Agents SDK
3. Connect to Qdrant vector database
4. Implement RAG functionality with textbook content
5. Test AI chat responses

### Milestone 4: Frontend-Backend Integration
1. Add chat widget to Docusaurus pages
2. Connect to FastAPI backend
3. Implement real-time chat functionality
4. Test end-to-end chat experience

### Milestone 5: Authentication
1. Integrate BetterAuth
2. Connect to Neon PostgreSQL
3. Implement personalization features
4. Add "Translate to Urdu" functionality
5. Test all authenticated features

## Running Tests
- Frontend: `npm run test`
- Backend: `pytest`

## Deployment to GitHub Pages
1. Build the Docusaurus site: `npm run build`
2. The site will be automatically deployed via GitHub Actions
3. Check GitHub Pages settings for deployment status