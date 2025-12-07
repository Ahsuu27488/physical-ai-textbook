# Chat Widget Component

The Chat Widget is an AI-powered assistant that integrates with the Physical AI & Humanoid Robotics textbook platform. It provides students with immediate access to information from the textbook content through a conversational interface.

## Features

- **Floating Chat Interface**: Appears as a floating button in the bottom-right corner of the screen
- **AI-Powered Responses**: Connects to the backend RAG system to provide contextually relevant answers
- **Source Attribution**: Shows which textbook chapters were used to generate responses
- **Real-time Interaction**: Provides immediate responses to student questions
- **Modern UI**: Clean, responsive design with smooth animations
- **Accessibility**: Keyboard navigation and screen reader support

## Installation

The component is already integrated into the Docusaurus layout via the Layout wrapper at `src/theme/Layout/index.js`. No additional installation is required.

## Configuration

### Environment Variables

The chat widget uses the following environment variable:

- `REACT_APP_API_URL`: Base URL for the backend API (defaults to `http://localhost:8000/api`)

## Usage

The chat widget is automatically available on all pages of the textbook. Users can:

1. Click the floating "AI Assistant" button to open the chat
2. Type questions about the textbook content in the input field
3. Press Enter or click the send button to submit questions
4. View AI-generated responses with source attribution
5. Close the chat by clicking the X button or the chat toggle

## API Integration

The widget communicates with the backend through the API client in `src/utils/api.js`:

- `chatApi.sendMessage(message, conversationId)`: Sends a message to the backend
- `chatApi.searchDocuments(query, topK)`: Searches the knowledge base (not currently used in the widget)

## Components

- `ChatWidget.js`: Main React component
- `ChatWidget.css`: Styles for the widget
- `api.js`: API client for backend communication

## Styling

The widget uses a modern, dark-themed design that complements the textbook's academic aesthetic. Key design elements include:

- Gradient header with purple/blue theme
- Smooth animations and transitions
- Responsive design for mobile devices
- Typing indicators during AI processing
- Source attribution for transparency

## Customization

To customize the widget's appearance, modify the CSS in `ChatWidget.css`. Key customization points include:

- Colors: Modify the gradient in `.chat-toggle-button` and `.chat-header`
- Size: Adjust the width and height in `.chat-window`
- Animations: Modify keyframes for floating and slide-in effects

## Accessibility

The widget includes several accessibility features:

- Proper ARIA labels for interactive elements
- Keyboard navigation support
- Focus management
- Semantic HTML structure
- Sufficient color contrast

## Error Handling

The widget gracefully handles various error conditions:

- Network errors during API communication
- Loading states during AI processing
- Error messages for failed requests
- User-friendly error notifications

## Dependencies

- React (part of Docusaurus)
- CSS for styling
- Fetch API for network requests

## Testing

The widget has been tested for:

- Cross-browser compatibility
- Mobile responsiveness
- API integration
- Error handling
- Accessibility compliance