# RAG Chatbot Frontend

Frontend implementation for the Physical AI & Humanoid Robotics Book RAG Chatbot using React.

## Architecture Overview
- React-based floating chat component
- Text selection integration
- Docusaurus theme wrapper integration
- Real-time chat interface

## Components

- `ChatComponent.jsx` - Main chat interface with floating button
- Text selection handler
- Context preservation
- Session management

## Integration with Docusaurus

The chat component can be integrated into Docusaurus using a theme wrapper:

1. Create a custom theme component in `src/theme/ChatWrapper/index.js`
2. Wrap the main layout with the chat component
3. The component will automatically detect text selection and provide context

## Features

- Floating chat button that appears on all pages
- Text selection detection with "Ask about this" functionality
- Real-time chat interface
- Session preservation across page navigation
- Context awareness of selected text

## Usage

The `ChatComponent.jsx` can be integrated into any React application or Docusaurus theme wrapper. The component handles:

- Text selection detection
- Message history
- Loading states
- Error handling
- Responsive design

## Styling

The component uses inline styles for simplicity but can be adapted to use CSS modules, styled-components, or other styling approaches as needed.