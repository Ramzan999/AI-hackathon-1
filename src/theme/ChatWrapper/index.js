import React from 'react';
import ChatComponent from '@site/src/components/ChatComponent';

// Theme wrapper component that adds the chatbot to all pages
const ChatWrapper = ({ children }) => {
  return (
    <>
      {children}
      <ChatComponent />
    </>
  );
};

export default ChatWrapper;