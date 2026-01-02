import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatComponent from '@site/src/components/ChatComponent';

const Layout = (props) => {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatComponent />
    </>
  );
};

export default Layout;