import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import BrowserOnly from '@docusaurus/BrowserOnly'; // <--- CHANGED THIS

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <BrowserOnly>
        {() => <ChatWidget />}
      </BrowserOnly>
    </>
  );
}