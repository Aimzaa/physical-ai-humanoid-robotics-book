import React from 'react';
import Footer from '@theme-original/Footer';
import type {Props} from '@theme-original/Footer';
import ChatbotWidget from '@site/src/components/ChatbotWidget/ChatbotWidget';

export default function FooterWrapper(props: Props): JSX.Element {
  return (
    <>
      <Footer {...props} />
      <ChatbotWidget />
    </>
  );
}