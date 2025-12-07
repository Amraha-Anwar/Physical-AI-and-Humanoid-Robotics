import React from 'react';
import RAGChatWidget from './RAGChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <RAGChatWidget />
    </>
  );
}