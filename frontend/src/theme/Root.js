import React from 'react';
import ChatkitWidget from '@site/src/components/ChatkitWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatkitWidget />
    </>
  );
}