import React from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

// Rivet's Markdown renderer (GFM tables, task lists, KaTeX). Loaded lazily by the chat, so pages don't download
// it until there is an answer to show.
export default function Markdown({children, components}) {
  return (
    <ReactMarkdown remarkPlugins={[remarkGfm, remarkMath]} rehypePlugins={[rehypeKatex]} components={components}>
      {children}
    </ReactMarkdown>
  );
}
