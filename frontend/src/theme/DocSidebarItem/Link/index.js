import React from 'react';
import clsx from 'clsx';
import Link from '@theme-original/DocSidebarItem/Link';
import {useCompleted} from '@site/src/utils/progress';

// Ticks completed lessons in the sidebar (styled by .pa-done in custom.css).
export default function LinkWrapper({item, ...props}) {
  const completed = useCompleted();
  const done = item.docId && completed.has(item.docId);
  return <Link {...props} item={done ? {...item, className: clsx(item.className, 'pa-done')} : item} />;
}
