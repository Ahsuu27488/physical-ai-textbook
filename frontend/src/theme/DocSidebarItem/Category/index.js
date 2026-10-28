import React, {useMemo} from 'react';
import clsx from 'clsx';
import Category from '@theme-original/DocSidebarItem/Category';
import {useCompleted} from '@site/src/utils/progress';

// The lesson doc ids inside a sidebar category (module overviews don't count as lessons).
function lessonIds(items, out = []) {
  for (const it of items || []) {
    if (it.type === 'category') lessonIds(it.items, out);
    else if (it.docId && !/(^|\/)index$/.test(it.docId)) out.push(it.docId);
  }
  return out;
}

// Draws a thin progress line under a module in the sidebar once a lesson in it is done (.pa-prog in custom.css).
export default function CategoryWrapper({item, ...props}) {
  const completed = useCompleted();
  const ids = useMemo(() => lessonIds(item.items), [item.items]);
  const done = ids.filter((id) => completed.has(id)).length;
  if (!done || !ids.length) return <Category {...props} item={item} />;
  const tenths = Math.max(1, Math.round((done / ids.length) * 10));
  return <Category {...props} item={{...item, className: clsx(item.className, 'pa-prog', `pa-p-${tenths}`)}} />;
}
