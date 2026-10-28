import React from 'react';
import {processAdmonitionProps} from '@docusaurus/theme-common';
import Admonition from '@theme-original/Admonition';

// Default titles in sentence case ("Note", not "note" or "NOTE"); custom titles (:::tip My title) are kept.
const TITLES = {note: 'Note', tip: 'Tip', info: 'Info', warning: 'Warning', caution: 'Caution', danger: 'Danger'};

export default function AdmonitionWrapper(unprocessedProps) {
  const props = processAdmonitionProps(unprocessedProps);
  return <Admonition {...props} title={props.title ?? TITLES[props.type]} />;
}
