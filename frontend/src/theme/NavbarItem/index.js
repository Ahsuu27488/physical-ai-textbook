import React from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import AuthButtons from './AuthButtons';
import PageTools from './PageTools';

// Custom navbar item types used in docusaurus.config.js
export default function NavbarItemWrapper(props) {
  if (props.type === 'custom-AuthButtons') return <AuthButtons {...props} />;
  if (props.type === 'custom-TranslatePersonalizeButtons') return <PageTools {...props} />;
  return <NavbarItem {...props} />;
}
