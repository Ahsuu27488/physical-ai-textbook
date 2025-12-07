import React from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import AuthButtons from './AuthButtons';
// IMPORTS FIXED: Import the translation button component
import TranslatePersonalizeButtons from './TranslatePersonalizeButtons'; 

export default function NavbarItemWrapper(props) {
  // 1. Handle Auth Buttons
  if (props.type === 'custom-AuthButtons') {
    return <AuthButtons {...props} />;
  }

  // 2. FIX: Handle Translation/Personalization Buttons
  if (props.type === 'custom-TranslatePersonalizeButtons') {
    return <TranslatePersonalizeButtons {...props} />;
  }

  // 3. Fallback to default Docusaurus items
  return <NavbarItem {...props} />;
}   