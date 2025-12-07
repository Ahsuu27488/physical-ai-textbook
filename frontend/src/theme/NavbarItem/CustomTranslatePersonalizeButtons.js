import React, { useState, useEffect, useCallback } from 'react';
import { useAuth } from '@site/src/hooks/useAuth';
import { useLocation } from '@docusaurus/router';
import clsx from 'clsx';
import ReactMarkdown from 'react-markdown';

const TranslatePersonalizeButtons = () => {
  const { isAuthenticated, getToken } = useAuth();
  const location = useLocation();
  const [showDropdown, setShowDropdown] = useState(false);

  // State for translation
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [showTranslationModal, setShowTranslationModal] = useState(false);
  const [targetLanguage, setTargetLanguage] = useState('ur');
  const [contentType, setContentType] = useState(''); // 'translate' or 'personalize'

  // State for personalization
  const [isPersonalizing, setIsPersonalizing] = useState(false);

  // Get current page content for translation/personalization
  const [pageContent, setPageContent] = useState('');

  useEffect(() => {
    // Extract content from the current page - this is a simplified approach
    // In a real implementation, we'd want to get the actual page content
    const titleElement = document.querySelector('h1, .hero__title');
    const contentElements = document.querySelectorAll('article p, .markdown p, .container p');

    let content = '';
    if (titleElement) {
      content += titleElement.textContent + '\n\n';
    }

    contentElements.forEach((el, index) => {
      if (index < 10) { // Limit to first 10 paragraphs to avoid too much content
        content += el.textContent + '\n\n';
      }
    });

    setPageContent(content.substring(0, 2000)); // Limit content length
  }, [location.pathname]);

  const handleTranslate = useCallback(async () => {
    if (!pageContent) {
      alert('No content available to translate on this page');
      return;
    }

    setIsTranslating(true);
    try {
      let token = null;
      if (isAuthenticated) {
        token = await getToken();
      }

      let response;
      if (token) {
        // Authenticated user - use the chapter-specific endpoint
        const url = new URL('http://localhost:8000/api/v1/translation/translate-chapter');
        url.searchParams.append('content', pageContent);
        url.searchParams.append('target_language', targetLanguage);

        response = await fetch(url, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
          },
        });
      } else {
        // Public user - use the public translation endpoint
        const formData = new FormData();
        formData.append('content', pageContent);

        response = await fetch('http://localhost:8000/api/v1/translation/translate-to-urdu', {
          method: 'POST',
          body: formData,
        });
      }

      if (response.ok) {
        const result = await response.json();
        setTranslatedContent(result.translated_content);
        setContentType('translate');
        setShowTranslationModal(true);
      } else {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }
    } catch (error) {
      console.error('Translation error:', error);
      alert(`Translation failed: ${error.message}`);
    } finally {
      setIsTranslating(false);
    }
  }, [pageContent, isAuthenticated, getToken, targetLanguage]);

  const handlePersonalize = useCallback(async () => {
    if (!pageContent) {
      alert('No content available to personalize on this page');
      return;
    }

    setIsPersonalizing(true);
    try {
      if (!isAuthenticated) {
        alert('Please sign in to personalize content');
        return;
      }

      const token = await getToken();

      // Apply personalization to content
      const url = new URL(`http://localhost:8000/api/v1/personalization/apply/${location.pathname.replace(/\//g, '-') || 'home'}`);
      url.searchParams.append('content', pageContent);

      const applyResponse = await fetch(url, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (applyResponse.ok) {
        const result = await applyResponse.json();
        setTranslatedContent(result.personalized_content); // Reusing translation modal to show personalized content
        setContentType('personalize');
        setShowTranslationModal(true);
      } else {
        const errorData = await applyResponse.json();
        throw new Error(errorData.detail || 'Personalization failed');
      }
    } catch (error) {
      console.error('Personalization error:', error);
      alert(`Personalization failed: ${error.message}`);
    } finally {
      setIsPersonalizing(false);
    }
  }, [pageContent, isAuthenticated, getToken, location.pathname]);

  const getLanguageName = (langCode) => {
    const languages = {
      'ur': 'Urdu',
      'es': 'Spanish',
      'fr': 'French',
      'de': 'German',
      'zh': 'Chinese',
      'ja': 'Japanese',
      'ar': 'Arabic',
    };
    return languages[langCode] || langCode.toUpperCase();
  };

  return (
    <>
      <div
        className={`navbar__item dropdown dropdown--right dropdown--translate-personalize ${showDropdown ? 'dropdown--show' : ''}`}
        onMouseEnter={() => setShowDropdown(true)}
        onMouseLeave={() => setShowDropdown(false)}
      >
        <span
          className="navbar__link"
          onClick={() => setShowDropdown(!showDropdown)}
          style={{ cursor: 'pointer' }}
        >
          ⚙️ Tools
        </span>
        {showDropdown && (
          <div className="dropdown__menu">
            <button
              className="dropdown__link"
              onClick={handleTranslate}
              disabled={isTranslating}
            >
              {isTranslating ? 'Translating...' : `🌐 Translate to ${getLanguageName(targetLanguage)}`}
            </button>
            <button
              className="dropdown__link"
              onClick={handlePersonalize}
              disabled={isPersonalizing}
            >
              {isPersonalizing ? 'Personalizing...' : '🔧 Personalize Content'}
            </button>

            {/* Language selection submenu */}
            <div className="dropdown__submenu">
              <div className="dropdown__submenu-title">Translate to:</div>
              {['ur', 'es', 'fr', 'de'].map((lang) => (
                <button
                  key={lang}
                  className={`dropdown__link ${targetLanguage === lang ? 'dropdown__link--active' : ''}`}
                  onClick={(e) => {
                    e.stopPropagation();
                    setTargetLanguage(lang);
                  }}
                >
                  {getLanguageName(lang)}
                </button>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* Translation/Personalization Result Modal */}
      {showTranslationModal && translatedContent && (
        <div className="modal-overlay" style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          backgroundColor: 'rgba(0, 0, 0, 0.5)',
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          zIndex: 1000,
          padding: '20px'
        }}>
          <div className="modal-content" style={{
            backgroundColor: 'var(--ifm-background-color)',
            borderRadius: '8px',
            padding: '20px',
            maxWidth: '800px',
            width: '90%',
            maxHeight: '80vh',
            overflowY: 'auto',
            position: 'relative'
          }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '15px' }}>
              <h3>Translated/Personalized Content</h3>
              <button
                className={clsx('button button--secondary button--sm')}
                onClick={() => setShowTranslationModal(false)}
                style={{ minWidth: 'auto', padding: '5px 10px' }}
              >
                ×
              </button>
            </div>

            <div className={contentType === 'translate' && targetLanguage === 'ur' ? 'translate-button-urdu' : ''} style={{
              padding: '15px',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '4px',
              backgroundColor: 'var(--ifm-background-surface-color)',
              maxHeight: '50vh',
              overflowY: 'auto'
            }}>
              <ReactMarkdown>{translatedContent}</ReactMarkdown>
            </div>

            <div style={{ marginTop: '15px', display: 'flex', gap: '10px' }}>
              <button
                className={clsx('button button--primary')}
                onClick={() => {
                  navigator.clipboard.writeText(translatedContent);
                  alert('Content copied to clipboard!');
                }}
              >
                Copy Content
              </button>
              <button
                className={clsx('button button--secondary')}
                onClick={() => setShowTranslationModal(false)}
              >
                Close
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

const CustomTranslatePersonalizeButtons = (props) => {
  return <TranslatePersonalizeButtons {...props} />;
};

export default CustomTranslatePersonalizeButtons;