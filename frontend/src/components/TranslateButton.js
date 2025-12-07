import React, { useState } from 'react';
import { useAuth } from '@site/src/hooks/useAuth'; // Assuming we have an auth hook
import ReactMarkdown from 'react-markdown';
import styles from './TranslateButton.module.css';

const TranslateButton = ({ content, onTranslate, targetLanguage = 'ur' }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [showTranslation, setShowTranslation] = useState(false);
  const { user, isAuthenticated, getToken } = useAuth(); // Custom auth hook

  const handleTranslate = async () => {
    if (!content) {
      alert('No content to translate');
      return;
    }

    setIsTranslating(true);
    try {
      let token = null;
      if (isAuthenticated) {
        token = await getToken();
      }

      // Use different endpoints based on authentication status
      let response;
      if (token) {
        // Authenticated user - can use the chapter-specific endpoint
        // Note: This endpoint expects query parameters, not JSON body
        const url = new URL('https://physical-ai-textbook-production-fd94.up.railway.app/api/v1/translation/translate-chapter');
        url.searchParams.append('content', content);
        url.searchParams.append('target_language', targetLanguage);

        response = await fetch(url, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
          },
        });
      } else {
        // Public user - use the public translation endpoint
        // Note: This endpoint expects content as a form field, not in JSON body
        const formData = new FormData();
        formData.append('content', content);

        response = await fetch('https://physical-ai-textbook-production-fd94.up.railway.app/api/v1/translation/translate-to-urdu', {
          method: 'POST',
          body: formData,
        });
      }

      if (response.ok) {
        const result = await response.json();
        setTranslatedContent(result.translated_content);
        setShowTranslation(true);

        if (onTranslate) {
          onTranslate(result.translated_content);
        }
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
  };

  const handleToggleTranslation = () => {
    if (translatedContent) {
      setShowTranslation(!showTranslation);
    }
  };

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
    <div className={styles.translateContainer}>
      <button
        className={styles.translateButton}
        onClick={isTranslating ? null : handleTranslate}
        disabled={isTranslating}
      >
        {isTranslating ? (
          <span>🔄 Translating to {getLanguageName(targetLanguage)}...</span>
        ) : (
          <span>🌐 Translate to {getLanguageName(targetLanguage)}</span>
        )}
      </button>

      {translatedContent && (
        <div className={styles.translationResult}>
          <div className={styles.translationHeader}>
            <h4>Translated Content ({getLanguageName(targetLanguage)}):</h4>
            <button
              className={styles.toggleButton}
              onClick={handleToggleTranslation}
            >
              {showTranslation ? 'Hide' : 'Show'}
            </button>
          </div>

          {showTranslation && (
            <div className={styles.translatedContent}>
              <div className={`${styles.contentBox} ${targetLanguage === 'ur' ? 'translate-button-urdu' : ''}`}>
                <ReactMarkdown>{translatedContent}</ReactMarkdown>
              </div>

              {!isAuthenticated && (
                <div className={styles.loginPrompt}>
                  <p>💡 <strong>Sign in to save translations and get personalized content!</strong></p>
                </div>
              )}
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default TranslateButton;