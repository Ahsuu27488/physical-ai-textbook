import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/hooks/useAuth'; // Assuming we have an auth hook
import ReactMarkdown from 'react-markdown';
import styles from './PersonalizeButton.module.css';

// Get backend URL from global config (injected by AppProvider)
const getBackendUrl = () => {
  if (typeof window !== 'undefined' && window.__BACKEND_URL__) {
    return window.__BACKEND_URL__;
  }
  return 'https://ahsuu27488-physical-ai-textbook-backend.hf.space';
};

const PersonalizeButton = ({ chapterId, content, onPersonalize }) => {
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [preferences, setPreferences] = useState({});
  const [showModal, setShowModal] = useState(false);
  const { user, isAuthenticated, getToken } = useAuth(); // Custom auth hook
  const [personalizedContent, setPersonalizedContent] = useState(null);

  // Fetch existing preferences when component mounts
  useEffect(() => {
    if (isAuthenticated && user && chapterId) {
      fetchPreferences();
    }
  }, [isAuthenticated, user, chapterId]);

  const fetchPreferences = async () => {
    try {
      const token = await getToken();
      const response = await fetch(`${getBackendUrl()}/api/v1/personalization/preference/${chapterId}`, {
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();
        setPreferences(data.preferences || {});
      }
    } catch (error) {
      console.error('Error fetching preferences:', error);
    }
  };

  const handlePersonalizeClick = async () => {
    if (!isAuthenticated) {
      alert('Please sign in to personalize content');
      return;
    }

    setShowModal(true);
  };

  const handleSavePreferences = async () => {
    if (!isAuthenticated) return;

    setIsPersonalizing(true);
    try {
      const token = await getToken();

      // Save preferences
      const preferenceResponse = await fetch(`${getBackendUrl()}/api/v1/personalization/preference`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          preferences: preferences,
        }),
      });

      if (preferenceResponse.ok) {
        // Apply personalization to content
        // Note: This endpoint expects content as a query parameter, not in the request body
        const url = new URL(`${getBackendUrl()}/api/v1/personalization/apply/${chapterId}`);
        url.searchParams.append('content', content);

        const applyResponse = await fetch(url, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
          },
        });

        if (applyResponse.ok) {
          const result = await applyResponse.json();
          setPersonalizedContent(result.personalized_content);

          if (onPersonalize) {
            onPersonalize(result.personalized_content);
          }
        }
      }
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('Error personalizing content. Please try again.');
    } finally {
      setIsPersonalizing(false);
      setShowModal(false);
    }
  };

  const updatePreference = (key, value) => {
    setPreferences(prev => ({
      ...prev,
      [key]: value,
    }));
  };

  if (!isAuthenticated) {
    return (
      <div className={styles.personalizeContainer}>
        <button
          className={styles.personalizeButton}
          onClick={() => alert('Please sign in to personalize content')}
        >
          🔧 Personalize Chapter
        </button>
      </div>
    );
  }

  return (
    <div className={styles.personalizeContainer}>
      <button
        className={styles.personalizeButton}
        onClick={handlePersonalizeClick}
        disabled={isPersonalizing}
      >
        {isPersonalizing ? 'Personalizing...' : '🔧 Personalize Chapter'}
      </button>

      {showModal && (
        <div className={styles.modalOverlay} onClick={() => setShowModal(false)}>
          <div className={styles.modalContent} onClick={e => e.stopPropagation()}>
            <h3>Personalize Chapter</h3>
            <div className={styles.preferencesForm}>
              <div className={styles.preferenceItem}>
                <label>
                  <input
                    type="checkbox"
                    checked={preferences.difficulty_level === 'beginner'}
                    onChange={(e) => updatePreference('difficulty_level', e.target.checked ? 'beginner' : 'advanced')}
                  />
                  Simplify for beginners
                </label>
              </div>

              <div className={styles.preferenceItem}>
                <label>
                  <input
                    type="checkbox"
                    checked={preferences.include_examples === true}
                    onChange={(e) => updatePreference('include_examples', e.target.checked)}
                  />
                  Include more examples
                </label>
              </div>

              <div className={styles.preferenceItem}>
                <label>
                  <input
                    type="checkbox"
                    checked={preferences.focus_on_code === true}
                    onChange={(e) => updatePreference('focus_on_code', e.target.checked)}
                  />
                  Focus on code implementation
                </label>
              </div>

              <div className={styles.preferenceItem}>
                <label>
                  <input
                    type="checkbox"
                    checked={preferences.focus_on_theory === true}
                    onChange={(e) => updatePreference('focus_on_theory', e.target.checked)}
                  />
                  Focus on theory
                </label>
              </div>

              <div className={styles.preferenceItem}>
                <label>
                  Software Background:
                  <select
                    value={preferences.software_background || user?.software_background || ''}
                    onChange={(e) => updatePreference('software_background', e.target.value)}
                  >
                    <option value="">Select level</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </label>
              </div>

              <div className={styles.preferenceItem}>
                <label>
                  Hardware Background:
                  <select
                    value={preferences.hardware_background || user?.hardware_background || ''}
                    onChange={(e) => updatePreference('hardware_background', e.target.value)}
                  >
                    <option value="">Select level</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </label>
              </div>
            </div>

            <div className={styles.modalActions}>
              <button className={styles.cancelButton} onClick={() => setShowModal(false)}>
                Cancel
              </button>
              <button className={styles.saveButton} onClick={handleSavePreferences}>
                {isPersonalizing ? 'Saving...' : 'Save & Apply'}
              </button>
            </div>
          </div>
        </div>
      )}

      {personalizedContent && (
        <div className={styles.personalizedPreview}>
          <h4>Personalized Content Preview:</h4>
          <div className={styles.contentPreview}>
            <ReactMarkdown>{personalizedContent}</ReactMarkdown>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;