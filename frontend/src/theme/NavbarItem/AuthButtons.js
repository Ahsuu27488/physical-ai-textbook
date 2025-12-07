import React, { useState } from 'react';
import { useAuth } from '@site/src/hooks/useAuth';
import clsx from 'clsx';
import styles from './AuthButtons.module.css';

const AuthButtons = (props) => {
  const { user, isAuthenticated, loading, login, register, logout } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [authMode, setAuthMode] = useState('login'); // 'login' or 'register'
  const [loginForm, setLoginForm] = useState({ email: '', password: '' });
  const [registerForm, setRegisterForm] = useState({
    email: '',
    password: '',
    name: '',
    software_background: '',
    hardware_background: ''
  });
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  const handleLogin = async (e) => {
    e.preventDefault();
    setError('');
    const result = await login(loginForm.email, loginForm.password);
    if (!result.success) {
      setError(result.error);
    } else {
      setSuccess('Login successful!');
      setTimeout(() => {
        setShowAuthModal(false);
        setSuccess('');
      }, 1000);
    }
  };

  const handleRegister = async (e) => {
    e.preventDefault();
    setError('');
    const result = await register(registerForm);
    if (!result.success) {
      setError(result.error);
    } else {
      setSuccess('Registration successful! Please login.');
      setAuthMode('login');
      setLoginForm({ email: registerForm.email, password: registerForm.password });
    }
  };

  const handleLogout = async () => {
    logout();
    setSuccess('Logged out successfully!');
    setTimeout(() => setSuccess(''), 1000);
  };

  if (loading) {
    return (
      <div className="navbar__item">
        <button className={clsx('button', 'button--secondary', 'button--sm')} disabled>
          Loading...
        </button>
      </div>
    );
  }

  return (
    <>
      {isAuthenticated ? (
        <div className="navbar__item dropdown dropdown--right dropdown--username">
          <span className="navbar__link">
            👤 {user?.name || user?.email?.split('@')[0]}
          </span>
          <div className="dropdown__menu">
            <button
              className="dropdown__link"
              onClick={handleLogout}
            >
              Logout
            </button>
          </div>
        </div>
      ) : (
        <div className="navbar__item">
          <button
            className={clsx('button', 'button--primary', 'button--sm')}
            onClick={() => setShowAuthModal(true)}
          >
            Sign In
          </button>
        </div>
      )}

      {/* Authentication Modal */}
      {showAuthModal && (
        <div className={styles.modalOverlay}>
          <div className={styles.modal}>
            <div className={styles.modalHeader}>
              <h3>{authMode === 'login' ? 'Sign In' : 'Sign Up'}</h3>
              <button
                className={styles.modalClose}
                onClick={() => setShowAuthModal(false)}
              >
                ×
              </button>
            </div>

            {error && (
              <div className={styles.error}>
                {error}
              </div>
            )}

            {success && (
              <div className={styles.success}>
                {success}
              </div>
            )}

            <div className={styles.modalTabs}>
              <button
                className={clsx(styles.tab, { [styles.active]: authMode === 'login' })}
                onClick={() => setAuthMode('login')}
              >
                Sign In
              </button>
              <button
                className={clsx(styles.tab, { [styles.active]: authMode === 'register' })}
                onClick={() => setAuthMode('register')}
              >
                Sign Up
              </button>
            </div>

            {authMode === 'login' ? (
              <form onSubmit={handleLogin} className={styles.authForm}>
                <div className={styles.formGroup}>
                  <label htmlFor="login-email">Email:</label>
                  <input
                    type="email"
                    id="login-email"
                    value={loginForm.email}
                    onChange={(e) => setLoginForm({...loginForm, email: e.target.value})}
                    required
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="login-password">Password:</label>
                  <input
                    type="password"
                    id="login-password"
                    value={loginForm.password}
                    onChange={(e) => setLoginForm({...loginForm, password: e.target.value})}
                    required
                  />
                </div>

                <button type="submit" className={clsx('button', 'button--primary', 'margin-top--md')}>
                  Sign In
                </button>
              </form>
            ) : (
              <form onSubmit={handleRegister} className={styles.authForm}>
                <div className={styles.formGroup}>
                  <label htmlFor="register-name">Full Name:</label>
                  <input
                    type="text"
                    id="register-name"
                    value={registerForm.name}
                    onChange={(e) => setRegisterForm({...registerForm, name: e.target.value})}
                    required
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="register-email">Email:</label>
                  <input
                    type="email"
                    id="register-email"
                    value={registerForm.email}
                    onChange={(e) => setRegisterForm({...registerForm, email: e.target.value})}
                    required
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="register-password">Password:</label>
                  <input
                    type="password"
                    id="register-password"
                    value={registerForm.password}
                    onChange={(e) => setRegisterForm({...registerForm, password: e.target.value})}
                    required
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="software-background">Software Background:</label>
                  <select
                    id="software-background"
                    value={registerForm.software_background}
                    onChange={(e) => setRegisterForm({...registerForm, software_background: e.target.value})}
                  >
                    <option value="">Select level</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="hardware-background">Hardware Background:</label>
                  <select
                    id="hardware-background"
                    value={registerForm.hardware_background}
                    onChange={(e) => setRegisterForm({...registerForm, hardware_background: e.target.value})}
                  >
                    <option value="">Select level</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </div>

                <button type="submit" className={clsx('button', 'button--primary', 'margin-top--md')}>
                  Sign Up
                </button>
              </form>
            )}
          </div>
        </div>
      )}
    </>
  );
};

export default AuthButtons;