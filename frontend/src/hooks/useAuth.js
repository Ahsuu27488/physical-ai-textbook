import React, { createContext, useCallback, useContext, useEffect, useMemo, useState } from 'react';
import { getBackendUrl } from '@site/src/utils/backendUrl';

const TOKEN_KEY = 'auth_token';

// FastAPI sends `detail` as a string, or as a list of field errors on 422.
const detailText = (detail, fallback) =>
  Array.isArray(detail) ? detail.map((d) => d.msg).join('; ') : detail || fallback;

const readToken = () => {
  try {
    return localStorage.getItem(TOKEN_KEY);
  } catch {
    return null;
  }
};

async function errorOf(response, fallback) {
  try {
    return detailText((await response.json()).detail, fallback);
  } catch {
    return fallback;
  }
}

const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  const logout = useCallback(() => {
    try {
      localStorage.removeItem(TOKEN_KEY);
    } catch {
      // storage blocked
    }
    setUser(null);
  }, []);

  // fetch with the bearer token; a 401 means the session ended, so sign out
  const authFetch = useCallback(
    async (path, options = {}) => {
      const token = readToken();
      if (!token) throw new Error('Sign in first.');
      const response = await fetch(`${getBackendUrl()}${path}`, {
        ...options,
        headers: { ...(options.body ? { 'Content-Type': 'application/json' } : {}), ...options.headers, Authorization: `Bearer ${token}` },
      });
      if (response.status === 401) {
        logout();
        throw new Error('Your session ended. Sign in again.');
      }
      return response;
    },
    [logout],
  );

  const refreshUser = useCallback(async () => {
    const token = readToken();
    if (!token) {
      setLoading(false);
      return null;
    }
    try {
      const response = await fetch(`${getBackendUrl()}/api/v1/auth/me`, { headers: { Authorization: `Bearer ${token}` } });
      if (response.ok) {
        const data = await response.json();
        setUser(data);
        return data;
      }
      if (response.status === 401) logout(); // only a rejected token signs you out, not a network blip
      return null;
    } catch {
      return null;
    } finally {
      setLoading(false);
    }
  }, [logout]);

  useEffect(() => {
    refreshUser();
  }, [refreshUser]);

  const login = useCallback(
    async (email, password) => {
      try {
        const response = await fetch(`${getBackendUrl()}/api/v1/auth/token`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: new URLSearchParams({ username: email, password }),
        });
        if (!response.ok) return { success: false, error: await errorOf(response, 'Sign-in failed.') };
        const { access_token: token } = await response.json();
        localStorage.setItem(TOKEN_KEY, token);
        await refreshUser();
        return { success: true };
      } catch (error) {
        return { success: false, error: error.message || 'Network error' };
      }
    },
    [refreshUser],
  );

  const register = useCallback(async (userData) => {
    try {
      const response = await fetch(`${getBackendUrl()}/api/v1/auth/register`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(userData),
      });
      if (!response.ok) return { success: false, error: await errorOf(response, 'Registration failed.') };
      return { success: true, user: await response.json() };
    } catch (error) {
      return { success: false, error: error.message || 'Network error' };
    }
  }, []);

  const call = useCallback(
    async (path, method, body, fallback) => {
      try {
        const response = await authFetch(path, { method, body: body ? JSON.stringify(body) : undefined });
        if (!response.ok) return { success: false, error: await errorOf(response, fallback) };
        const data = response.status === 204 ? null : await response.json().catch(() => null);
        return { success: true, data };
      } catch (error) {
        return { success: false, error: error.message || 'Network error' };
      }
    },
    [authFetch],
  );

  const updateProfile = useCallback(
    async (fields) => {
      const res = await call('/api/v1/auth/profile', 'PUT', fields, 'Couldn’t save your profile.');
      if (res.success) await refreshUser();
      return res;
    },
    [call, refreshUser],
  );

  const changePassword = useCallback(
    (currentPassword, newPassword) =>
      call('/api/v1/auth/change-password', 'POST', { current_password: currentPassword, new_password: newPassword }, 'Couldn’t change your password.'),
    [call],
  );

  const deleteAccount = useCallback(
    async (password) => {
      const res = await call('/api/v1/auth/account', 'DELETE', { password }, 'Couldn’t delete your account.');
      if (res.success) logout();
      return res;
    },
    [call, logout],
  );

  // kept for older callers: the current token, or null when signed out
  const getToken = useCallback(async () => readToken(), []);

  const value = useMemo(
    () => ({
      user,
      isAuthenticated: Boolean(user),
      loading,
      login,
      register,
      logout,
      getToken,
      authFetch,
      refreshUser,
      updateProfile,
      changePassword,
      deleteAccount,
    }),
    [user, loading, login, register, logout, getToken, authFetch, refreshUser, updateProfile, changePassword, deleteAccount],
  );

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within an AuthProvider');
  return context;
};

// Ask the navbar to open the sign-in dialog from anywhere (AuthButtons listens).
export const OPEN_SIGN_IN = 'pa:open-sign-in';
export const openSignIn = (mode = 'login') => window.dispatchEvent(new CustomEvent(OPEN_SIGN_IN, { detail: { mode } }));
