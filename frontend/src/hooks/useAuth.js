import { useState, useEffect, createContext, useContext } from 'react';

// Get backend URL from global config (injected by AppProvider)
const getBackendUrl = () => {
  if (typeof window !== 'undefined' && window.__BACKEND_URL__) {
    return window.__BACKEND_URL__;
  }
  return 'https://ahsuu27488-physical-ai-textbook-backend.hf.space';
};

// Create Auth Context
const AuthContext = createContext();

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);

  // Check for existing token on component mount
  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    if (token) {
      // Verify token and get user info
      verifyTokenAndLoadUser(token);
    } else {
      setLoading(false);
    }
  }, []);

  const verifyTokenAndLoadUser = async (token) => {
    try {
      const response = await fetch(`${getBackendUrl()}/api/v1/auth/me`, {
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (response.ok) {
        const userData = await response.json();
        setUser(userData);
        setIsAuthenticated(true);
      } else {
        // Token is invalid, clear it
        localStorage.removeItem('auth_token');
      }
    } catch (error) {
      console.error('Error verifying token:', error);
      localStorage.removeItem('auth_token');
    } finally {
      setLoading(false);
    }
  };

  const login = async (email, password) => {
    try {
      const response = await fetch(`${getBackendUrl()}/api/v1/auth/token`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams({
          username: email,
          password: password,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        const { access_token } = data;

        // Store token in localStorage
        localStorage.setItem('auth_token', access_token);

        // Load user data
        await verifyTokenAndLoadUser(access_token);

        return { success: true };
      } else {
        const errorData = await response.json();
        return { success: false, error: errorData.detail || 'Login failed' };
      }
    } catch (error) {
      return { success: false, error: error.message || 'Network error' };
    }
  };

  const register = async (userData) => {
    try {
      const response = await fetch(`${getBackendUrl()}/api/v1/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(userData),
      });

      if (response.ok) {
        const user = await response.json();
        return { success: true, user };
      } else {
        const errorData = await response.json();
        return { success: false, error: errorData.detail || 'Registration failed' };
      }
    } catch (error) {
      return { success: false, error: error.message || 'Network error' };
    }
  };

  const logout = () => {
    localStorage.removeItem('auth_token');
    setUser(null);
    setIsAuthenticated(false);
  };

  const getToken = async () => {
    const token = localStorage.getItem('auth_token');
    if (token) {
      // Check if token is still valid by attempting to use it
      try {
        const response = await fetch(`${getBackendUrl()}/api/v1/auth/me`, {
          headers: {
            'Authorization': `Bearer ${token}`,
          },
        });

        if (response.ok) {
          return token;
        } else {
          // Token is invalid, clear it
          localStorage.removeItem('auth_token');
          return null;
        }
      } catch (error) {
        return null;
      }
    }
    return null;
  };

  const value = {
    user,
    isAuthenticated,
    loading,
    login,
    register,
    logout,
    getToken,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};