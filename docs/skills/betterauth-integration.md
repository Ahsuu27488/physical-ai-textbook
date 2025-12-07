# BetterAuth Integration Skill

## Purpose
This skill handles the integration of BetterAuth for user authentication with software and hardware background collection.

## Functionality
- Setup BetterAuth client-side configuration
- Handle user registration with background information
- Manage user login and session
- Provide authentication state to the application

## Implementation Pattern
```javascript
import { createAuthClient } from "better-auth";

const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_BETTERAUTH_URL || "http://localhost:8000",
  fetch: window.fetch,
});

// Registration with background information
const registerWithBackground = async (email, password, name, softwareBackground, hardwareBackground) => {
  const response = await authClient.register({
    email,
    password,
    name,
    additionalData: {
      software_background: softwareBackground,
      hardware_background: hardwareBackground
    }
  });
  return response;
};

// Login
const login = async (email, password) => {
  const response = await authClient.signIn.email({
    email,
    password
  });
  return response;
};
```

## Usage Examples
1. Initialize auth client in the application
2. Use registration function with background collection
3. Handle authentication state in components
4. Protect routes based on authentication status