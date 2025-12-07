# BetterAuth Skill

## Installation

### Install BetterAuth package:
```bash
npm install better-auth
# or
npm i better-auth
```

## Next.js/React Setup

### 1. Server-side setup (e.g., in `lib/auth.ts`):
```typescript
import { betterAuth } from "better-auth";
import { Pool } from "pg"; // For PostgreSQL

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL
  }),
  // Other configuration options
  socialProviders: {
    // Configure social providers if needed
  }
});
```

### 2. API Route Handler (Next.js - app directory):
Create `app/api/auth/[...all]/route.ts`:
```typescript
import { auth } from "@/lib/auth"; // path to your auth file

export const { GET, POST } = auth.handler;
```

### 3. Client-side setup (e.g., in `lib/auth-client.ts`):
```typescript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_BASE_URL || "http://localhost:3000", // Your Next.js app URL
});
```

### 4. Using in React components:
```typescript
import { authClient } from "@/lib/auth-client";

export function UserComponent() {
  const { data: session, isPending, error, refetch } = authClient.useSession();

  if (isPending) return <div>Loading...</div>;
  if (session) {
    return <div>Welcome {session.user.name}</div>;
  }
  return <div><button onClick={() => authClient.signIn.social({ provider: "google" })}>Sign In</button></div>;
}
```

## Database Connection

### PostgreSQL:
```typescript
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL // e.g., postgresql://user:password@localhost:5432/mydb
  }),
});
```

### Neon Postgres (Serverless PostgreSQL):
Neon connection string format:
```
postgresql://[user]:[password]@[neon_hostname]/[dbname]?sslmode=require
```

Example:
```
postgresql://alex:AbC123dEf@ep-cool-darkness-123456.us-east-2.aws.neon.tech/dbname?sslmode=require
```

With connection pooling (recommended for production):
```
postgresql://alex:AbC123dEf@ep-cool-darkness-123456-pooler.us-east-2.aws.neon.tech/dbname?sslmode=require
```

Environment Variables (.env):
```
DATABASE_URL=postgresql://username:password@ep-xxxxxxx.region.aws.neon.tech/dbname?sslmode=require
```

### MySQL:
```typescript
import { betterAuth } from "better-auth";
import { createPool } from "mysql2/promise";

export const auth = betterAuth({
  database: createPool({
    host: process.env.DB_HOST || "localhost",
    user: process.env.DB_USER || "root",
    password: process.env.DB_PASSWORD || "",
    database: process.env.DB_NAME || "database",
    timezone: "Z", // Important to ensure consistent timezone values
  }),
});
```

### SQLite with Prisma:
```typescript
import { betterAuth } from "better-auth";
import { prismaAdapter } from "better-auth/adapters/prisma";
import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

export const auth = betterAuth({
  database: prismaAdapter(prisma, {
    provider: "sqlite",
  }),
});
```

### Environment Variables (.env):
```
DATABASE_URL=postgresql://username:password@localhost:5432/database_name
NEXT_PUBLIC_BASE_URL=http://localhost:3000
```