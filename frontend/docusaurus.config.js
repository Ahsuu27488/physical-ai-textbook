// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// One Light / One Dark with comments raised to 4.5:1 contrast (the stock greys are ~2.3:1).
const withComments = (theme, color) => ({
  ...theme,
  styles: theme.styles.map((s) => (s.types.includes('comment') ? {...s, style: {...s.style, color}} : s)),
});

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Advanced textbook platform for Physical AI & Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: process.env.SITE_URL || 'https://physical-ai-textbook-puce-two.vercel.app',
  // Vercel serves the site at '/'; GitHub Pages at '/physical-ai-textbook/' (set in deploy.yml).
  baseUrl: process.env.BASE_URL || '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Ahsuu27488', // Usually your GitHub org/user name.
  projectName: 'physical-ai-textbook', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenAnchors: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Custom environment fields - exposed to client code via siteConfig
  customFields: {
    backendUrl: process.env.BACKEND_URL || 'https://ahsuu27488-physical-ai-textbook-backend.hf.space',
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Ahsuu27488/physical-ai-textbook/tree/main/frontend',
        },
        blog: false, // the sample Docusaurus posts were never replaced
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  // UI fonts are self-hosted (imported in src/theme/Root.js); the Urdu font loads on demand
  // (src/utils/urduFont.js), so no page waits on a third-party stylesheet.

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/logo.png',
      colorMode: {
        defaultMode: 'dark',
        respectPrefersColorScheme: true,
      },
      metadata: [
        {name: 'viewport', content: 'width=device-width, initial-scale=1.0'},
        {name: 'theme-color', content: '#0e1117'},
        {rel: 'icon', href: 'img/favicon.ico'},
        {rel: 'icon', href: 'img/favicon-32x32.png', sizes: '32x32'},
        {rel: 'icon', href: 'img/favicon-16x16.png', sizes: '16x16'},
        {rel: 'apple-touch-icon', href: 'img/apple-touch-icon.png', sizes: '180x180'},
        {rel: 'manifest', href: 'img/site.webmanifest'},
      ],
      navbar: {
        title: 'Physical AI',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Textbook Logo',
          src: 'img/logo-nav.png', // 96px copy of logo.png (500px, 276 KB) for the 28px navbar icon
          width: 28,
          height: 28,
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/Ahsuu27488/physical-ai-textbook',
            label: 'GitHub',
            position: 'right',
          },
          {
            type: 'custom-TranslatePersonalizeButtons',
            position: 'right',
          },
          {
            type: 'custom-AuthButtons',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {label: 'Start here', to: '/docs/'},
              {label: 'Module 1 · ROS 2', to: '/docs/module-1/intro'},
              {label: 'Module 3 · NVIDIA Isaac', to: '/docs/module-3/intro'},
              {label: 'Module 4 · VLA', to: '/docs/module-4/intro'},
            ],
          },
          {
            title: 'Project',
            items: [
              {label: 'GitHub', href: 'https://github.com/Ahsuu27488/physical-ai-textbook'},
              {label: 'Report an issue', href: 'https://github.com/Ahsuu27488/physical-ai-textbook/issues'},
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook`,
      },
      prism: {
        theme: withComments(prismThemes.oneLight, '#5f6672'),
        darkTheme: withComments(prismThemes.oneDark, '#9ba3b4'),
        additionalLanguages: ['bash'],
      },
    }),
};

export default config;
